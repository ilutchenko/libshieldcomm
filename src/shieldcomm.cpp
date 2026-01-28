#include "shieldcomm/shieldcomm.hpp"
#include "uart_protocol.h"

#include <atomic>
#include <cerrno>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <chrono>

#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

namespace shieldcomm {

static speed_t baud_to_speed(int baud) {
    switch (baud) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
#ifdef B460800
        case 460800: return B460800;
#endif
#ifdef B921600
        case 921600: return B921600;
#endif
        default: return 0;
    }
}

static bool set_termios_raw(int fd, int baud, std::string* err) {
    speed_t spd = baud_to_speed(baud);
    if (spd == 0) {
        if (err) *err = "Unsupported baud rate: " + std::to_string(baud);
        return false;
    }

    termios tty{};
    if (tcgetattr(fd, &tty) != 0) {
        if (err) *err = std::string("tcgetattr failed: ") + std::strerror(errno);
        return false;
    }

    cfmakeraw(&tty);

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag |= (CLOCAL | CREAD);

#ifdef CRTSCTS
    tty.c_cflag &= ~CRTSCTS;
#endif

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);

    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;

    if (cfsetispeed(&tty, spd) != 0 || cfsetospeed(&tty, spd) != 0) {
        if (err) *err = std::string("cfset*speed failed: ") + std::strerror(errno);
        return false;
    }

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        if (err) *err = std::string("tcsetattr failed: ") + std::strerror(errno);
        return false;
    }

    tcflush(fd, TCIOFLUSH);
    return true;
}

static inline uint8_t ubx_id_from_frame(const uint8_t* frame, size_t len) {
    // Convention: if frame looks like [addr, cmd, ...], use cmd as UBX id
    if (frame && len >= 2) return frame[1];
    return UBX_DATA_ID_RAW;
}

static SasEventType ubx_cls_to_sas_evt(uint8_t cls) {
    switch (cls) {
        case UBX_DATA_CLASS_HOST_POLL_R:        return SasEventType::SAS_EVT_HOST_POLL_R;
        case UBX_DATA_CLASS_HOST_POLL_SMG:      return SasEventType::SAS_EVT_HOST_POLL_SMG;
        case UBX_DATA_CLASS_EGM_RESP:           return SasEventType::SAS_EVT_EGM_RESP;
        case UBX_DATA_CLASS_BUSY:               return SasEventType::SAS_EVT_BUSY;
        case UBX_DATA_CLASS_ACK:                return SasEventType::SAS_EVT_ACK;
        case UBX_DATA_CLASS_NACK:               return SasEventType::SAS_EVT_NACK;
        case UBX_DATA_CLASS_GP_EXCEPTION:       return SasEventType::SAS_EVT_GP_EXCEPTION;
        case UBX_DATA_CLASS_CHIRP:              return SasEventType::SAS_EVT_CHIRP;
        case UBX_DATA_CLASS_EGM_EVENT:          return SasEventType::SAS_EVT_EGM_EVENT;
        case UBX_DATA_CLASS_HOST_GENERAL_POLL:  return SasEventType::SAS_EVT_HOST_GENERAL_POLL;
        case UBX_DATA_CLASS_BAD_CRC:            return SasEventType::SAS_EVT_BAD_CRC;
        case UBX_DATA_CLASS_FRAME_ERR:          return SasEventType::SAS_EVT_FRAME_ERR;
        default:                                return SasEventType::SAS_EVT_UNKNOWN;
    }
}

struct ShieldComm::Impl {
    int fd = -1;
    Options opt{};
    ShieldComm* owner = nullptr;

    std::atomic<bool> running{false};
    std::thread rx_thread;

    std::mutex tx_mtx;

    SasCallback sas_cb;
    SasReplyCallback sas_reply_cb;
    DebugCallback debug_cb;
    ServiceCallback service_cb;

    // ----- FD API -----
    int app_fd = -1;   // returned to application
    int lib_fd = -1;   // owned by library
    std::atomic<bool> fd_running{false};
    std::thread fd_thread;

    struct PendingReq {
        enum class Kind : uint8_t { None = 0, HostGP, HostR, HostSMG };

        Kind kind = Kind::None;
        uint8_t addr = 0;  // 7-bit SAS addr as sent (without wakeup bit)
        uint8_t cmd  = 0;  // SAS command (for R/SMG)
        uint64_t seq = 0;
        std::chrono::steady_clock::time_point ts{};
    };

    std::mutex pending_mtx;
    PendingReq pending;
    uint64_t pending_seq = 0;


    ubx_parser_t parser{};
    std::vector<ubx_dispatch_t> dispatch;

    struct TxCtx {
        int fd;
        bool ok;
        std::string* err;
    };

    void fd_loop() {
        std::vector<uint8_t> buf(UBX_MAX_PAYLOAD);

        while (fd_running.load(std::memory_order_relaxed)) {
            pollfd pfd{};
            pfd.fd = lib_fd;
            pfd.events = POLLIN;

            int pr = ::poll(&pfd, 1, 200);
            if (!fd_running.load(std::memory_order_relaxed)) break;
            if (pr < 0) {
                if (errno == EINTR) continue;
                break;
            }
            if (pr == 0) continue;
            if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) break;

            if (pfd.revents & POLLIN) {
                // One recv = one packet = one SAS frame (SEQPACKET preserves boundaries)
                ssize_t n = ::recv(lib_fd, buf.data(), buf.size(), 0);
                if (n < 0) {
                    if (errno == EINTR || errno == EAGAIN) continue;
                    break;
                }
                if (n == 0) break;

                size_t len = static_cast<size_t>(n);
                const uint8_t* frame = buf.data();

                // Map "classic serial write" to UBX TX.
                // Treat each packet as a complete SAS frame.
                std::string err;
                if (owner) {
                    if (len == 1) {
                        (void)owner->send_host_general_poll(frame[0], &err);
                    } else if (len == 2) {
                        (void)owner->send_host_poll_r(frame[0], frame[1], &err);
                    } else {
                        (void)owner->send_host_poll_smg(frame, len, &err);
                    }
                } else {
                    err = "owner not set";
                }

                // Non-fatal error reporting
                if (!err.empty()) {
                    if (auto cb = debug_cb) cb(std::string("[fd_api] tx error: ") + err);
                }
            }
        }
    }

    void fd_api_write_to_app(const uint8_t* data, size_t len) {
        if (lib_fd < 0) return;
        if (!data || len == 0) return;

        // Best-effort nonblocking send.
        // If app doesn't read fast enough, packets may be dropped.
        (void)::send(lib_fd, data, len, MSG_DONTWAIT);
    }

    void set_pending(PendingReq::Kind kind, uint8_t addr, uint8_t cmd) {
        std::lock_guard<std::mutex> lk(pending_mtx);
        pending.kind = kind;
        pending.addr = static_cast<uint8_t>(addr & 0x7Fu);
        pending.cmd  = cmd;
        pending.seq  = ++pending_seq;
        pending.ts   = std::chrono::steady_clock::now();
    }

    void clear_pending(uint64_t seq) {
        std::lock_guard<std::mutex> lk(pending_mtx);
        if (pending.kind != PendingReq::Kind::None && pending.seq == seq) {
            pending = PendingReq{};
        }
    }

    bool match_pending_and_get_seq(const SasEvent& ev, uint64_t* out_seq) {
        PendingReq p;
        {
            std::lock_guard<std::mutex> lk(pending_mtx);
            p = pending;
        }
        if (p.kind == PendingReq::Kind::None) return false;

        // Simple “freshness” guard (optional, but useful to avoid matching stale pending)
        // Keep it small: typical SAS reply is fast; adjust if needed.
        auto age = std::chrono::steady_clock::now() - p.ts;
        if (age > std::chrono::seconds(2)) return false;

        auto addr_from_payload0 = [&]() -> uint8_t {
            if (ev.payload.empty()) return 0xFF;
            return static_cast<uint8_t>(ev.payload[0] & 0x7Fu);
        };

        bool ok = false;

        switch (p.kind) {
            case PendingReq::Kind::HostR: {
                // Primary expected: EGM_RESP with matching cmd (=ubx_id) and matching addr in payload[0]
                if (ev.type == SasEventType::SAS_EVT_EGM_RESP &&
                    ev.ubx_id == p.cmd &&
                    addr_from_payload0() == p.addr) {
                    ok = true;
                }

                // Optional: some firmwares may answer NACK/ACK/BUSY at addr level
                if (!ok && (ev.type == SasEventType::SAS_EVT_ACK ||
                            ev.type == SasEventType::SAS_EVT_NACK ||
                            ev.type == SasEventType::SAS_EVT_BUSY)) {
                    if (addr_from_payload0() == p.addr) ok = true;
                }
            } break;

            case PendingReq::Kind::HostSMG: {
                // If response carries data: EGM_RESP cmd matches
                if (ev.type == SasEventType::SAS_EVT_EGM_RESP &&
                    ev.ubx_id == p.cmd &&
                    addr_from_payload0() == p.addr) {
                    ok = true;
                }
                // If response is just ACK/NACK/BUSY: match addr
                if (!ok && (ev.type == SasEventType::SAS_EVT_ACK ||
                            ev.type == SasEventType::SAS_EVT_NACK ||
                            ev.type == SasEventType::SAS_EVT_BUSY)) {
                    if (addr_from_payload0() == p.addr) ok = true;
                }
            } break;

            case PendingReq::Kind::HostGP: {
                // General poll response can be “GP exception” or “EGM event” (RTE 0xFF…) depending on mode.
                if (ev.type == SasEventType::SAS_EVT_GP_EXCEPTION ||
                    ev.type == SasEventType::SAS_EVT_EGM_EVENT) {
                    ok = true;
                }
                // Также иногда можно ловить BAD_CRC/FRAME_ERR как “ответ” на send,
                // но это обычно не то, что хочет пользователь.
            } break;

            default: break;
        }

        if (!ok) return false;
        if (out_seq) *out_seq = p.seq;
        return true;
    }

    static void ubx_write_all(const uint8_t* data, size_t len, void* user) {
        auto* ctx = static_cast<TxCtx*>(user);
        if (!ctx->ok) return;

        const uint8_t* p = data;
        size_t left = len;

        while (left > 0) {
            ssize_t n = ::write(ctx->fd, p, left);
            if (n < 0) {
                if (errno == EINTR) continue;
                if (errno == EAGAIN) {
                    pollfd pfd{};
                    pfd.fd = ctx->fd;
                    pfd.events = POLLOUT;
                    (void)::poll(&pfd, 1, 200);
                    continue;
                }
                ctx->ok = false;
                if (ctx->err) *ctx->err = std::string("write failed: ") + std::strerror(errno);
                return;
            }
            p += n;
            left -= static_cast<size_t>(n);
        }
    }

    static void on_ubx_debug(uint8_t cls, uint8_t id,
                             const uint8_t* payload, uint16_t payload_len,
                             void* user) {
        (void)cls; (void)id;
        auto* self = static_cast<Impl*>(user);

        DebugCallback cb = self->debug_cb;
        if (!cb) return;

        std::string s;
        if (payload && payload_len) {
            s.assign(reinterpret_cast<const char*>(payload),
                     reinterpret_cast<const char*>(payload) + payload_len);
        }
        cb(s);
    }

    static void on_ubx_service(uint8_t cls, uint8_t id,
                               const uint8_t* payload, uint16_t payload_len,
                               void* user) {
        (void)cls; (void)id;
        auto* self = static_cast<Impl*>(user);

        ServiceCallback cb = self->service_cb;
        if (!cb) return;

        // Firmware contract: payload = [cmd][data...]
        if (!payload || payload_len < 1) return;

        const uint8_t cmd = payload[0];
        const uint8_t* data = payload + 1;
        const uint16_t data_len = static_cast<uint16_t>(payload_len - 1);

        if (cmd == static_cast<uint8_t>(ServiceCommand::ReadSerial)) {
            ServiceSerial ev{};
            ev.serial.assign(reinterpret_cast<const char*>(data),
                             reinterpret_cast<const char*>(data) + data_len);
            cb(ev);
            return;
        }

        if (cmd == static_cast<uint8_t>(ServiceCommand::ReadFirmwareVersion)) {
            if (data_len < 2) return;
            ServiceFirmwareVersion ev{};
            ev.major = data[0];
            ev.minor = data[1];
            cb(ev);
            return;
        }
    }

    static void on_ubx_data(uint8_t cls, uint8_t id,
                            const uint8_t* payload, uint16_t payload_len,
                            void* user) {
        auto* self = static_cast<Impl*>(user);

        SasEvent ev{};
        ev.type = ubx_cls_to_sas_evt(cls);
        ev.ubx_id = id;
        if (payload && payload_len) {
            ev.payload.assign(payload, payload + payload_len);
        }

        // 1) “raw stream” callback (как сейчас)
        if (auto cb = self->sas_cb) cb(ev);

        // 2) “reply-only” callback
        
        uint64_t seq = 0;
        if (self->match_pending_and_get_seq(ev, &seq)) {
            if (auto rcb = self->sas_reply_cb) {
                rcb(ev);
            }
            self->fd_api_write_to_app(ev.payload.data(), ev.payload.size());
            // Clear pending on “final” responses; keep pending on BUSY (optional)
            if (ev.type == SasEventType::SAS_EVT_BUSY) {
                // keep pending
            } else {
                self->clear_pending(seq);
            }
        
        }

        // 3) FD API stream (legacy read())
    }


    void build_dispatch() {
        dispatch.clear();
        dispatch.reserve(32);

        // Service response
        dispatch.push_back(ubx_dispatch_t{
            .cls = UBX_APP_CLASS,
            .id = UBX_APP_ID_SERVICE,
            .expected_len = -1,
            .handler = &Impl::on_ubx_service,
            .typed = nullptr,
            .typed_size = 0
        });

        // Debug stream
        dispatch.push_back(ubx_dispatch_t{
            .cls = UBX_DATA_CLASS_DEBUG,
            .id = UBX_ID_ANY,
            .expected_len = -1,
            .handler = &Impl::on_ubx_debug,
            .typed = nullptr,
            .typed_size = 0
        });

        // All SAS-related data classes use the same handler
        const uint8_t classes[] = {
            UBX_DATA_CLASS_HOST_POLL_R,
            UBX_DATA_CLASS_HOST_POLL_SMG,
            UBX_DATA_CLASS_EGM_RESP,
            UBX_DATA_CLASS_BUSY,
            UBX_DATA_CLASS_ACK,
            UBX_DATA_CLASS_NACK,
            UBX_DATA_CLASS_GP_EXCEPTION,
            UBX_DATA_CLASS_CHIRP,
            UBX_DATA_CLASS_EGM_EVENT,
            UBX_DATA_CLASS_HOST_GENERAL_POLL,
            UBX_DATA_CLASS_BAD_CRC,
            UBX_DATA_CLASS_FRAME_ERR,
        };

        for (uint8_t c : classes) {
            dispatch.push_back(ubx_dispatch_t{
                .cls = c,
                .id = UBX_ID_ANY,
                .expected_len = -1,
                .handler = &Impl::on_ubx_data,
                .typed = nullptr,
                .typed_size = 0
            });
        }

        ubx_parser_init(&parser, dispatch.data(), dispatch.size(), this);
    }

    void rx_loop() {
        std::vector<uint8_t> buf(512);

        while (running.load(std::memory_order_relaxed)) {
            pollfd pfd{};
            pfd.fd = fd;
            pfd.events = POLLIN;

            int pr = ::poll(&pfd, 1, 200);
            if (!running.load(std::memory_order_relaxed)) break;

            if (pr < 0) {
                if (errno == EINTR) continue;
                running.store(false, std::memory_order_relaxed);
                break;
            }
            if (pr == 0) continue;

            if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
                running.store(false, std::memory_order_relaxed);
                break;
            }

            if (pfd.revents & POLLIN) {
                ssize_t n = ::read(fd, buf.data(), buf.size());
                if (n < 0) {
                    if (errno == EAGAIN || errno == EINTR) continue;
                    running.store(false, std::memory_order_relaxed);
                    break;
                }
                if (n == 0) {
                    running.store(false, std::memory_order_relaxed);
                    break;
                }

                (void)ubx_parser_feed(&parser, buf.data(), static_cast<size_t>(n));
                // UBX parser resyncs on errors; caller can optionally report errors in firmware via UBX_DATA_CLASS_*.
            }
        }
    }

    Status send_ubx_data(uint8_t ubx_cls,
                         const uint8_t* payload, size_t len,
                         uint8_t ubx_id,
                         std::string* err) {
        if (fd < 0) {
            if (err) *err = "Port not open";
            return Status::NotOpen;
        }
        if (!payload && len != 0u) {
            if (err) *err = "Null data with non-zero length";
            return Status::InvalidArg;
        }
        if (len > UBX_MAX_PAYLOAD) {
            if (err) *err = "Payload too large for UBX_MAX_PAYLOAD";
            return Status::IoError;
        }

        std::lock_guard<std::mutex> lk(tx_mtx);
        TxCtx ctx{ .fd = fd, .ok = true, .err = err };

        ubx_status_t st = ubx_send(&Impl::ubx_write_all, &ctx,
                                  ubx_cls, ubx_id,
                                  payload, static_cast<uint16_t>(len));
        if (st != UBX_OK) {
            if (err && err->empty()) *err = "ubx_send failed";
            return Status::IoError;
        }
        if (!ctx.ok) return Status::IoError;
        return Status::Ok;
    }
};

ShieldComm::ShieldComm() : p_(new Impl) {
    p_->owner = this;
}
ShieldComm::~ShieldComm() {
    close();
    delete p_;
}

bool ShieldComm::open(const Options& opt, std::string* err) {
    if (is_open()) {
        if (err) *err = "Already open";
        return false;
    }

    int fd = ::open(opt.device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        if (err) *err = std::string("open failed: ") + std::strerror(errno);
        return false;
    }

    std::string terr;
    if (!set_termios_raw(fd, opt.baud, &terr)) {
        ::close(fd);
        if (err) *err = terr;
        return false;
    }

    p_->fd = fd;
    p_->opt = opt;

    p_->build_dispatch();

    p_->running.store(true, std::memory_order_relaxed);
    p_->rx_thread = std::thread([this]() { p_->rx_loop(); });

    return true;
}

void ShieldComm::close() {
    if (!is_open()) return;

    disable_fd_api();

    p_->running.store(false, std::memory_order_relaxed);

    int fd = p_->fd;
    p_->fd = -1;
    ::close(fd);

    if (p_->rx_thread.joinable()) {
        p_->rx_thread.join();
    }
}

bool ShieldComm::is_open() const {
    return p_ && p_->fd >= 0;
}

void ShieldComm::set_sas_callback(SasCallback cb) {
    p_->sas_cb = std::move(cb);
}

void ShieldComm::set_sas_reply_callback(SasReplyCallback cb) {
    p_->sas_reply_cb = std::move(cb);
}

void ShieldComm::set_debug_callback(DebugCallback cb) {
    p_->debug_cb = std::move(cb);
}

void ShieldComm::set_service_callback(ServiceCallback cb) {
    p_->service_cb = std::move(cb);
}

Status ShieldComm::enable_fd_api(std::string* err) {
    if (!is_open()) {
        if (err) *err = "Port not open";
        return Status::NotOpen;
    }
    if (p_->app_fd >= 0 || p_->lib_fd >= 0) {
        if (err) *err = "FD API already enabled";
        return Status::InvalidArg;
    }

    int sv[2] = {-1, -1};
    // SEQPACKET preserves write boundaries
    if (::socketpair(AF_UNIX, SOCK_SEQPACKET, 0, sv) != 0) {
        if (err) *err = std::string("socketpair failed: ") + std::strerror(errno);
        return Status::IoError;
    }

    // Make both ends nonblocking (optional but recommended)
    for (int i = 0; i < 2; ++i) {
        int fl = ::fcntl(sv[i], F_GETFL, 0);
        if (fl >= 0) (void)::fcntl(sv[i], F_SETFL, fl | O_NONBLOCK);
    }

    p_->app_fd = sv[0];
    p_->lib_fd = sv[1];

    p_->fd_running.store(true, std::memory_order_relaxed);
    p_->fd_thread = std::thread([this]() { p_->fd_loop(); });

    return Status::Ok;
}

void ShieldComm::disable_fd_api() {
    if (!p_) return;

    p_->fd_running.store(false, std::memory_order_relaxed);

    int a = p_->app_fd;
    int b = p_->lib_fd;
    p_->app_fd = -1;
    p_->lib_fd = -1;

    if (a >= 0) ::close(a);
    if (b >= 0) ::close(b);

    if (p_->fd_thread.joinable()) p_->fd_thread.join();
}

int ShieldComm::get_fd() const {
    if (!p_) return -1;
    return p_->app_fd;
}

ssize_t ShieldComm::fd_read(void* buf, size_t len) {
    if (!p_ || p_->app_fd < 0) {
        errno = EBADF;
        return -1;
    }
    return ::read(p_->app_fd, buf, len);
}

ssize_t ShieldComm::fd_write(const void* buf, size_t len) {
    if (!p_ || p_->app_fd < 0) {
        errno = EBADF;
        return -1;
    }
    return ::write(p_->app_fd, buf, len);
}

// ---------------- TX: one function per SAS event ----------------

// Host -> EGM

Status ShieldComm::send_host_general_poll(uint8_t addr_with_wakeup, std::string* err) {
    uint8_t b = addr_with_wakeup;

    // NEW: pending request (addr without wakeup bit)
    p_->set_pending(Impl::PendingReq::Kind::HostGP, static_cast<uint8_t>(addr_with_wakeup & 0x7Fu), 0);

    Status st = p_->send_ubx_data(UBX_DATA_CLASS_HOST_GENERAL_POLL, &b, 1, UBX_DATA_ID_RAW, err);
    if (st != Status::Ok) {
        // clear only if this pending is still ours (best-effort)
        // simplest: just clear unconditionally
        std::lock_guard<std::mutex> lk(p_->pending_mtx);
        p_->pending = Impl::PendingReq{};
    }
    return st;
}

Status ShieldComm::send_host_poll_r(uint8_t addr, uint8_t cmd, std::string* err) {
    uint8_t fr[2] = { addr, cmd };

    // NEW pending
    p_->set_pending(Impl::PendingReq::Kind::HostR, addr, cmd);

    Status st = p_->send_ubx_data(UBX_DATA_CLASS_HOST_POLL_R, fr, sizeof(fr), cmd, err);
    if (st != Status::Ok) {
        std::lock_guard<std::mutex> lk(p_->pending_mtx);
        p_->pending = Impl::PendingReq{};
    }
    return st;
}

Status ShieldComm::send_host_poll_smg(const uint8_t* frame, size_t len, std::string* err) {
    uint8_t addr = 0;
    uint8_t cmd  = 0;
    if (frame && len >= 2) {
        addr = frame[0];
        cmd  = frame[1];
    }

    // NEW pending
    p_->set_pending(Impl::PendingReq::Kind::HostSMG, addr, cmd);

    Status st = p_->send_ubx_data(UBX_DATA_CLASS_HOST_POLL_SMG, frame, len, ubx_id_from_frame(frame, len), err);
    if (st != Status::Ok) {
        std::lock_guard<std::mutex> lk(p_->pending_mtx);
        p_->pending = Impl::PendingReq{};
    }
    return st;
}


// EGM -> Host (simulation/injection/testing)

Status ShieldComm::send_egm_resp(const uint8_t* frame, size_t len, std::string* err) {
    return p_->send_ubx_data(UBX_DATA_CLASS_EGM_RESP, frame, len, ubx_id_from_frame(frame, len), err);
}

Status ShieldComm::send_egm_event(const uint8_t* frame, size_t len, std::string* err) {
    return p_->send_ubx_data(UBX_DATA_CLASS_EGM_EVENT, frame, len, ubx_id_from_frame(frame, len), err);
}

// EGM signals

Status ShieldComm::send_busy(uint8_t addr, std::string* err) {
    // Per your definition: busy is (addr, 00)
    uint8_t fr[2] = { addr, 0x00 };
    return p_->send_ubx_data(UBX_DATA_CLASS_BUSY, fr, sizeof(fr), UBX_DATA_ID_RAW, err);
}

Status ShieldComm::send_ack(uint8_t addr, std::string* err) {
    uint8_t fr[1] = { addr };
    return p_->send_ubx_data(UBX_DATA_CLASS_ACK, fr, sizeof(fr), UBX_DATA_ID_RAW, err);
}

Status ShieldComm::send_nack(uint8_t addr, std::string* err) {
    uint8_t fr[2] = { addr, static_cast<uint8_t>(0x80u | (addr & 0x7Fu)) };
    return p_->send_ubx_data(UBX_DATA_CLASS_NACK, fr, sizeof(fr), UBX_DATA_ID_RAW, err);
}

Status ShieldComm::send_gp_exception(uint8_t code, std::string* err) {
    uint8_t fr[1] = { code };
    return p_->send_ubx_data(UBX_DATA_CLASS_GP_EXCEPTION, fr, sizeof(fr), UBX_DATA_ID_RAW, err);
}

Status ShieldComm::send_chirp(uint8_t addr_with_wakeup, std::string* err) {
    uint8_t fr[1] = { addr_with_wakeup };
    return p_->send_ubx_data(UBX_DATA_CLASS_CHIRP, fr, sizeof(fr), UBX_DATA_ID_RAW, err);
}

// Service

Status ShieldComm::send_service(ServiceCommand cmd, std::string* err) {
    if (!is_open()) {
        if (err) *err = "Port not open";
        return Status::NotOpen;
    }

    uint8_t c = static_cast<uint8_t>(cmd);

    std::lock_guard<std::mutex> lk(p_->tx_mtx);
    Impl::TxCtx ctx{ .fd = p_->fd, .ok = true, .err = err };

    ubx_status_t st = ubx_send(&Impl::ubx_write_all, &ctx,
                              UBX_APP_CLASS, UBX_APP_ID_SERVICE,
                              &c, 1);
    if (st != UBX_OK) {
        if (err && err->empty()) *err = "ubx_send failed";
        return Status::IoError;
    }
    if (!ctx.ok) return Status::IoError;
    return Status::Ok;
}

} // namespace shieldcomm
