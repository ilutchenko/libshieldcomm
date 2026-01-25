#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <variant>
#include <vector>

namespace shieldcomm {

struct Options {
    std::string device = "/dev/ttyAMA3";
    int baud = 115200;
};

enum class Status {
    Ok = 0,
    NotOpen,
    IoError,
    InvalidArg,
};

enum class ServiceCommand : uint8_t {
    ReadSerial = 0x01,
    ReadFirmwareVersion = 0x02,
};

enum class SasEventType : uint8_t {
    SAS_EVT_HOST_POLL_R = 0,
    SAS_EVT_HOST_POLL_SMG,
    SAS_EVT_EGM_RESP,
    SAS_EVT_BUSY,
    SAS_EVT_ACK,
    SAS_EVT_NACK,
    SAS_EVT_GP_EXCEPTION,
    SAS_EVT_CHIRP,
    SAS_EVT_EGM_EVENT,
    SAS_EVT_HOST_GENERAL_POLL,
    SAS_EVT_BAD_CRC,
    SAS_EVT_FRAME_ERR,
    SAS_EVT_UNKNOWN,
};

struct SasEvent {
    SasEventType type = SasEventType::SAS_EVT_UNKNOWN;
    uint8_t ubx_id = 0;                 // UBX "id" (often SAS cmd, else RAW)
    std::vector<uint8_t> payload;       // raw SAS bytes inside UBX payload
};

struct ServiceSerial { std::string serial; };
struct ServiceFirmwareVersion { uint8_t major = 0, minor = 0; };
using ServiceEvent = std::variant<ServiceSerial, ServiceFirmwareVersion>;

class ShieldComm {
public:
    using SasCallback = std::function<void(const SasEvent&)>;
    using DebugCallback = std::function<void(const std::string&)>;
    using ServiceCallback = std::function<void(const ServiceEvent&)>;

    ShieldComm();
    ~ShieldComm();

    ShieldComm(const ShieldComm&) = delete;
    ShieldComm& operator=(const ShieldComm&) = delete;

    bool open(const Options& opt, std::string* err = nullptr);
    void close();
    bool is_open() const;

    void set_sas_callback(SasCallback cb);
    void set_debug_callback(DebugCallback cb);
    void set_service_callback(ServiceCallback cb);

    // ---------- TX: one function per SAS event ----------
    // Host -> EGM
    Status send_host_general_poll(uint8_t addr_with_wakeup, std::string* err = nullptr); // 1 byte
    Status send_host_poll_r(uint8_t addr, uint8_t cmd, std::string* err = nullptr);     // 2 bytes
    Status send_host_poll_smg(const uint8_t* frame, size_t len, std::string* err = nullptr);
    Status send_host_poll_smg(const std::vector<uint8_t>& frame, std::string* err = nullptr) {
        return send_host_poll_smg(frame.data(), frame.size(), err);
    }

    // EGM -> Host (useful for simulation/injection/testing)
    Status send_egm_resp(const uint8_t* frame, size_t len, std::string* err = nullptr);
    Status send_egm_resp(const std::vector<uint8_t>& frame, std::string* err = nullptr) {
        return send_egm_resp(frame.data(), frame.size(), err);
    }
    Status send_egm_event(const uint8_t* frame, size_t len, std::string* err = nullptr);
    Status send_egm_event(const std::vector<uint8_t>& frame, std::string* err = nullptr) {
        return send_egm_event(frame.data(), frame.size(), err);
    }

    // “Single/tiny” EGM signals
    Status send_busy(uint8_t addr, std::string* err = nullptr);          // payload: [addr, 0x00]
    Status send_ack(uint8_t addr, std::string* err = nullptr);           // payload: [addr]
    Status send_nack(uint8_t addr, std::string* err = nullptr);          // payload: [addr, 0x80|addr]
    Status send_gp_exception(uint8_t code, std::string* err = nullptr);  // payload: [code]
    Status send_chirp(uint8_t addr_with_wakeup, std::string* err = nullptr); // payload: [addr|wakeup]

    // Service
    Status send_service(ServiceCommand cmd, std::string* err = nullptr);

private:
    struct Impl;
    Impl* p_;
};

} // namespace shieldcomm
