# libshieldcomm

`libshieldcomm` — небольшая C++ библиотека для обмена SAS-сообщениями через UART с вашим STM32-шилдом. На линии UART библиотека использует **UBX-фрейминг** (см. `uart_protocol.h`), а внутри UBX payload переносит «сырые» SAS байты.

Библиотека:
- открывает UART (`/dev/ttyAMA3` по умолчанию) в raw-режиме;
- поднимает RX-поток, который парсит UBX и вызывает callbacks;
- предоставляет набор TX-функций “one function per SAS event” для отправки сообщений на шилд;
- поддерживает “service” команды (например, чтение serial и версии прошивки).

---

## Быстрый старт

```cpp
#include <iostream>
#include "shieldcomm/shieldcomm.hpp"

static void print_hex(const std::vector<uint8_t>& v) {
  for (auto b : v) std::printf("%02X ", b);
  std::printf("\n");
}

int main() {
  shieldcomm::ShieldComm sc;

  sc.set_debug_callback([](const std::string& s) {
    std::cerr << "[DBG] " << s << "\n";
  });

  sc.set_service_callback([](const shieldcomm::ServiceEvent& ev) {
    std::visit([](auto&& e) {
      using T = std::decay_t<decltype(e)>;
      if constexpr (std::is_same_v<T, shieldcomm::ServiceSerial>) {
        std::cerr << "[SERVICE] serial: " << e.serial << "\n";
      } else if constexpr (std::is_same_v<T, shieldcomm::ServiceFirmwareVersion>) {
        std::cerr << "[SERVICE] fw: " << int(e.major) << "." << int(e.minor) << "\n";
      } else if constexpr (std::is_same_v<T, shieldcomm::ServiceProxyStatus>) {
        std::cerr << "[SERVICE] host-egm proxy: " << (e.enabled ? "ON" : "OFF") << "\n";
      }
    }, ev);
  });

  sc.set_sas_callback([](const shieldcomm::SasEvent& ev) {
    std::cerr << "[SAS] type=" << int(ev.type)
              << " ubx_id=0x" << std::hex << int(ev.ubx_id) << std::dec
              << " len=" << ev.payload.size() << " data=";
    print_hex(ev.payload);
  });

  shieldcomm::Options opt;
  opt.device = "/dev/ttyAMA3";
  opt.baud   = 115200;

  std::string err;
  if (!sc.open(opt, &err)) {
    std::cerr << "open failed: " << err << "\n";
    return 1;
  }

  // Пример: запрос сервисной информации
  sc.send_service(shieldcomm::ServiceCommand::ReadSerial, &err);
  sc.send_service(shieldcomm::ServiceCommand::ReadFirmwareVersion, &err);

  // Пример: отправить Host General Poll
  // addr_with_wakeup: адрес EGM с wakeup-битом (если нужен)
  sc.send_host_general_poll(0x01, &err);

  // Держим процесс живым, чтобы принимать RX
  std::cerr << "running...\n";
  for (;;) std::this_thread::sleep_for(std::chrono::seconds(1));
}
````

---

## Сборка и линковка через CMake (библиотека как сабмодуль)

### Пример CmakeLists.txt
```
cmake_minimum_required(VERSION 3.16)
project(my_app CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

FetchContent_Declare(
  libshieldcomm
  GIT_REPOSITORY https://github.com/ilutchenko/libshieldcomm
  GIT_TAG        v0.1   # или конкретный tag/commit для воспроизводимости
)

# Важно: библиотека содержит собственный CMakeLists.txt, поэтому делаем доступной как подпроект
FetchContent_MakeAvailable(libshieldcomm)

add_executable(my_app main.cpp)
target_link_libraries(my_app PRIVATE shieldcomm_static) # STATIC
# target_link_libraries(my_app PRIVATE shieldcomm)        # SHARED
```

---

### Права на UART

Если используете `/dev/ttyAMA3`, часто нужны права группы `dialout` (или аналогичной на вашей системе):

```bash
ls -l /dev/ttyAMA3
sudo usermod -aG dialout $USER
# затем перелогиниться
```


## Концепция данных

### UBX транспорт

UART несёт **UBX-сообщения**. Библиотека:

* принимает байты из UART;
* парсит UBX (`ubx_parser_feed`);
* в зависимости от `cls/id` маршрутизирует payload в callbacks.

### SasEvent

```cpp
struct SasEvent {
    SasEventType type = SasEventType::SAS_EVT_UNKNOWN;
    uint8_t ubx_id = 0;                 // UBX "id" (обычно SAS cmd, иначе RAW)
    std::vector<uint8_t> payload;       // сырые SAS bytes внутри UBX payload
};
```

* `type` — тип события (класс UBX → `SasEventType`).
* `ubx_id` — обычно равен SAS-команде (cmd), если payload выглядит как `[addr, cmd, ...]`. Иначе ставится RAW.
* `payload` — «как есть» SAS-байты (без UBX заголовка/CRC UBX).

---

## API (подробно)

### Простые типы

#### `Options`

```cpp
struct Options {
    std::string device = "/dev/ttyAMA3";
    int baud = 115200;
};
```

#### `Status`

```cpp
enum class Status {
    Ok = 0,
    NotOpen,
    IoError,
    Timeout,
    InvalidArg,
};
```

### Сервисные команды (Service)

#### `ServiceCommand`

```cpp
enum class ServiceCommand : uint8_t {
    ReadSerial = 0x01,
    ReadFirmwareVersion = 0x02,
    SetHostEgmProxy = 0x03,
    GetHostEgmProxy = 0x04,
};
```

#### Сервисные события

```cpp
struct ServiceSerial { std::string serial; };
struct ServiceFirmwareVersion { uint8_t major = 0, minor = 0; };
struct ServiceProxyStatus { bool enabled = false; };
using ServiceEvent = std::variant<ServiceSerial, ServiceFirmwareVersion, ServiceProxyStatus>;
```

**Контракт**: ответ от прошивки приходит как UBX payload = `[cmd][data...]`.

---

## Класс `ShieldComm`

### Жизненный цикл и состояние порта

```cpp
bool open(const Options& opt, std::string* err = nullptr);
void close();
bool is_open() const;
```

* `open()` открывает UART (non-blocking), настраивает termios raw, запускает RX-thread и UBX-dispatch.
* `close()` останавливает RX-thread и закрывает fd.
* `open()` вернёт `false` и заполнит `err`, если:

  * порт уже открыт;
  * ошибка `open()/tcsetattr()` и т.п.;
  * неподдерживаемый baud (внутри есть таблица допустимых скоростей).

### Коллбеки

```cpp
using SasCallback = std::function<void(const SasEvent&)>;
using SasReplyCallback = std::function<void(const SasEvent&)>;
using DebugCallback = std::function<void(const std::string&)>;
using ServiceCallback = std::function<void(const ServiceEvent&)>;

void set_sas_callback(SasCallback cb);
void set_sas_reply_callback(SasReplyCallback cb);
void set_debug_callback(DebugCallback cb);
void set_service_callback(ServiceCallback cb);
```

Важно:

* callbacks вызываются из RX-потока библиотеки (внутренний `rx_thread`).
* если внутри callback вы трогаете UI/общие структуры — обеспечьте потокобезопасность (mutex/queue/dispatch в главный поток).
* `set_sas_reply_callback()` вызывается только для событий, которые матчатся к текущему pending host-запросу.

---

## FD API (виртуальный сериал)

FD API позволяет общаться с библиотекой через file descriptor (как с обычным портом),
но внутри используется `socketpair(AF_UNIX, SOCK_SEQPACKET)`, поэтому **границы пакетов сохраняются**.
Один `write()` == один SAS-фрейм, один `read()` == один SAS-фрейм.

```cpp
Status enable_fd_api(std::string* err = nullptr);
void disable_fd_api();
int get_fd() const; // fd приложения, или -1 если API выключен

ssize_t fd_read(void* buf, size_t len);
ssize_t fd_write(const void* buf, size_t len);
```

Контракт записи (TX):

* `len == 1` → Host General Poll (`[addr|wakeup]`)
* `len == 2` → Host Poll R (`[addr, cmd]`)
* `len >= 3` → Host Poll SMG (`[addr, cmd, ...]`)

Контракт чтения (RX):

* `read()` возвращает **ровно один** SAS-фрейм, как он пришёл от прошивки.
* Для ACK/NACK/BUSY/CHIRP/GP_EXCEPTION вернётся их payload (например, `[addr]` или `[addr, 0x80|addr]`).

Примечания:

* fd делается non-blocking, поэтому `read()` может вернуть `-1` с `EAGAIN`.
* если приложение не успевает читать, часть пакетов может быть отброшена (best-effort).

---

## Отправка (TX): функции “one function per SAS event”

Все TX-методы возвращают `Status` и могут заполнить `err`.

### Host → EGM

#### 1) Host General Poll (1 байт)

```cpp
Status send_host_general_poll(uint8_t addr_with_wakeup, std::string* err = nullptr);
```

Payload: `[addr_with_wakeup]`.

#### 2) Host Poll R (2 байта: addr+cmd)

```cpp
Status send_host_poll_r(uint8_t addr, uint8_t cmd, std::string* err = nullptr);
```

Payload: `[addr, cmd]`.
`ubx_id` для UBX сообщения = `cmd`.

#### 3) Host Poll SMG (переменная длина)

```cpp
Status send_host_poll_smg(const uint8_t* frame, size_t len, std::string* err = nullptr);
Status send_host_poll_smg(const std::vector<uint8_t>& frame, std::string* err = nullptr);
```

Payload: `frame[0..len-1]`.
Если `len >= 2`, то `ubx_id = frame[1]` (как “cmd”), иначе RAW.

### Блокирующие TX-варианты (send + wait)

Для host-запросов доступны блокирующие методы, которые отправляют фрейм и ждут
соответствующий ответ (или timeout):

```cpp
Status send_host_general_poll_wait(uint8_t addr_with_wakeup,
                                   int timeout_ms,
                                   SasEvent* out_reply,
                                   std::string* err = nullptr);

Status send_host_poll_r_wait(uint8_t addr,
                             uint8_t cmd,
                             int timeout_ms,
                             SasEvent* out_reply,
                             std::string* err = nullptr);

Status send_host_poll_smg_wait(const uint8_t* frame,
                               size_t len,
                               int timeout_ms,
                               SasEvent* out_reply,
                               std::string* err = nullptr);
Status send_host_poll_smg_wait(const std::vector<uint8_t>& frame,
                               int timeout_ms,
                               SasEvent* out_reply,
                               std::string* err = nullptr);
```

Контракт:

* `Status` отражает результат отправки/ожидания.
* `Status::Ok` — получен matched-ответ (в `out_reply`, если указатель не `nullptr`).
* `Status::Timeout` — timeout ожидания ответа.
* `Status::InvalidArg`, `Status::NotOpen`, `Status::IoError` — ошибки аргументов/порта/IO.
* `BUSY` не завершает ожидание: wait продолжается до “финального” ответа или timeout.
* в один момент времени поддерживается один blocking wait на экземпляр `ShieldComm`.
* matched-ответ продолжает приходить и в `set_sas_reply_callback()` (если callback установлен).
* timeout передаётся как `int` в миллисекундах; внутри библиотека конвертирует его в `std::chrono::milliseconds`.

### EGM → Host (симуляция / инъекция / тесты)

```cpp
Status send_egm_resp(const uint8_t* frame, size_t len, std::string* err = nullptr);
Status send_egm_event(const uint8_t* frame, size_t len, std::string* err = nullptr);
```

### «Одиночные» сигналы EGM

#### Busy

```cpp
Status send_busy(uint8_t addr, std::string* err = nullptr);
```

Payload: `[addr, 0x00]`.

#### ACK

```cpp
Status send_ack(uint8_t addr, std::string* err = nullptr);
```

Payload: `[addr]`.

#### NACK

```cpp
Status send_nack(uint8_t addr, std::string* err = nullptr);
```

Payload: `[addr, 0x80 | (addr & 0x7F)]`.

#### GP Exception

```cpp
Status send_gp_exception(uint8_t code, std::string* err = nullptr);
```

Payload: `[code]`.

#### Chirp

```cpp
Status send_chirp(uint8_t addr_with_wakeup, std::string* err = nullptr);
```

Payload: `[addr_with_wakeup]`.


### Service (к прошивке)

```cpp
Status send_service(ServiceCommand cmd, std::string* err = nullptr);
```

Отправляет UBX сообщение класса `UBX_APP_CLASS` / `UBX_APP_ID_SERVICE` с payload `[cmd]`.
Ответ ловится в `set_service_callback()` как `ServiceEvent`.

---

## Типы входящих SAS-событий (`SasEventType`)

```cpp
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
```

Коротко:

* `HOST_*` — то, что пришло от хоста в сторону EGM (как событие, если прошивка “эхом” сообщает хосту).
* `EGM_RESP` / `EGM_EVENT` — ответы/ивенты автомата.
* `ACK/NACK/BUSY/CHIRP/GP_EXCEPTION` — спец-сигналы.

---

## Примеры использования

### Пример 1: Отправить R-полл и ждать ответ

```cpp
shieldcomm::ShieldComm sc;
std::string err;

sc.set_sas_callback([&](const shieldcomm::SasEvent& ev) {
  if (ev.type == shieldcomm::SasEventType::SAS_EVT_EGM_RESP && ev.ubx_id == 0x54) {
    // Ответ на R-команду 0x54
    // ev.payload: сырые SAS bytes (как прислали)
  }
});

shieldcomm::Options opt{"/dev/ttyAMA3", 115200};
if (!sc.open(opt, &err)) { /* обработать */ }

// отправим R: [addr, cmd]
auto st = sc.send_host_poll_r(/*addr*/0x01, /*cmd*/0x54, &err);
if (st != shieldcomm::Status::Ok) { /* обработать */ }
```

### Пример 2: Запросить serial + версию прошивки

```cpp
sc.set_service_callback([](const shieldcomm::ServiceEvent& ev) {
  std::visit([](auto&& e) {
    using T = std::decay_t<decltype(e)>;
    if constexpr (std::is_same_v<T, shieldcomm::ServiceSerial>)
      std::printf("SERIAL: %s\n", e.serial.c_str());
    if constexpr (std::is_same_v<T, shieldcomm::ServiceFirmwareVersion>)
      std::printf("FW: %u.%u\n", e.major, e.minor);
    if constexpr (std::is_same_v<T, shieldcomm::ServiceProxyStatus>)
      std::printf("HOST-EGM PROXY: %s\n", e.enabled ? "ON" : "OFF");
  }, ev);
});

sc.send_service(shieldcomm::ServiceCommand::ReadSerial);
sc.send_service(shieldcomm::ServiceCommand::ReadFirmwareVersion);
```

### Пример 3: Логирование входящих событий по типам

```cpp
static const char* type_to_str(shieldcomm::SasEventType t) {
  switch (t) {
    case shieldcomm::SasEventType::SAS_EVT_ACK: return "ACK";
    case shieldcomm::SasEventType::SAS_EVT_NACK: return "NACK";
    case shieldcomm::SasEventType::SAS_EVT_BUSY: return "BUSY";
    case shieldcomm::SasEventType::SAS_EVT_EGM_RESP: return "EGM_RESP";
    default: return "OTHER";
  }
}

sc.set_sas_callback([](const shieldcomm::SasEvent& ev) {
  std::fprintf(stderr, "[%s] id=0x%02X len=%zu\n",
               type_to_str(ev.type), ev.ubx_id, ev.payload.size());
});
```

### Пример 4: Чтение через `fd_read()` (FD API)

```cpp
shieldcomm::ShieldComm sc;
std::string err;

shieldcomm::Options opt{"/dev/ttyAMA3", 115200};
if (!sc.open(opt, &err)) { /* обработать */ }

if (sc.enable_fd_api(&err) != shieldcomm::Status::Ok) { /* обработать */ }

// Отправим R-полл через fd API (addr, cmd)
uint8_t rpoll[2] = {0x01, 0x54};
if (sc.fd_write(rpoll, sizeof(rpoll)) < 0) { /* обработать */ }

// Прочитаем один фрейм ответа
uint8_t buf[256];
ssize_t n = sc.fd_read(buf, sizeof(buf));
if (n > 0) {
  // buf[0..n-1] — SAS-фрейм (как пришёл)
} else {
  // n < 0: errno (EAGAIN, EBADF, ...)
}
```

### Пример 5: Блокирующий `send_host_poll_r_wait()`

```cpp
shieldcomm::ShieldComm sc;
std::string err;

shieldcomm::Options opt{"/dev/ttyAMA3", 115200};
if (!sc.open(opt, &err)) { /* обработать */ }

shieldcomm::SasEvent reply;
auto st = sc.send_host_poll_r_wait(/*addr*/0x01,
                                   /*cmd*/0x54,
                                   200, // timeout_ms
                                   &reply,
                                   &err);
if (st == shieldcomm::Status::Timeout) {
  // timeout
} else if (st != shieldcomm::Status::Ok) {
  // ошибка отправки/IO/порта
} else {
  // Получен matched-ответ на 0x54
  // reply.payload: сырой SAS-фрейм
}
```

---

## Потокобезопасность и модель исполнения

* RX выполняется в отдельном потоке (`rx_thread`).
* TX защищён mutex’ом (`tx_mtx`), поэтому несколько потоков могут вызывать `send_*`, но:

  * ваши callbacks тоже вызываются из RX-потока, поэтому избегайте блокировать надолго.
* `close()` останавливает RX-поток, закрывает fd и делает `join()`.
* host-запросы сериализуются; одновременно допустим только один blocking wait на экземпляр.

Рекомендуемый паттерн для приложений с UI:

* в `set_sas_callback()` складывать события в thread-safe очередь;
* в UI/main loop доставать события и обновлять интерфейс.

---

## Типовые проблемы

1. **`open failed: Permission denied`**

* добавьте пользователя в `dialout` (или нужную группу), перелогиньтесь.

2. **Нет данных / тишина**

* проверьте, что устройство и baud совпадают с прошивкой;
* убедитесь, что на линии действительно UBX-фрейминг, а не “голый” ASCII/сырые SAS байты.

3. **`Unsupported baud rate`**

* библиотека поддерживает ограниченный набор baud (9600..230400, возможно 460800/921600 если определены макросы в системе).

---

## Справка по соответствию классов UBX → SasEventType

Внутри `shieldcomm.cpp` классы UBX (например `UBX_DATA_CLASS_EGM_RESP`) мапятся на `SasEventType::SAS_EVT_EGM_RESP`, и для всех “SAS data classes” используется один и тот же обработчик `on_ubx_data()`.

Если вы добавляете новые UBX классы на прошивке, их нужно:

1. добавить в `ubx_cls_to_sas_evt()`;
2. добавить в список `classes[]` внутри `build_dispatch()`.

---
