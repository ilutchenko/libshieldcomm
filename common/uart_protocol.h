/* ubx.h - UBX protocol parser/sender skeleton */

#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define UBX_SYNC1 0xB5u
#define UBX_SYNC2 0x62u
#define UBX_ID_ANY 0xFFu

/* Application-specific UBX classes and IDs. */
#define UBX_APP_CLASS        0xF0u
#define UBX_APP_ID_TEXT      0x01u
#define UBX_APP_ID_SERVICE   0x02u
#define UBX_DATA_CLASS_HOST_POLL_R        0xF1u
#define UBX_DATA_CLASS_HOST_POLL_SMG      0xF2u
#define UBX_DATA_CLASS_EGM_RESP           0xF3u
#define UBX_DATA_CLASS_BUSY               0xF4u
#define UBX_DATA_CLASS_ACK                0xF5u
#define UBX_DATA_CLASS_NACK               0xF6u
#define UBX_DATA_CLASS_GP_EXCEPTION       0xF7u
#define UBX_DATA_CLASS_CHIRP              0xF8u
#define UBX_DATA_CLASS_EGM_EVENT          0xF9u
#define UBX_DATA_CLASS_HOST_GENERAL_POLL  0xFAu
#define UBX_DATA_CLASS_BAD_CRC            0xFBu
#define UBX_DATA_CLASS_FRAME_ERR          0xFCu
#define UBX_DATA_CLASS_DEBUG              0xFDu
#define UBX_DATA_ID_RAW                   0x01u

#define UBX_SVC_READ_SERIAL          0x01u
#define UBX_SVC_READ_FIRMWARE_VER    0x02u

/* Choose a max payload you expect (NAV-PVT is 92 bytes, some can be larger). */
#ifndef UBX_MAX_PAYLOAD
#define UBX_MAX_PAYLOAD 512u
#endif

typedef enum {
    UBX_OK = 0,
    UBX_ERR_CHECKSUM,
    UBX_ERR_OVERFLOW,
    UBX_ERR_BAD_LEN,
} ubx_status_t;

/* Callback for sending bytes (UART TX) */
typedef void (*ubx_write_fn)(const uint8_t *data, size_t len, void *user);

/* Handler called when a full valid UBX message arrives */
typedef void (*ubx_handler_fn)(uint8_t cls, uint8_t id,
                              const uint8_t *payload, uint16_t payload_len,
                              void *user);

/* Optional: strongly-typed handler (payload already copied into struct buffer) */
typedef void (*ubx_typed_handler_fn)(uint8_t cls, uint8_t id,
                                    const void *msg_struct, size_t struct_len,
                                    void *user);

/* Dispatch entry: if expected_len != -1, length must match. */
typedef struct {
    uint8_t cls;
    uint8_t id;
    int16_t expected_len;        /* -1 => variable length allowed */
    ubx_handler_fn handler;      /* raw payload handler */
    ubx_typed_handler_fn typed;  /* typed handler (optional) */
    size_t typed_size;           /* size of struct to memcpy into before typed() */
} ubx_dispatch_t;

typedef struct {
    /* Parser state */
    enum {
        UBX_ST_SYNC1 = 0,
        UBX_ST_SYNC2,
        UBX_ST_CLASS,
        UBX_ST_ID,
        UBX_ST_LEN1,
        UBX_ST_LEN2,
        UBX_ST_PAYLOAD,
        UBX_ST_CKA,
        UBX_ST_CKB
    } st;

    uint8_t cls;
    uint8_t id;
    uint16_t len;
    uint16_t pos;

    uint8_t ck_a, ck_b;
    uint8_t payload[UBX_MAX_PAYLOAD];

    /* Dispatch table */
    const ubx_dispatch_t *dispatch;
    size_t dispatch_count;

    void *user;
} ubx_parser_t;

/* Init parser with dispatch table */
void ubx_parser_init(ubx_parser_t *p,
                     const ubx_dispatch_t *dispatch,
                     size_t dispatch_count,
                     void *user);

/* Feed bytes from UART RX into the parser */
ubx_status_t ubx_parser_feed(ubx_parser_t *p, const uint8_t *data, size_t len);

/* Build+send UBX message (payload is arbitrary bytes) */
ubx_status_t ubx_send(ubx_write_fn write_cb, void *user,
                      uint8_t cls, uint8_t id,
                      const void *payload, uint16_t payload_len);

/* Build+send UBX message where payload is a struct */
#define ubx_send_struct(write_cb, user, cls, id, ptr_struct) \
    ubx_send((write_cb), (user), (cls), (id), (ptr_struct), (uint16_t)sizeof(*(ptr_struct)))

#ifdef __cplusplus
}
#endif
