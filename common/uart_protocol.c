/* ubx.c */

#include "uart_protocol.h"
#include <string.h>

/* UBX checksum: 8-bit Fletcher over CLASS, ID, LEN_L, LEN_H, PAYLOAD... */
static inline void ubx_ck_reset(uint8_t *a, uint8_t *b) { *a = 0; *b = 0; }

static inline void ubx_ck_update(uint8_t *a, uint8_t *b, uint8_t byte)
{
    *a = (uint8_t)((*a + byte) & 0xFFu);
    *b = (uint8_t)((*b + *a) & 0xFFu);
}

static void ubx_ck_update_buf(uint8_t *a, uint8_t *b, const uint8_t *buf, size_t n)
{
    for (size_t i = 0; i < n; i++) ubx_ck_update(a, b, buf[i]);
}

void ubx_parser_init(ubx_parser_t *p,
                     const ubx_dispatch_t *dispatch,
                     size_t dispatch_count,
                     void *user)
{
    memset(p, 0, sizeof(*p));
    p->st = UBX_ST_SYNC1;
    p->dispatch = dispatch;
    p->dispatch_count = dispatch_count;
    p->user = user;
}

static const ubx_dispatch_t *ubx_find(const ubx_parser_t *p, uint8_t cls, uint8_t id)
{
    const ubx_dispatch_t *wildcard = NULL;
    for (size_t i = 0; i < p->dispatch_count; i++) {
        if (p->dispatch[i].cls != cls) continue;
        if (p->dispatch[i].id == id) {
            return &p->dispatch[i];
        }
        if (p->dispatch[i].id == UBX_ID_ANY && !wildcard) {
            wildcard = &p->dispatch[i];
        }
    }
    return wildcard;
}

static ubx_status_t ubx_emit(ubx_parser_t *p)
{
    const ubx_dispatch_t *d = ubx_find(p, p->cls, p->id);
    if (!d) return UBX_OK; /* unknown message: ignore silently */

    if (d->expected_len >= 0 && (uint16_t)d->expected_len != p->len) {
        return UBX_OK; /* length mismatch: ignore or log */
    }

    if (d->handler) {
        d->handler(p->cls, p->id, p->payload, p->len, p->user);
    }

    if (d->typed && d->typed_size > 0) {
        /* Safe casting approach: memcpy into a properly-aligned local buffer */
        /* If typed_size == len (or <= len), copy what you expect. */
        if (p->len >= d->typed_size) {
            /* VLA-free aligned storage */
            uint8_t tmp[UBX_MAX_PAYLOAD];
            if (d->typed_size <= sizeof(tmp)) {
                memcpy(tmp, p->payload, d->typed_size);
                d->typed(p->cls, p->id, tmp, d->typed_size, p->user);
            }
        }
    }

    return UBX_OK;
}

ubx_status_t ubx_parser_feed(ubx_parser_t *p, const uint8_t *data, size_t n)
{
    for (size_t i = 0; i < n; i++) {
        uint8_t b = data[i];

        switch (p->st) {
        case UBX_ST_SYNC1:
            if (b == UBX_SYNC1) p->st = UBX_ST_SYNC2;
            break;

        case UBX_ST_SYNC2:
            if (b == UBX_SYNC2) {
                p->st = UBX_ST_CLASS;
            } else if (b == UBX_SYNC1) {
                /* stay in SYNC2 if we saw another 0xB5 (common resync trick) */
                p->st = UBX_ST_SYNC2;
            } else {
                p->st = UBX_ST_SYNC1;
            }
            break;

        case UBX_ST_CLASS:
            p->cls = b;
            ubx_ck_reset(&p->ck_a, &p->ck_b);
            ubx_ck_update(&p->ck_a, &p->ck_b, b);
            p->st = UBX_ST_ID;
            break;

        case UBX_ST_ID:
            p->id = b;
            ubx_ck_update(&p->ck_a, &p->ck_b, b);
            p->st = UBX_ST_LEN1;
            break;

        case UBX_ST_LEN1:
            p->len = (uint16_t)b;
            ubx_ck_update(&p->ck_a, &p->ck_b, b);
            p->st = UBX_ST_LEN2;
            break;

        case UBX_ST_LEN2:
            p->len |= ((uint16_t)b << 8);
            ubx_ck_update(&p->ck_a, &p->ck_b, b);

            p->pos = 0;

            if (p->len > UBX_MAX_PAYLOAD) {
                /* consume until resync */
                p->st = UBX_ST_SYNC1;
                return UBX_ERR_OVERFLOW;
            }

            p->st = (p->len == 0) ? UBX_ST_CKA : UBX_ST_PAYLOAD;
            break;

        case UBX_ST_PAYLOAD:
            p->payload[p->pos++] = b;
            ubx_ck_update(&p->ck_a, &p->ck_b, b);

            if (p->pos >= p->len) {
                p->st = UBX_ST_CKA;
            }
            break;

        case UBX_ST_CKA:
            if (b != p->ck_a) {
                p->st = UBX_ST_SYNC1;
                return UBX_ERR_CHECKSUM;
            }
            p->st = UBX_ST_CKB;
            break;

        case UBX_ST_CKB:
            if (b != p->ck_b) {
                p->st = UBX_ST_SYNC1;
                return UBX_ERR_CHECKSUM;
            }
            /* Full valid message */
            (void)ubx_emit(p);
            p->st = UBX_ST_SYNC1;
            break;

        default:
            p->st = UBX_ST_SYNC1;
            break;
        }
    }

    return UBX_OK;
}

ubx_status_t ubx_send(ubx_write_fn write_cb, void *user,
                      uint8_t cls, uint8_t id,
                      const void *payload, uint16_t payload_len)
{
    if (!write_cb) return UBX_ERR_BAD_LEN;
    if (payload_len > UBX_MAX_PAYLOAD) return UBX_ERR_OVERFLOW;

    uint8_t hdr[6];
    hdr[0] = UBX_SYNC1;
    hdr[1] = UBX_SYNC2;
    hdr[2] = cls;
    hdr[3] = id;
    hdr[4] = (uint8_t)(payload_len & 0xFFu);
    hdr[5] = (uint8_t)((payload_len >> 8) & 0xFFu);

    uint8_t ck_a, ck_b;
    ubx_ck_reset(&ck_a, &ck_b);

    /* checksum excludes sync bytes; includes cls,id,lenL,lenH,payload */
    ubx_ck_update(&ck_a, &ck_b, hdr[2]);
    ubx_ck_update(&ck_a, &ck_b, hdr[3]);
    ubx_ck_update(&ck_a, &ck_b, hdr[4]);
    ubx_ck_update(&ck_a, &ck_b, hdr[5]);

    if (payload_len && payload) {
        ubx_ck_update_buf(&ck_a, &ck_b, (const uint8_t *)payload, payload_len);
    }

    uint8_t ck[2] = { ck_a, ck_b };

    /* Send header, payload, checksum */
    write_cb(hdr, sizeof(hdr), user);
    if (payload_len && payload) write_cb((const uint8_t *)payload, payload_len, user);
    write_cb(ck, sizeof(ck), user);

    return UBX_OK;
}
