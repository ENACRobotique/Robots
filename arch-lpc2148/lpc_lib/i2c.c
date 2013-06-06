#include <string.h>

#include "i2c.h"

#ifndef MIN
# define MIN(a, b) ((a)>(b)?(b):(a))
#endif

struct i2c_transaction send_buffer[MAX_SEND_I2C_CHUNKS];

struct i2c_transaction recv_buffer[MAX_RECV_I2C_CHUNKS];
uint8_t recv_extract_idx = 0, recv_insert_idx = 0;

void _i2c_handler(struct i2c_transaction *t, void *userp) {
    uint8_t tmp;

    if(t->len_r > 0) {
        tmp = (recv_insert_idx + 1)%MAX_RECV_I2C_CHUNKS;
        if(tmp == recv_extract_idx)
            return;

        memcpy(recv_buffer[recv_insert_idx].buf, t->buf, t->len_r);
        recv_buffer[recv_insert_idx].slave_addr = t->slave_addr;
        recv_buffer[recv_insert_idx].len_r = t->len_r;

        recv_insert_idx = tmp;
    }
}

void i2c_init(unsigned int speed, uint8_t addr) {
    int i;

    recv_extract_idx = 0;
    recv_insert_idx = 0;

    for(i = 0; i < MAX_SEND_I2C_CHUNKS; i++)
        send_buffer[i].status = I2CTransSuccess;

    i2c0_init(speed, addr, _i2c_handler, NULL);
}

int i2c_sendchunk(uint8_t addr, const uint8_t *data, uint8_t len) {
    int i;

    if(!data || len > MAX_CHUNK_SIZE)
        return -1;

    for(i = 0; i < MAX_SEND_I2C_CHUNKS; i++)
        if(send_buffer[i].status == I2CTransSuccess)
            break;

    if(i == MAX_SEND_I2C_CHUNKS)
        return -1;

    memcpy(send_buffer[i].buf, data, len);
    send_buffer[i].type = I2CTransTx;
    send_buffer[i].status = I2CTransPending;
    send_buffer[i].slave_addr = addr;
    send_buffer[i].len_r = 0;
    send_buffer[i].len_w = len;

    if(i2c0_submit(&send_buffer[i]))
        return -1;

    return len;
}

int i2c_recvchunk(uint8_t *addr, uint8_t *data, uint8_t size) {
    uint8_t len;

    if(!data)
        return -1;

    if(recv_extract_idx == recv_insert_idx)
        return 0; // no data received

    len = recv_buffer[recv_extract_idx].len_r;

    memcpy(data, recv_buffer[recv_extract_idx].buf, MIN(size, len));
    if(addr)
        *addr = recv_buffer[recv_extract_idx].slave_addr;

    recv_extract_idx = (recv_extract_idx + 1)%MAX_RECV_I2C_CHUNKS;

    return len;
}

