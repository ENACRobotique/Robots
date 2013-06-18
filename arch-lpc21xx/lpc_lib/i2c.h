#ifndef _I2C_H
#define _I2C_H

#include <stdint.h>

#include "i2c0.h"

#ifdef I2C_TRANSACTION_QUEUE_LEN
# define MAX_SEND_I2C_CHUNKS (I2C_TRANSACTION_QUEUE_LEN)
#else
# define MAX_SEND_I2C_CHUNKS (8)
#endif
#define MAX_RECV_I2C_CHUNKS (8)
#ifdef I2C_BUF_LEN
# define MAX_CHUNK_SIZE (I2C_BUF_LEN)
#else
# define MAX_CHUNK_SIZE (64)
#endif

void i2c_init(unsigned int speed, uint8_t addr);
int i2c_sendchunk(uint8_t addr, const uint8_t *data, uint8_t len);
int i2c_recvchunk(uint8_t *addr, uint8_t *data, uint8_t size);

#endif

