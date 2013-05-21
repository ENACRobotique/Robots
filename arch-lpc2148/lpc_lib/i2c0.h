#ifndef _I2C0_H
#define _I2C0_H

#include <stdint.h>

enum I2CDeviceStatus {
  I2CDevIdle,
  I2CDevStartRequested,
  I2CDevBusy
};

enum I2CTransType {
  I2CTransTx,
  I2CTransRx,
  I2CTransTxRx
};

enum I2CTransStatus {
  I2CTransPending,
  I2CTransRunning,
  I2CTransSuccess,
  I2CTransFailed,
  I2CTransDone
};

#ifndef I2C_BUF_LEN
#define I2C_BUF_LEN 32
#endif

struct i2c_transaction {
  enum I2CTransType type;
  volatile enum I2CTransStatus status;
  uint8_t slave_addr;
  uint8_t len_r;
  uint8_t len_w;
  uint8_t buf[I2C_BUF_LEN];
  void *userp;
};

#ifndef I2C_TRANSACTION_QUEUE_LEN
#define I2C_TRANSACTION_QUEUE_LEN 8
#endif

typedef void (*i2c_handler)(struct i2c_transaction *t, void *userp);

struct i2c_periph {
  /* master data */
  struct i2c_transaction *trans[I2C_TRANSACTION_QUEUE_LEN]; // circular buffer holding transactions
  uint8_t trans_insert_idx;
  uint8_t trans_extract_idx;
  /* slave data */
  struct i2c_transaction trans_sla;
  i2c_handler handler;
  void *userp;
  uint8_t own_addr;
  /* internal state of the peripheral */
  volatile enum I2CDeviceStatus status;
  volatile uint8_t idx_buf;
};

void i2c0_init(unsigned int speed, uint8_t sla_addr, i2c_handler h, void *userp);
int i2c0_submit(struct i2c_transaction* t);

#endif
