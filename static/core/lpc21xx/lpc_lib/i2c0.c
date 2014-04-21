#include <lpc214x.h>
#include <ime.h>

#include "sys_time.h"
#include "i2c0.h"

//#define I2C_DEBUG
#define I2C_STATS

#ifndef BIT
#define BIT(b) (1<<(b))
#endif

struct i2c_periph p;

#define I2cNextTransaction() do {                                            \
  p.trans_extract_idx = (p.trans_extract_idx + 1)%I2C_TRANSACTION_QUEUE_LEN; \
} while(0)

#define I2cEndOfTransaction() do {              \
  if(p.trans_extract_idx == p.trans_insert_idx) \
    p.status = I2CDevIdle;                      \
  else {                                        \
    p.status = I2CDevStartRequested;            \
    I2C0_CONSET = BIT(5 /*STA*/);                \
  }                                             \
} while(0)

#ifdef I2C_DEBUG
unsigned int i_debug_tab = 0;
struct {
  unsigned int t;
  uint8_t stat;
  uint8_t start_set;
  uint8_t end_set;
  uint8_t extract_idx;
} debug_tab[128];
#endif

#ifdef I2C_STATS
// FIXME only valid when MT&SR modes are used
struct {
    uint32_t nb_unexpected_states;
    uint32_t t_first_err;
    uint32_t t_last_err;

    // master transmitter statistics
    uint32_t nb_MT_trials;
    uint32_t nb_MT_ok;
    uint32_t nb_MT_retry;
    uint32_t nb_MT_err_nack;
    uint32_t nb_MT_err_arblst;
    uint32_t nb_MT_err_bus;
    uint32_t nb_MT_err_buffer_full;

    // slave receiver statistics
    uint32_t nb_SR_trials;
    uint32_t nb_SR_ok;
    uint32_t nb_SR_err_bus;
} stats = {0};
#endif

// TODO reflect failure cases to the state of the transaction
void _i2c0_isr() __attribute__((interrupt("IRQ")));
void _i2c0_isr() { // SI bit is set in I2C0_CONSET => state change
  struct i2c_transaction *trans = p.trans[p.trans_extract_idx];

#ifdef I2C_DEBUG
  uint8_t stat = I2C0_STAT;

  if(i_debug_tab < sizeof(debug_tab)/sizeof(*debug_tab)) {
    debug_tab[i_debug_tab].stat = stat;
    debug_tab[i_debug_tab].t = micros();
    debug_tab[i_debug_tab].start_set = I2C0_CONSET;
    debug_tab[i_debug_tab].extract_idx = p.trans_extract_idx;
  }

  switch(stat) { // get new status
#else
  switch(I2C0_STAT) {
#endif
  case 0x08: // [MT/MR] bus free, start condition emitted, can send slave address
  case 0x10: // [MT/MR] bus free, repeated start condition emitted, can send slave address
    if(trans->type == I2CTransTx || (trans->type == I2CTransTxRx && trans->len_w > 0))
      I2C0_DAT = trans->slave_addr & ~BIT(0); // bit0 is direction (0 is write)
    else
      I2C0_DAT = trans->slave_addr | BIT(0); // bit0 is direction (1 is read)

#ifdef I2C_STATS
    stats.nb_MT_trials++;
#endif
    trans->status = I2CTransRunning;
    p.status = I2CDevBusy;
    I2C0_CONSET = BIT(2 /*AA*/);
    I2C0_CONCLR = BIT(5 /*STAC*/); // clear START bit
    p.idx_buf = 0;
    break;

  case 0x18: // [MT from 08|10] slave_addr+W transmitted, ACK received, transmit first byte
  case 0x28: // [MT] data transmitted, ACK received
    if(p.idx_buf < trans->len_w)
      I2C0_DAT = trans->buf[p.idx_buf++];
    else { // no more bytes to transmit
      if(trans->type == I2CTransTxRx) {
        trans->len_w = 0;
        p.idx_buf = 0;
        p.status = I2CDevStartRequested;
        I2C0_CONSET = BIT(5 /*STA*/);  // ask start condition => repeated start condition
      }
      else {
        I2C0_CONSET = BIT(4 /*STO*/); // ask stop condition
#ifdef I2C_STATS
        stats.nb_MT_ok++;
#endif
        trans->status = I2CTransSuccess;
        I2cNextTransaction();
        I2cEndOfTransaction();
      }
    }
    break;

  // error cases
  case 0x30: // [MT] data byte transmitted ; /ACK received
  case 0x20: // [MT] slave_addr+W transmitted, /ACK received
    I2C0_CONSET = BIT(4 /*STO*/) | BIT(2 /*AA*/);

    if(trans->nb_retry > I2C_MAX_NB_RETRY) {
#ifdef I2C_STATS
      stats.nb_MT_err_nack++;
      if(!stats.t_first_err) stats.t_first_err = micros();
      stats.t_last_err = micros();
#endif
      trans->status = I2CTransFailed;
      I2cNextTransaction();
    }
    else {
#ifdef I2C_STATS
      stats.nb_MT_retry++;
#endif
      trans->nb_retry++;
      trans->status = I2CTransPending;
    }

    I2cEndOfTransaction();
    break;
  case 0x48: // [MR] slave_addr+R transmitted, /ACK received
    I2C0_CONSET = BIT(4 /*STO*/) | BIT(2 /*AA*/);
#ifdef I2C_STATS
    stats.nb_unexpected_states++;
#endif
    trans->status = I2CTransFailed;
    I2cNextTransaction();
    I2cEndOfTransaction();
    break;
  case 0x38: // [MT/MR] slave_addr+R/W | data byte transmitted, arbitration lost
    I2C0_CONSET = BIT(2 /*AA*/);
#ifdef I2C_STATS
    stats.nb_MT_err_arblst++;
    if(!stats.t_first_err) stats.t_first_err = micros();
    stats.t_last_err = micros();
#endif
    trans->status = I2CTransFailed;
    I2cNextTransaction();
    I2cEndOfTransaction();
    break;

  case 0x40: // [MR from 08|10] slave_addr+R transmitted, ACK received
#ifdef I2C_STATS
    stats.nb_unexpected_states++;
#endif
    if (p.idx_buf < trans->len_r - 1) I2C0_CONSET = BIT(2 /*AA*/);  // set AA
    else I2C0_CONCLR = BIT(2 /*AA*/); // clear AA (last character will be received => 0x58)
    break;

  case 0x50: // [MR] data received, ACK transmitted, will receive data
#ifdef I2C_STATS
    stats.nb_unexpected_states++;
#endif
    if (p.idx_buf < trans->len_r) {
      trans->buf[p.idx_buf++] = I2C0_DAT;
      if (p.idx_buf < trans->len_r - 1) I2C0_CONSET = BIT(2 /*AA*/);  // set AA
      else I2C0_CONCLR = BIT(2 /*AA*/); // clear AA (last character will be received => 0x58)
    }
    break;

  case 0x58: // [MR] last data byte received, /ACK transmitted
#ifdef I2C_STATS
    stats.nb_unexpected_states++;
#endif
    if(p.idx_buf < trans->len_r)
      trans->buf[p.idx_buf] = I2C0_DAT;

    I2C0_CONSET = BIT(4 /*STO*/) | BIT(2 /*AA*/);// transmit stop

    trans->status = I2CTransSuccess;
    if(p.handler)
      p.handler(trans, p.userp);
    I2cNextTransaction();
    I2cEndOfTransaction();
    break;


  case 0x60: // [SR] own slave_addr+W received, ACK transmitted
  case 0x68: // [SR] arbitration lost, slave_addr+W received, ACK transmitted
    I2C0_CONSET = BIT(2 /*AA*/);  // send AA
    p.trans_sla.slave_addr = p.own_addr; // TODO use transaction buffer gave by user instead of a local one (avoids a copy...)
    p.trans_sla.status = I2CTransRunning;
    p.trans_sla.type = I2CTransRx;
    p.status = I2CDevBusy;
    p.idx_buf = 0;
#ifdef I2C_STATS
    stats.nb_SR_trials++;
#endif
    break;

  case 0x70: // [SR-GCA] gca+W received, ACK transmitted
  case 0x78: // [SR-GCA] arbitration lost, gca+W received, ACK transmitted
    I2C0_CONSET = BIT(2 /*AA*/);  // send AA
    p.trans_sla.slave_addr = 0;
    p.trans_sla.status = I2CTransRunning;
    p.trans_sla.type = I2CTransRx;
    p.status = I2CDevBusy;
    p.idx_buf = 0;
#ifdef I2C_STATS
    stats.nb_SR_trials++;
#endif
    break;

  case 0x90: // [SR-GCA] data byte received, ACK transmitted
  case 0x80: // [SR] data byte received, ACK transmitted
    I2C0_CONSET = BIT(2 /*AA*/);  // send AA
    p.trans_sla.buf[p.idx_buf++] = I2C0_DAT; // TODO more than I2C_BUF_LEN stop
    break;

  case 0x98: // [SR-GCA] data byte received, /ACK transmitted
  case 0x88: // [SR] data byte received, /ACK transmitted
    // In theory, we do not transmit back /ACK to the master so we shouldn't get those 2 status codes
#ifdef I2C_STATS
    stats.nb_unexpected_states++;
#endif
    I2C0_CONSET = BIT(2 /*AA*/);  // send AA
    I2cEndOfTransaction();
    break;

  case 0xA0: // [SR] no more bytes to get (stop or repeated start condition received)
    I2C0_CONSET = BIT(2 /*AA*/);  // send AA
#ifdef I2C_STATS
    stats.nb_SR_ok++;
#endif
    p.trans_sla.status = I2CTransSuccess;
    p.trans_sla.len_r = p.idx_buf;
    p.trans_sla.len_w = 0;
    p.handler(&p.trans_sla, p.userp);
    p.trans_sla.len_r = 0;
    if(!p.trans_sla.len_w)
      I2cEndOfTransaction();
    break;


  case 0xA8: // [ST] own slave_addr+R received, ACK transmitted
  case 0xB0: // [ST] arbitration lost, own slave_addr+R received, ACK transmitted
#ifdef I2C_STATS
    stats.nb_unexpected_states++;
#endif
    p.idx_buf = 0;

    if(!p.trans_sla.len_w) {
      p.trans_sla.len_r = 0;
      p.trans_sla.status = I2CTransRunning;
      p.trans_sla.type = I2CTransTx;
      p.handler(&p.trans_sla, p.userp);
      p.status = I2CDevBusy;
    }

    if(p.trans_sla.len_w > 0) {
      I2C0_DAT = p.trans_sla.buf[p.idx_buf++];
      if(p.trans_sla.len_w > 1)
        I2C0_CONSET = BIT(2 /*AA*/); // send AA
      else
        I2C0_CONCLR = BIT(2 /*AAC*/); // clear AA
    }
    // TODO else (error case: no data to transmit)
    break;

  case 0xB8: // [ST] data byte transmitted, ACK received, transmit next byte
#ifdef I2C_STATS
    stats.nb_unexpected_states++;
#endif
    I2C0_DAT = p.trans_sla.buf[p.idx_buf++];
    I2C0_CONSET = BIT(2 /*AA*/);  // send AA
    break;

  case 0xC0: // [ST] last data byte transmitted, /ACK received
  case 0xC8: // [ST] last data byte transmitted, ACK received
#ifdef I2C_STATS
    stats.nb_unexpected_states++;
#endif
    p.trans_sla.len_w = 0;
//    p.trans_sla.status = I2cTransSuccess; useless
    I2C0_CONSET = BIT(2 /*AA*/);  // send AA
    I2cEndOfTransaction();
    break;


  // error cases
  default: /* unimplemented case... */
  case 0xF8: // shouldn't happen, we are interrupts driven
#ifdef I2C_STATS
    stats.nb_unexpected_states++;
#endif
  case 0x00:
#ifdef I2C_STATS
    if(trans->type == I2CTransRx){
        stats.nb_SR_err_bus++;
    }
    else{
        stats.nb_MT_err_bus++;
    }
    if(!stats.t_first_err) stats.t_first_err = micros();
    stats.t_last_err = micros();
#endif
    trans->status = I2CTransFailed;
    I2C0_CONSET = BIT(4 /*STO*/) | BIT(2 /*AA*/); // release bus (set STO and AA bits)
    break;
  }

#ifdef I2C_DEBUG
  if(i_debug_tab < sizeof(debug_tab)/sizeof(*debug_tab)) {
    debug_tab[i_debug_tab].end_set = I2C0_CONSET;
    i_debug_tab++;
  }
#endif

  I2C0_CONCLR = BIT(3 /*SIC*/); // clear SI bit in I2C0_CONSET | acknowledges interrupt
  VIC_VectAddr = (unsigned)0; // updates priority hardware
}

void i2c0_init(unsigned int speed, uint8_t sla_addr, i2c_handler h, void *userp) {
  // SCL0 on P0.2
  PCB_PINSEL0 &= ~(3<<4);
  PCB_PINSEL0 |= 1<<4;

  // SDA0 on P0.3
  PCB_PINSEL0 &= ~(3<<6);
  PCB_PINSEL0 |= 1<<6;

  // freq_i2c = PCLK/(I2C0_SCLL + I2C0_SCLH)
  I2C0_SCLL = (PCLK/speed)>>1;
  I2C0_SCLH = (PCLK/speed)>>1;

  // clear all flags...
  I2C0_CONCLR = BIT(2) | BIT(3) | BIT(5) | BIT(6);
  // ...and set only the desired ones
  I2C0_CONSET = BIT(6 /*I2EN*/) | (sla_addr?BIT(2 /*AA*/):0);

  // set hw/slave addr
  I2C0_ADR = sla_addr; // when LSB is set, accepts general call address 0x00

  // bind I2C0 (irq9) to interrupt priority 11
  VIC_VectCntl11 = 0x20 | 9;
  VIC_VectAddr11 = (unsigned int)_i2c0_isr;

  // enable I2C0 interrupt
  VIC_IntEnable = BIT(9);


  // state machine init
  p.trans_insert_idx = 0;
  p.trans_extract_idx = 0;
  p.status = I2CDevIdle;
  p.own_addr = sla_addr & ~BIT(0 /*GC*/);
  p.handler = h;  // slave transaction handler
  p.userp = userp; // user data
  p.trans_sla.len_r = 0;
  p.trans_sla.len_w = 0;
}

int i2c0_submit(struct i2c_transaction* t) {
  uint8_t idx;
  int irq_status;

  idx = (p.trans_insert_idx + 1)%I2C_TRANSACTION_QUEUE_LEN;
  if (idx == p.trans_extract_idx) {
    t->status = I2CTransFailed;
#ifdef I2C_STATS
    stats.nb_MT_err_buffer_full++;
    if(!stats.t_first_err) stats.t_first_err = micros();
    stats.t_last_err = micros();
#endif
    return 1;  /* queue full */
  }
  t->status = I2CTransPending;
  t->nb_retry = 0;

  irq_status = global_IRQ_disable();
  p.trans[p.trans_insert_idx] = t;
  p.trans_insert_idx = idx;
  /* if peripheral is idle, start the transaction */
  if (p.status == I2CDevIdle) {
    p.status = I2CDevStartRequested;
    I2C0_CONSET = BIT(5 /*STA*/);  // sets START bit
  }
  /* else it will be started by the interrupt handler */
  /* when the previous transactions completes (see I2cEndOfTransaction) */
  global_IRQ_restore(irq_status);

  return 0;
}
