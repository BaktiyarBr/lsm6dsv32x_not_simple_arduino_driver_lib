#include "lsm6dsv32x_reg.h"
#include "lsm6dsv32x.h"

#define LSM6DSV32X_I2C_ADDR  0x6B
#define    BOOT_TIME            10 //ms
static uint8_t whoamI;
static lsm6dsv32x_interrupt_mode_t irq;

LSM6DSV32X_Handle_t imu_handle;

static stmdev_ctx_t dev_ctx;
static uint8_t sixd_event_catched = 0;

void lsm6dsv32x_sixd_handler(void)
{
  lsm6dsv32x_all_sources_t status;

  /* Read output only if new xl value is available */
  lsm6dsv32x_all_sources_get(&dev_ctx, &status);

  if (status.six_d) {
    sixd_event_catched = (status.six_d    << 6) |
                         (status.six_d_zh << 5) |
                         (status.six_d_zl << 4) |
                         (status.six_d_yh << 3) |
                         (status.six_d_yl << 2) |
                         (status.six_d_xh << 1) |
                         (status.six_d_xl << 0);
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Configure the handle
  imu_handle.dev_spi = nullptr;
  imu_handle.dev_i2c = &Wire;
  imu_handle.address = LSM6DSV32X_I2C_ADDR; // I2C address of LSM6DSV32X

  lsm6dsv32x_pin_int_route_t pin_int;
  lsm6dsv32x_reset_t rst;

  /* Initialize mems driver interface */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.mdelay = platform_delay;
  dev_ctx.handle = &imu_handle;

 /* Wait sensor boot time */
  platform_delay(BOOT_TIME);
  /* Check device ID */
  lsm6dsv32x_device_id_get(&dev_ctx, &whoamI);

  if (whoamI != LSM6DSV32X_ID)
    while (1);

  /* Restore default configuration */
  lsm6dsv32x_reset_set(&dev_ctx, LSM6DSV32X_RESTORE_CTRL_REGS);
  do {
    lsm6dsv32x_reset_get(&dev_ctx, &rst);
  } while (rst != LSM6DSV32X_READY);

  /* Enable Block Data Update */
  lsm6dsv32x_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

  pin_int.sixd = PROPERTY_ENABLE;
  lsm6dsv32x_pin_int1_route_set(&dev_ctx, &pin_int);
  //lsm6dsv32x_pin_int2_route_set(&dev_ctx, &pin_int);

  irq.enable = 1;
  irq.lir = 1;
  lsm6dsv32x_interrupt_enable_set(&dev_ctx, irq);

  lsm6dsv32x_filt_sixd_feed_set(&dev_ctx, LSM6DSV32X_SIXD_FEED_LOW_PASS);
  lsm6dsv32x_6d_threshold_set(&dev_ctx, LSM6DSV32X_DEG_60);

  /* Set Output Data Rate.*/
  lsm6dsv32x_xl_data_rate_set(&dev_ctx, LSM6DSV32X_ODR_AT_120Hz);
  /* Set full scale */
  lsm6dsv32x_xl_full_scale_set(&dev_ctx, LSM6DSV32X_32g);

}

void loop() {
  if (sixd_event_catched & 0x40) {

    switch (sixd_event_catched) {
      case 0x48:
        Serial.println("6D Position A");
        break;

      case 0x41:
        Serial.println("6D Position B");
        break;

      case 0x42:
        Serial.println("6D Position C");
        break;

      case 0x44:
        Serial.println("6D Position D");
        break;

      case 0x60:
        Serial.println("6D Position E");
        break;

      case 0x50:
        Serial.println("6D Position F");
        break;

      default:
        Serial.println("6D Position Unknown");
        break;
    }

    sixd_event_catched = 0;
  }
}
