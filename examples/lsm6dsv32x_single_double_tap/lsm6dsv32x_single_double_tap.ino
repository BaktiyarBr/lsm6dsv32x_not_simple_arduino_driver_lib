/*
 * @brief   This file shows how to generate and handle a Free Fall event.
 */


/* Private macro -------------------------------------------------------------*/
/*
 * Select FIFO samples watermark, max value is 512
 * in FIFO are stored acc, gyro and timestamp samples
 */

#include <lsm6dsv32x.h>
#include <lsm6dsv32x_reg.h>
#define LSM6DSV32X_I2C_ADDR  0x6B
#define BOOT_TIME         10
#define FIFO_WATERMARK    64

/* Private variables ---------------------------------------------------------*/
static uint8_t whoamI;

static lsm6dsv32x_interrupt_mode_t irq;
static lsm6dsv32x_tap_detection_t tap;
static lsm6dsv32x_tap_thresholds_t tap_ths;
static lsm6dsv32x_tap_time_windows_t tap_win;

LSM6DSV32X_Handle_t imu_handle;
lsm6dsv32x_fifo_status_t fifo_status;
stmdev_ctx_t dev_ctx;
lsm6dsv32x_reset_t rst;

static uint8_t stap_event_catched = 0;
static uint8_t dtap_event_catched = 0;

void lsm6dsv32x_single_double_tap_handler(void)
{
  lsm6dsv32x_all_sources_t status;

  /* Read output only if new xl value is available */
  lsm6dsv32x_all_sources_get(&dev_ctx, &status);

  if (status.single_tap) {
    stap_event_catched = 1;
  }
  if (status.double_tap) {
    dtap_event_catched = 1;
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

  pin_int.double_tap = PROPERTY_ENABLE;
  pin_int.single_tap = PROPERTY_ENABLE;
  lsm6dsv32x_pin_int1_route_set(&dev_ctx, &pin_int);
  //lsm6dsv32x_pin_int2_route_set(&dev_ctx, &pin_int);

  irq.enable = 1;
  irq.lir = 0;
  lsm6dsv32x_interrupt_enable_set(&dev_ctx, irq);

  tap.tap_z_en = 1;
  lsm6dsv32x_tap_detection_set(&dev_ctx, tap);

  tap_ths.z = 3;
  lsm6dsv32x_tap_thresholds_set(&dev_ctx, tap_ths);

  tap_win.tap_gap = 7;
  tap_win.shock = 3;
  tap_win.quiet = 3;
  lsm6dsv32x_tap_time_windows_set(&dev_ctx, tap_win);

  lsm6dsv32x_tap_mode_set(&dev_ctx, LSM6DSV32X_BOTH_SINGLE_DOUBLE);

  /* Set Output Data Rate.*/
  lsm6dsv32x_xl_data_rate_set(&dev_ctx, LSM6DSV32X_ODR_AT_480Hz);
  /* Set full scale */
  lsm6dsv32x_xl_full_scale_set(&dev_ctx, LSM6DSV32X_8g);

        Serial.println( " Whileeeee");

}

void loop() {
 if (stap_event_catched) {
        stap_event_catched = 0;
        Serial.println( " Single TAP ");
  }
  if (dtap_event_catched) {
      dtap_event_catched = 0;
      Serial.println( " Double TAP ");
  }
}
