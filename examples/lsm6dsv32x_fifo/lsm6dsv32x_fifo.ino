/*
 * @brief   This file how to configure compressed FIFO and to retrieve acc
 *          and gyro data. This sample use a fifo utility library tool
 *          for FIFO decompression.

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
static uint8_t tx_buffer[1000];

/* Private variables ---------------------------------------------------------*/
static int16_t *datax;
static int16_t *datay;
static int16_t *dataz;
static int32_t *ts;

LSM6DSV32X_Handle_t imu_handle;
lsm6dsv32x_fifo_status_t fifo_status;
stmdev_ctx_t dev_ctx;
lsm6dsv32x_reset_t rst;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Configure the handle
  imu_handle.dev_spi = nullptr;
  imu_handle.dev_i2c = &Wire;
  imu_handle.address = LSM6DSV32X_I2C_ADDR; // I2C address of LSM6DSV32X

  /* Uncomment to configure INT 1 */
  //lsm6dsv32x_pin_int1_route_t int1_route;
  /* Uncomment to configure INT 2 */
  //lsm6dsv32x_pin_int2_route_t int2_route;
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
  /* Set full scale */
  lsm6dsv32x_xl_full_scale_set(&dev_ctx, LSM6DSV32X_8g);
  lsm6dsv32x_gy_full_scale_set(&dev_ctx, LSM6DSV32X_2000dps);
  /*
   * Set FIFO watermark (number of unread sensor data TAG + 6 bytes
   * stored in FIFO) to FIFO_WATERMARK samples
   */
  lsm6dsv32x_fifo_watermark_set(&dev_ctx, FIFO_WATERMARK);
  /* Set FIFO batch XL/Gyro ODR to 12.5Hz */
  lsm6dsv32x_fifo_xl_batch_set(&dev_ctx, LSM6DSV32X_XL_BATCHED_AT_60Hz);
  lsm6dsv32x_fifo_gy_batch_set(&dev_ctx, LSM6DSV32X_GY_BATCHED_AT_15Hz);
  /* Set FIFO mode to Stream mode (aka Continuous Mode) */
  lsm6dsv32x_fifo_mode_set(&dev_ctx, LSM6DSV32X_STREAM_MODE);

  /* Set Output Data Rate */
  lsm6dsv32x_xl_data_rate_set(&dev_ctx, LSM6DSV32X_ODR_AT_60Hz);
  lsm6dsv32x_gy_data_rate_set(&dev_ctx, LSM6DSV32X_ODR_AT_15Hz);
  lsm6dsv32x_fifo_timestamp_batch_set(&dev_ctx, LSM6DSV32X_TMSTMP_DEC_8);
  lsm6dsv32x_timestamp_set(&dev_ctx, PROPERTY_ENABLE);

}

void loop() {
  uint16_t num = 0;
  lsm6dsv32x_fifo_status_t fifo_status;

  /* Read watermark flag */
  lsm6dsv32x_fifo_status_get(&dev_ctx, &fifo_status);

  if (fifo_status.fifo_th == 1) {
    num = fifo_status.fifo_level;

    Serial.print("-- FIFO num: ");
    Serial.println(num);

    while (num--) {
      lsm6dsv32x_fifo_out_raw_t f_data;
      int16_t *datax, *datay, *dataz;
      int32_t *ts;

      /* Read FIFO sensor value */
      lsm6dsv32x_fifo_out_raw_get(&dev_ctx, &f_data);
      datax = (int16_t *)&f_data.data[0];
      datay = (int16_t *)&f_data.data[2];
      dataz = (int16_t *)&f_data.data[4];
      ts = (int32_t *)&f_data.data[0];

      switch (f_data.tag) {
      case 0x2: //LSM6DSV32X_XL_NC_TAG
        Serial.print("ACC [mg]: ");
        Serial.print(lsm6dsv32x_from_fs8_to_mg(*datax), 2);
        Serial.print("\t");
        Serial.print(lsm6dsv32x_from_fs8_to_mg(*datay), 2);
        Serial.print("\t");
        Serial.println(lsm6dsv32x_from_fs8_to_mg(*dataz), 2);
        break;

      case 0x1: //LSM6DSV32X_GY_NC_TAG
        Serial.print("GYR [mdps]: ");
        Serial.print(lsm6dsv32x_from_fs2000_to_mdps(*datax), 2);
        Serial.print("\t");
        Serial.print(lsm6dsv32x_from_fs2000_to_mdps(*datay), 2);
        Serial.print("\t");
        Serial.println(lsm6dsv32x_from_fs2000_to_mdps(*dataz), 2);
        break;

      case 0x4: //LSM6DSV32X_TIMESTAMP_TAG
        Serial.print("TIMESTAMP [ms]: ");
        Serial.println(*ts);
        break;

      default:
        break;
      }
    }

    Serial.println("------");
    Serial.println();
  }
  delay(500);
}
