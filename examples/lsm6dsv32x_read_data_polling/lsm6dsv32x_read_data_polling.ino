#include <lsm6dsv32x.h>
#include <lsm6dsv32x_reg.h>
#define LSM6DSV32X_I2C_ADDR  0x6B
#define BOOT_TIME         10
#define FIFO_WATERMARK    64


static int16_t data_raw_acceleration[3];
static int16_t data_raw_angular_rate[3];
static int16_t data_raw_temperature;
static float_t acceleration_mg[3];
static float_t angular_rate_mdps[3];
static float_t temperature_degC;
static uint8_t whoamI;

lsm6dsv32x_filt_settling_mask_t filt_settling_mask;
stmdev_ctx_t dev_ctx;
lsm6dsv32x_data_ready_t drdy;
lsm6dsv32x_reset_t rst;

LSM6DSV32X_Handle_t imu_handle;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Configure the handle
  imu_handle.dev_spi = nullptr;
  imu_handle.dev_i2c = &Wire;
  imu_handle.address = LSM6DSV32X_I2C_ADDR; // I2C address of LSM6DSV32X

  /* Initialize mems driver interface */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
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
  /* Set Output Data Rate.
   * Selected data rate have to be equal or greater with respect
   * with MLC data rate.
   */
  lsm6dsv32x_xl_data_rate_set(&dev_ctx, LSM6DSV32X_ODR_AT_7Hz5);
  lsm6dsv32x_gy_data_rate_set(&dev_ctx, LSM6DSV32X_ODR_AT_15Hz);
  /* Set full scale */
  lsm6dsv32x_xl_full_scale_set(&dev_ctx, LSM6DSV32X_4g);
  lsm6dsv32x_gy_full_scale_set(&dev_ctx, LSM6DSV32X_2000dps);
  /* Configure filtering chain */
  filt_settling_mask.drdy = PROPERTY_ENABLE;
  filt_settling_mask.irq_xl = PROPERTY_ENABLE;
  filt_settling_mask.irq_g = PROPERTY_ENABLE;
  lsm6dsv32x_filt_settling_mask_set(&dev_ctx, filt_settling_mask);
  lsm6dsv32x_filt_gy_lp1_set(&dev_ctx, PROPERTY_ENABLE);
  lsm6dsv32x_filt_gy_lp1_bandwidth_set(&dev_ctx, LSM6DSV32X_GY_ULTRA_LIGHT);
  lsm6dsv32x_filt_xl_lp2_set(&dev_ctx, PROPERTY_ENABLE);
  lsm6dsv32x_filt_xl_lp2_bandwidth_set(&dev_ctx, LSM6DSV32X_XL_STRONG);
}

void loop() {
  /* Check if new accelerometer data is available */
  lsm6dsv32x_flag_data_ready_get(&dev_ctx, &drdy);

  if (drdy.drdy_xl) {
    /* Read acceleration field data */
    int16_t data_raw_acceleration[3] = {0};
    float acceleration_mg[3] = {0.0f};

    lsm6dsv32x_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
    acceleration_mg[0] = lsm6dsv32x_from_fs4_to_mg(data_raw_acceleration[0]);
    acceleration_mg[1] = lsm6dsv32x_from_fs4_to_mg(data_raw_acceleration[1]);
    acceleration_mg[2] = lsm6dsv32x_from_fs4_to_mg(data_raw_acceleration[2]);

    Serial.print("Acceleration [mg]: ");
    Serial.print(acceleration_mg[0], 2);
    Serial.print("\t");
    Serial.print(acceleration_mg[1], 2);
    Serial.print("\t");
    Serial.println(acceleration_mg[2], 2);
  }

  if (drdy.drdy_gy) {
    /* Read angular rate field data */
    int16_t data_raw_angular_rate[3] = {0};
    float angular_rate_mdps[3] = {0.0f};

    lsm6dsv32x_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate);
    angular_rate_mdps[0] = lsm6dsv32x_from_fs2000_to_mdps(data_raw_angular_rate[0]);
    angular_rate_mdps[1] = lsm6dsv32x_from_fs2000_to_mdps(data_raw_angular_rate[1]);
    angular_rate_mdps[2] = lsm6dsv32x_from_fs2000_to_mdps(data_raw_angular_rate[2]);

    Serial.print("Angular rate [mdps]: ");
    Serial.print(angular_rate_mdps[0], 2);
    Serial.print("\t");
    Serial.print(angular_rate_mdps[1], 2);
    Serial.print("\t");
    Serial.println(angular_rate_mdps[2], 2);
  }

  if (drdy.drdy_temp) {
    /* Read temperature data */
    int16_t data_raw_temperature = 0;
    float temperature_degC = 0.0f;

    lsm6dsv32x_temperature_raw_get(&dev_ctx, &data_raw_temperature);
    temperature_degC = lsm6dsv32x_from_lsb_to_celsius(data_raw_temperature);

    Serial.print("Temperature [degC]: ");
    Serial.println(temperature_degC, 2);
  }

  delay(100); // Adjust as necessary
}
