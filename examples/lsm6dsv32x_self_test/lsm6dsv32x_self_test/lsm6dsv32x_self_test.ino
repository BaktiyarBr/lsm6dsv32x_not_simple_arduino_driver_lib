/*
* @brief   This file implements the self test procedure.
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

/* Self test limits. */
#define    MIN_ST_LIMIT_mg        50.0f
#define    MAX_ST_LIMIT_mg      1700.0f
#define    MIN_ST_LIMIT_mdps   150000.0f
#define    MAX_ST_LIMIT_mdps   700000.0f

/* Self test results. */
#define    ST_PASS     1U
#define    ST_FAIL     0U


LSM6DSV32X_Handle_t imu_handle;


lsm6dsv32x_data_ready_t drdy;
int16_t data_raw[3];
stmdev_ctx_t dev_ctx;
float_t val_st_off[3];
float_t val_st_on[3];
float_t test_val[3];
uint8_t st_result;
uint8_t whoamI;
lsm6dsv32x_reset_t rst;
uint8_t i;
uint8_t j;

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
  /*
   * Accelerometer Self Test
   */
  /* Set Output Data Rate */
  lsm6dsv32x_xl_data_rate_set(&dev_ctx, LSM6DSV32X_ODR_AT_60Hz);
  /* Set full scale */
  lsm6dsv32x_xl_full_scale_set(&dev_ctx, LSM6DSV32X_4g);
  /* Wait stable output */
  platform_delay(100);

  /* Check if new value available */
  do {
    lsm6dsv32x_flag_data_ready_get(&dev_ctx, &drdy);
  } while (!drdy.drdy_xl);

  /* Read dummy data and discard it */
  lsm6dsv32x_acceleration_raw_get(&dev_ctx, data_raw);
  /* Read 5 sample and get the average vale for each axis */
  memset(val_st_off, 0x00, 3 * sizeof(float));

  for (i = 0; i < 5; i++) {
    /* Check if new value available */
    do {
      lsm6dsv32x_flag_data_ready_get(&dev_ctx, &drdy);
    } while (!drdy.drdy_xl);

    /* Read data and accumulate the mg value */
    lsm6dsv32x_acceleration_raw_get(&dev_ctx, data_raw);

    for (j = 0; j < 3; j++) {
      val_st_off[j] += lsm6dsv32x_from_fs4_to_mg(data_raw[j]);
    }
  }

  /* Calculate the mg average values */
  for (i = 0; i < 3; i++) {
    val_st_off[i] /= 5.0f;
  }

  /* Enable Self Test positive (or negative) */
  lsm6dsv32x_xl_self_test_set(&dev_ctx, LSM6DSV32X_XL_ST_NEGATIVE);
  //lsm6dsv32x_xl_self_test_set(&dev_ctx, LSM6DSV32X_XL_ST_POSITIVE);
  /* Wait stable output */
  platform_delay(100);

  /* Check if new value available */
  do {
    lsm6dsv32x_flag_data_ready_get(&dev_ctx, &drdy);
  } while (!drdy.drdy_xl);

  /* Read dummy data and discard it */
  lsm6dsv32x_acceleration_raw_get(&dev_ctx, data_raw);
  /* Read 5 sample and get the average vale for each axis */
  memset(val_st_on, 0x00, 3 * sizeof(float));

  for (i = 0; i < 5; i++) {
    /* Check if new value available */
    do {
      lsm6dsv32x_flag_data_ready_get(&dev_ctx, &drdy);
    } while (!drdy.drdy_xl);

    /* Read data and accumulate the mg value */
    lsm6dsv32x_acceleration_raw_get(&dev_ctx, data_raw);

    for (j = 0; j < 3; j++) {
      val_st_on[j] += lsm6dsv32x_from_fs4_to_mg(data_raw[j]);
    }
  }

  /* Calculate the mg average values */
  for (i = 0; i < 3; i++) {
    val_st_on[i] /= 5.0f;
  }

  /* Calculate the mg values for self test */
  for (i = 0; i < 3; i++) {
    test_val[i] = fabsf((val_st_on[i] - val_st_off[i]));
  }

  /* Check self test limit */
  st_result = ST_PASS;

  for (i = 0; i < 3; i++) {
    if (( MIN_ST_LIMIT_mg > test_val[i] ) ||
        ( test_val[i] > MAX_ST_LIMIT_mg)) {
      st_result = ST_FAIL;
    }
  }

  /* Disable Self Test */
  lsm6dsv32x_xl_self_test_set(&dev_ctx, LSM6DSV32X_XL_ST_DISABLE);
  /* Disable sensor. */
  lsm6dsv32x_xl_data_rate_set(&dev_ctx, LSM6DSV32X_ODR_OFF);
  /*
   * Gyroscope Self Test
   */
  /* Set Output Data Rate */
  lsm6dsv32x_gy_data_rate_set(&dev_ctx, LSM6DSV32X_ODR_AT_240Hz);
  /* Set full scale */
  lsm6dsv32x_gy_full_scale_set(&dev_ctx, LSM6DSV32X_2000dps);
  /* Wait stable output */
  platform_delay(100);

  /* Check if new value available */
  do {
    lsm6dsv32x_flag_data_ready_get(&dev_ctx, &drdy);
  } while (!drdy.drdy_gy);

  /* Read dummy data and discard it */
  lsm6dsv32x_angular_rate_raw_get(&dev_ctx, data_raw);
  /* Read 5 sample and get the average vale for each axis */
  memset(val_st_off, 0x00, 3 * sizeof(float));

  for (i = 0; i < 5; i++) {
    /* Check if new value available */
    do {
      lsm6dsv32x_flag_data_ready_get(&dev_ctx, &drdy);
    } while (!drdy.drdy_gy);
    /* Read data and accumulate the mg value */
    lsm6dsv32x_angular_rate_raw_get(&dev_ctx, data_raw);

    for (j = 0; j < 3; j++) {
      val_st_off[j] += lsm6dsv32x_from_fs2000_to_mdps(data_raw[j]);
    }
  }

  /* Calculate the mg average values */
  for (i = 0; i < 3; i++) {
    val_st_off[i] /= 5.0f;
  }

  /* Enable Self Test positive (or negative) */
  lsm6dsv32x_gy_self_test_set(&dev_ctx, LSM6DSV32X_GY_ST_POSITIVE);
  //lsm6dsv32x_gy_self_test_set(&dev_ctx, LIS2DH12_GY_ST_NEGATIVE);
  /* Wait stable output */
  platform_delay(100);
  /* Read 5 sample and get the average vale for each axis */
  memset(val_st_on, 0x00, 3 * sizeof(float));

  for (i = 0; i < 5; i++) {
    /* Check if new value available */
    do {
      lsm6dsv32x_flag_data_ready_get(&dev_ctx, &drdy);
    } while (!drdy.drdy_gy);

    /* Read data and accumulate the mg value */
    lsm6dsv32x_angular_rate_raw_get(&dev_ctx, data_raw);

    for (j = 0; j < 3; j++) {
      val_st_on[j] += lsm6dsv32x_from_fs2000_to_mdps(data_raw[j]);
    }
  }

  /* Calculate the mg average values */
  for (i = 0; i < 3; i++) {
    val_st_on[i] /= 5.0f;
  }

  /* Calculate the mg values for self test */
  for (i = 0; i < 3; i++) {
    test_val[i] = fabsf((val_st_on[i] - val_st_off[i]));
  }

  /* Check self test limit */
  for (i = 0; i < 3; i++) {
    if (( MIN_ST_LIMIT_mdps > test_val[i] ) ||
        ( test_val[i] > MAX_ST_LIMIT_mdps)) {
      st_result = ST_FAIL;
    }
  }

  /* Disable Self Test */
  lsm6dsv32x_gy_self_test_set(&dev_ctx, LSM6DSV32X_GY_ST_DISABLE);
  /* Disable sensor. */
  lsm6dsv32x_xl_data_rate_set(&dev_ctx, LSM6DSV32X_ODR_OFF);

  if (st_result == ST_PASS) {
    Serial.println("Self Test - PASS");
  }

  else {
    Serial.println("Self Test - FAILL");
  }

}

void loop() {
}
