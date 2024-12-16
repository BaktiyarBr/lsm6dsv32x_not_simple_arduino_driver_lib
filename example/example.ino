#include "lsm6dsv32x_reg.h"
#include "lsm6dsv32x.h"

// Define I2C and SPI configuration
#define LSM6DSV32X_I2C_ADDR  0x6B
#define FIFO_WATERMARK    32
lsm6dsv32x_fifo_sflp_raw_t fifo_sflp;

// Communication interface objects
stmdev_ctx_t dev_ctx;
LSM6DSV32X_Handle_t imu_handle;


  lsm6dsv32x_data_ready_t drdy;
  int16_t data_raw[3];
  float val_st_off[3];
  float val_st_on[3];
  float test_val[3];
  uint8_t st_result;
  lsm6dsv32x_reset_t rst;
  uint8_t i;
  uint8_t j;

lsm6dsv32x_filt_settling_mask_t filt_settling_mask;

int16_t data_raw_acceleration[3];
int16_t data_raw_angular_rate[3];
int16_t data_raw_temperature;
float acceleration_mg[3];
float angular_rate_mdps[3];
float temperature_degC;
uint8_t whoamI;
uint8_t tx_buffer[1000];
lsm6dsv32x_sflp_gbias_t gbias;
lsm6dsv32x_fifo_status_t fifo_status;


void setup() {
  // Initialize I2C
  Serial.begin(115200);
  Wire.begin();

  // Configure the handle
  imu_handle.dev_spi = nullptr;
  imu_handle.dev_i2c = &Wire;
  imu_handle.address = LSM6DSV32X_I2C_ADDR; // I2C address of LSM6DSV32X
    /* Initialize mems driver interface */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.mdelay = platform_delay;
  dev_ctx.handle = &imu_handle;

  uint8_t whoamI;
   /* Check device ID */
  lsm6dsv32x_device_id_get(&dev_ctx, &whoamI);

  if (whoamI != LSM6DSV32X_ID){
    while (1);
  }
    Serial.println("IMU Initialized Successfully");
    Serial.println(whoamI);

  /* Restore default configuration */
  lsm6dsv32x_reset_set(&dev_ctx, LSM6DSV32X_RESTORE_CTRL_REGS);
  do {
    lsm6dsv32x_reset_get(&dev_ctx, &rst);
    Serial.println(rst);
  } while (rst == LSM6DSV32X_READY);

  Serial.println("IMU LSM6DSV32X_RESTORE_CTRL_REGS");

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

  /* Set FIFO batch of sflp data */
  fifo_sflp.game_rotation = 1;
  fifo_sflp.gravity = 1;
  fifo_sflp.gbias = 1;
  lsm6dsv32x_fifo_sflp_batch_set(&dev_ctx, fifo_sflp);

  /* Set FIFO mode to Stream mode (aka Continuous Mode) */
  lsm6dsv32x_fifo_mode_set(&dev_ctx, LSM6DSV32X_STREAM_MODE);

  /* Set Output Data Rate */
  lsm6dsv32x_xl_data_rate_set(&dev_ctx, LSM6DSV32X_ODR_AT_7680Hz);
  lsm6dsv32x_gy_data_rate_set(&dev_ctx, LSM6DSV32X_ODR_AT_7680Hz);
  lsm6dsv32x_sflp_data_rate_set(&dev_ctx, LSM6DSV32X_SFLP_480Hz);

  lsm6dsv32x_sflp_game_rotation_set(&dev_ctx, PROPERTY_ENABLE);

  /*
   * here application may initialize offset with latest values
   * calculated from previous run and saved to non volatile memory.
   */
  gbias.gbias_x = 0.0f;
  gbias.gbias_y = 0.0f;
  gbias.gbias_z = 0.0f;
  lsm6dsv32x_sflp_game_gbias_set(&dev_ctx, &gbias);


      
}

void loop() {

  uint16_t num = 0;

  /* Read watermark flag */
  lsm6dsv32x_fifo_status_get(&dev_ctx, &fifo_status);

  if (fifo_status.fifo_th == 1) {
    num = fifo_status.fifo_level;

    Serial.print("-- FIFO num ");
    Serial.println(num);

    while (num--) {
      lsm6dsv32x_fifo_out_raw_t f_data;
      int16_t *axis;
      float quat[4];
      float gravity_mg[3];
      float gbias_mdps[3];

      /* Read FIFO sensor value */
      lsm6dsv32x_fifo_out_raw_get(&dev_ctx, &f_data);

      switch (f_data.tag) {
      case 0x16:
        axis = (int16_t *)&f_data.data[0];
        gbias_mdps[0] = lsm6dsv32x_from_fs125_to_mdps(axis[0]);
        gbias_mdps[1] = lsm6dsv32x_from_fs125_to_mdps(axis[1]);
        gbias_mdps[2] = lsm6dsv32x_from_fs125_to_mdps(axis[2]);
        // Serial.print("GBIAS [mdps]: ");
        // Serial.print(gbias_mdps[0], 2);
        // Serial.print("\t");
        // Serial.print(gbias_mdps[1], 2);
        // Serial.print("\t");
        // Serial.println(gbias_mdps[2], 2);
        break;
      case  0x17:
        axis = (int16_t *)&f_data.data[0];
        gravity_mg[0] = lsm6dsv32x_from_sflp_to_mg(axis[0]);
        gravity_mg[1] = lsm6dsv32x_from_sflp_to_mg(axis[1]);
        gravity_mg[2] = lsm6dsv32x_from_sflp_to_mg(axis[2]);
        // Serial.print("Gravity [mg]: ");
        // Serial.print(gravity_mg[0], 2);
        // Serial.print("\t");
        // Serial.print(gravity_mg[1], 2);
        // Serial.print("\t");
        // Serial.println(gravity_mg[2], 2);
        break;
      case 0x13:
        sflp2q(quat, (uint16_t *)&f_data.data[0]);
        Serial.print("Game Rotation \tX: ");
        Serial.print(quat[0], 3);
        Serial.print("\tY: ");
        Serial.print(quat[1], 3);
        Serial.print("\tZ: ");
        Serial.print(quat[2], 3);
        Serial.print("\tW: ");
        Serial.println(quat[3], 3);
        break;
      default:
        break;
      }
    }
    Serial.println("------ \r\n");
  }
}

