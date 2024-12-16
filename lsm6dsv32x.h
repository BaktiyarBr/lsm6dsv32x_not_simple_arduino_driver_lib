#ifndef LSM6DSV32X_H
#define LSM6DSV32X_H

#include <stdint.h>
#include <stdbool.h>
#include "lsm6dsv32x_reg.h"
#include "Wire.h"
#include "SPI.h"




// Function prototypes
#ifdef __cplusplus
extern "C" {
#endif

// Global Variables
extern TwoWire *_i2c;
extern uint8_t _address;
extern uint8_t _csPin;
extern bool _useSPI;
extern bool _useSPIHS;
extern SPIClass *_spi;

const uint32_t SPI_LS_CLOCK = 1000000;  // 1 MHz
const uint32_t SPI_HS_CLOCK = 15000000; // 15 MHz


static uint32_t npy_halfbits_to_floatbits(uint16_t h);
static float_t npy_half_to_float(uint16_t h);
static void sflp2q(float quat[4], uint16_t sflp[3]);

/**
 * @brief Initialize the LSM6DSV32X device over I2C.
 * 
 * @param dev_ctx Pointer to the device context.
 * @param bus I2C bus handle.
 * @param address I2C address of the device.
 * @return 0 on success, negative value on failure.
 */
int lsm6dsv32x_init_I2C(stmdev_ctx_t *dev_ctx, TwoWire &bus, uint8_t address);

/**
 * @brief Initialize the LSM6DSV32X device over SPI.
 * 
 * @param dev_ctx Pointer to the device context.
 * @param bus SPI bus handle.
 * @param cs_pin Chip select pin.
 * @return 0 on success, negative value on failure.
 */
int lsm6dsv32x_init_SPI(stmdev_ctx_t *dev_ctx, SPIClass &bus, uint8_t cs_pin);

/**
 * @brief Perform a full initialization of the LSM6DSV32X sensor.
 * 
 * @param dev_ctx Device context structure.
 * @param boot_time Boot time in milliseconds.
 * @return 0 on success, negative value on failure.
 */
int lsm6dsv32x_init(stmdev_ctx_t dev_ctx,  lsm6dsv32x_reset_t rst, uint32_t boot_time);

/**
 * @brief Platform-specific write function.
 * 
 * @param handle Pointer to the platform context.
 * @param reg Register address.
 * @param bufp Pointer to the data buffer.
 * @param len Length of the data buffer.
 * @return 0 on success, negative value on failure.
 */
int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);

/**
 * @brief Platform-specific read function.
 * 
 * @param handle Pointer to the platform context.
 * @param reg Register address.
 * @param bufp Pointer to the buffer to store data.
 * @param len Length of the buffer.
 * @return 0 on success, negative value on failure.
 */
int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);

/**
 * @brief Delay function for the platform.
 * 
 * @param ms Time to delay in milliseconds.
 */
void platform_delay(uint32_t ms);

/**
 * @brief Initialize platform-specific settings.
 */
void platform_init(void);

#ifdef __cplusplus
}
#endif

#endif // LSM6DSV32X_H
