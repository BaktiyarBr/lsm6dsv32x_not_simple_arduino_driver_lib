#ifndef lsm6dsv32x_h
#define lsm6dsv32x_h

#include <Arduino.h>
#include "lsm6dsv32x_reg.h"
#include "lsm6dsv32x.h"
#include "Wire.h"


TwoWire *_i2c;
uint8_t _address;
uint8_t _csPin;
bool _useSPI;
bool _useSPIHS;
SPIClass *_spi;


int lsm6dsv32x_init_I2C(stmdev_ctx_t *dev_ctx, TwoWire &bus, uint8_t address){
    // Initialize the device context
    dev_ctx->write_reg = platform_write;
    dev_ctx->read_reg = platform_read;
    dev_ctx->mdelay = platform_delay;
    dev_ctx->handle = (void *)&bus; // Pass `this` for accessing instance variables
    
    _i2c = &bus; // I2C bus
    _address = address; // I2C address
    _useSPI = false; // set to use I2C
    _i2c->begin();
    return 0;

}

int lsm6dsv32x_init_SPI(stmdev_ctx_t *dev_ctx, SPIClass &bus,uint8_t csPin){
    // Initialize the device context
    dev_ctx->write_reg = platform_write;
    dev_ctx->read_reg = platform_read;
    dev_ctx->mdelay = platform_delay;
    dev_ctx->handle = (void *)&bus; // Pass `this` for accessing instance variables

    _spi = &bus; // SPI bus
    _csPin = csPin; // chip select pin
    _useSPI = true; // set to use SPI
    pinMode(_csPin, OUTPUT);
    digitalWrite(_csPin, HIGH); // Deselect the device initially
    _spi ->begin();

    return 0;
}

int lsm6dsv32x_init(stmdev_ctx_t dev_ctx,  lsm6dsv32x_reset_t rst, uint32_t boot_time) {
    uint8_t whoamI = 0;

    // Delay for boot time
    platform_delay(boot_time);

    // Check device ID
    if (lsm6dsv32x_device_id_get(&dev_ctx, &whoamI) != 0 || whoamI != LSM6DSV32X_ID) {
        return -1; // Device ID mismatch
    }

    // Reset device to default settings
    if (lsm6dsv32x_reset_set(&dev_ctx, LSM6DSV32X_RESTORE_CTRL_REGS) != 0) {
        return -1; // Failed to reset
    }

    // Wait for reset completion
    do {
        if (lsm6dsv32x_reset_get(&dev_ctx, &rst) != 0) {
            return -1; // Failed to get reset status
        }
    } while (rst != LSM6DSV32X_READY);

    // Enable block data update
    lsm6dsv32x_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);


    return 0; // Success
}

// void *hanlde is not used instead use init function
int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
    if(_useSPI){
    
        if(_useSPIHS){
        _spi->beginTransaction(SPISettings(SPI_HS_CLOCK, MSBFIRST, SPI_MODE3));
        }
        else{
        _spi->beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3));
        }
        
        //spi receive data begin the transaction
        
        reg |= 0x80; //! ???? eto sho za hren 
        
        digitalWrite(_csPin, LOW); // Pull CS low to select the device
        _spi->transfer(reg); // Transmit the register address
        
        // Receive the data
        for (uint8_t i = 0; i < len; i++) {
            bufp[i] = _spi->transfer(0x00); // Send dummy data to receive
        }
        digitalWrite(_csPin, HIGH); // Pull CS high to deselect the device
        _spi->endTransaction(); // end the transaction

    } else {
        _i2c->beginTransmission(_address); // open the device
        _i2c->write(reg); // specify the starting register address
        _i2c->endTransmission(false);
        _i2c->requestFrom(_address, len); // Request len bytes from the device
  
        for (uint8_t i = 0; i < len; i++) {
            if (_i2c->available()) {
            bufp[i] = _i2c->read(); // Read the received byte
            }
        }
    }
  return 0;
}


int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len)
{
    if( _useSPI ){
        _spi->beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3)); // begin the transaction
        digitalWrite(_csPin,LOW); // select the MPU9250 chip
        _spi->transfer(reg); // write the register address

        // Transmit the buffer data
        for (uint8_t i = 0; i < len; i++) {
            _spi->transfer(bufp[i]);     
        }  

        digitalWrite(_csPin,HIGH); // deselect the MPU9250 chip
        _spi->endTransaction(); // end the transaction
    } else {
        _i2c->beginTransmission(_address); // open the device
        _i2c->write(reg); // write the register address
        // Write data from the buffer
        for (uint8_t i = 0; i < len; i++) {
            _i2c->write(bufp[i]);
        }
        _i2c->endTransmission();
    }
    
    return 0;
}


void platform_delay(uint32_t ms)
{
    delay(ms);
}


void platform_init(void)
{

}


/*
 * Original conversion routines taken from: https://github.com/numpy/numpy
 *
 * uint32_t npy_halfbits_to_floatbits(uint16_t h);
 * float_t npy_half_to_float(uint16_t h);
 *
 * Released under BSD-3-Clause License
 */

static uint32_t npy_halfbits_to_floatbits(uint16_t h)
{
    uint16_t h_exp, h_sig;
    uint32_t f_sgn, f_exp, f_sig;

    h_exp = (h&0x7c00u);
    f_sgn = ((uint32_t)h&0x8000u) << 16;
    switch (h_exp) {
        case 0x0000u: /* 0 or subnormal */
            h_sig = (h&0x03ffu);
            /* Signed zero */
            if (h_sig == 0) {
                return f_sgn;
            }
            /* Subnormal */
            h_sig <<= 1;
            while ((h_sig&0x0400u) == 0) {
                h_sig <<= 1;
                h_exp++;
            }
            f_exp = ((uint32_t)(127 - 15 - h_exp)) << 23;
            f_sig = ((uint32_t)(h_sig&0x03ffu)) << 13;
            return f_sgn + f_exp + f_sig;
        case 0x7c00u: /* inf or NaN */
            /* All-ones exponent and a copy of the significand */
            return f_sgn + 0x7f800000u + (((uint32_t)(h&0x03ffu)) << 13);
        default: /* normalized */
            /* Just need to adjust the exponent and shift */
            return f_sgn + (((uint32_t)(h&0x7fffu) + 0x1c000u) << 13);
    }
}

static float_t npy_half_to_float(uint16_t h)
{
    union { float_t ret; uint32_t retbits; } conv;
    conv.retbits = npy_halfbits_to_floatbits(h);
    return conv.ret;
}

static void sflp2q(float quat[4], uint16_t sflp[3])
{
  float sumsq = 0;

  quat[0] = npy_half_to_float(sflp[0]);
  quat[1] = npy_half_to_float(sflp[1]);
  quat[2] = npy_half_to_float(sflp[2]);

  for (uint8_t i = 0; i < 3; i++)
    sumsq += quat[i] * quat[i];

  if (sumsq > 1.0f)
    sumsq = 1.0f;

  quat[3] = sqrtf(1.0f - sumsq);
}

#endif

