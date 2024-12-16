#include "lsm6dsv32x.h"


/**
 * @brief Low-level read function for LSM6DSV32X over SPI or I2C.
 * 
 * @param handle            Pointer to the LSM6DSV32X device handle.
 * @param pBuffer           Pointer to the buffer where read data will be stored.
 * @param RegisterAddr      The address of the register to read from.
 * @param NumByteToRead     Number of bytes to read.
 * @return uint8_t          0 on success, non-zero on failure.
 */
uint8_t IO_Read(LSM6DSV32X_Handle_t *handle, uint8_t *pBuffer, uint8_t RegisterAddr, uint16_t NumByteToRead) {
    // SPI communication
    if (handle->dev_spi) {
        handle->dev_spi->beginTransaction(SPISettings(handle->spi_speed, MSBFIRST, SPI_MODE3));
        digitalWrite(handle->cs_pin, LOW);

        handle->dev_spi->transfer(RegisterAddr | 0x80);
        for (uint16_t i = 0; i < NumByteToRead; i++) {
            pBuffer[i] = handle->dev_spi->transfer(0x00);
        }

        digitalWrite(handle->cs_pin, HIGH);
        handle->dev_spi->endTransaction();

        return 0;
    }

    // I2C communication
    if (handle->dev_i2c) {
        handle->dev_i2c->beginTransmission(handle->address);
        handle->dev_i2c->write(RegisterAddr);
        handle->dev_i2c->endTransmission(false);

        handle->dev_i2c->requestFrom(handle->address, (uint8_t)NumByteToRead);
        int i = 0;
        while (handle->dev_i2c->available()) {
            pBuffer[i++] = handle->dev_i2c->read();
        }
        return 0;
    }

    return 1; // Error if no communication interface found
}

/**
 * @brief Low-level write function for LSM6DSV32X over SPI or I2C.
 * 
 * @param handle            Pointer to the LSM6DSV32X device handle.
 * @param pBuffer           Pointer to the buffer containing data to write.
 * @param RegisterAddr      The address of the register to write to.
 * @param NumByteToWrite    Number of bytes to write.
 * @return uint8_t          0 on success, non-zero on failure.
 */
uint8_t IO_Write(LSM6DSV32X_Handle_t *handle, const uint8_t *pBuffer, uint8_t RegisterAddr, uint16_t NumByteToWrite) {
    // SPI communication
    if (handle->dev_spi) {
        handle->dev_spi->beginTransaction(SPISettings(handle->spi_speed, MSBFIRST, SPI_MODE3));
        digitalWrite(handle->cs_pin, LOW);

        handle->dev_spi->transfer(RegisterAddr);
        for (uint16_t i = 0; i < NumByteToWrite; i++) {
            handle->dev_spi->transfer(pBuffer[i]);
        }

        digitalWrite(handle->cs_pin, HIGH);
        handle->dev_spi->endTransaction();
        return 0;
    }

    // I2C communication
    if (handle->dev_i2c) {
        handle->dev_i2c->beginTransmission(handle->address);
        handle->dev_i2c->write(RegisterAddr);
        for (uint16_t i = 0; i < NumByteToWrite; i++) {
            handle->dev_i2c->write(pBuffer[i]);
        }
        handle->dev_i2c->endTransmission(true);
        return 0;
    }

    return 1; // Error if no communication interface found
}

/**
 * @brief Platform-specific read function (callback for ST driver).
 * 
 * @param handle    Pointer to device handle (cast from user context).
 * @param reg       Register address to read.
 * @param bufp      Pointer to buffer to store data.
 * @param len       Number of bytes to read.
 * @return int32_t  0 on success, non-zero on failure.
 */
int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {
    return IO_Read((LSM6DSV32X_Handle_t *)handle, bufp, reg, len);
}

/**
 * @brief Platform-specific write function (callback for ST driver).
 * 
 * @param handle    Pointer to device handle (cast from user context).
 * @param reg       Register address to write.
 * @param bufp      Pointer to data buffer.
 * @param len       Number of bytes to write.
 * @return int32_t  0 on success, non-zero on failure.
 */
int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len) {
    return IO_Write((LSM6DSV32X_Handle_t *)handle, bufp, reg, len);
}

/**
 * @brief Platform-specific delay function.
 * 
 * @param ms    Time in milliseconds to delay.
 */
void platform_delay(uint32_t ms) {
    delay(ms);
}

/**
 * @brief Convert a half-precision float (16-bit) to a single-precision float (32-bit).
 * 
 * @param h Half-precision float bits.
 * @return uint32_t Single-precision float bits.
 */
uint32_t npy_halfbits_to_floatbits(uint16_t h)
{
    uint16_t h_exp, h_sig;
    uint32_t f_sgn, f_exp, f_sig;

    h_exp = (h & 0x7C00u);
    f_sgn = ((uint32_t)h & 0x8000u) << 16;
    switch (h_exp) {
        case 0x0000u: /* 0 or subnormal */
            h_sig = (h & 0x03FFu);
            /* Signed zero */
            if (h_sig == 0) {
                return f_sgn;
            }
            /* Subnormal */
            h_sig <<= 1;
            while ((h_sig & 0x0400u) == 0) {
                h_sig <<= 1;
                h_exp++;
            }
            f_exp = ((uint32_t)(127 - 15 - h_exp)) << 23;
            f_sig = ((uint32_t)(h_sig & 0x03FFu)) << 13;
            return f_sgn + f_exp + f_sig;
        case 0x7C00u: /* inf or NaN */
            /* All-ones exponent and a copy of the significand */
            return f_sgn + 0x7F800000u + (((uint32_t)(h & 0x03FFu)) << 13);
        default: /* normalized */
            /* Just need to adjust the exponent and shift */
            return f_sgn + (((uint32_t)(h & 0x7FFFu) + 0x1C000u) << 13);
    }
}

/**
 * @brief Convert half-precision float to single-precision float.
 * 
 * @param h Half-precision float bits.
 * @return float Single-precision float.
 */
float npy_half_to_float(uint16_t h)
{
    union { float ret; uint32_t retbits; } conv;
    conv.retbits = npy_halfbits_to_floatbits(h);
    return conv.ret;
}

/**
 * @brief Convert a 3D scaled floating point vector (half-precision) into a quaternion.
 * 
 * @param quat  Array to store the resulting quaternion (quat[0..3]).
 * @param sflp  Array of three half-precision floats representing the first three components.
 */
void sflp2q(float quat[4], uint16_t sflp[3])
{
    float sumsq = 0;

    quat[0] = npy_half_to_float(sflp[0]);
    quat[1] = npy_half_to_float(sflp[1]);
    quat[2] = npy_half_to_float(sflp[2]);

    uint8_t i = 0;
    for (i = 0; i < 3; i++)
        sumsq += quat[i] * quat[i];

    if (sumsq > 1.0f)
        sumsq = 1.0f;

    quat[3] = sqrtf(1.0f - sumsq);
}


