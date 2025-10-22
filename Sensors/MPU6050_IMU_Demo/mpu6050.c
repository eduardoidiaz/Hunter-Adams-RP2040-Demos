/**
 * Hunter Adams (vha3@cornell.edu)
 * 
 *
 */

#include "hardware/i2c.h"
#include "mpu6050.h"
#include "pico/time.h"
#include <pico/types.h>
#define TIMEOUT 900 // us for timeout

void mpu6050_reset() {
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in
    // different ways that could be added here
    uint8_t buf[] = {0x6B, 0x00};
    absolute_time_t timeout;
    timeout = make_timeout_time_us(TIMEOUT) ;
    i2c_write_blocking_until(I2C_CHAN, ADDRESS, buf, 2, false, timeout);

    // Set gyro sample rate (set to 1KHz, same as accel)
    uint8_t gyro_rate[] = {0x19, 0b00000111} ;
    timeout = make_timeout_time_us(TIMEOUT) ;
    i2c_write_blocking_until(I2C_CHAN, ADDRESS, gyro_rate, 2, false, timeout);

    // Configure the Gyro range (+/- 250 deg/s)
    uint8_t gyro_settings[] = {0x1b, 0b00000000} ;
    timeout = make_timeout_time_us(TIMEOUT) ;
    i2c_write_blocking_until(I2C_CHAN, ADDRESS, gyro_settings, 2, false, timeout);

    // Configure the Accel range (+/- 2g's)
    uint8_t accel_settings[] = {0x1c, 0b00000000} ;
    timeout = make_timeout_time_us(TIMEOUT) ;
    i2c_write_blocking_until(I2C_CHAN, ADDRESS, accel_settings, 2, false, timeout);

    // Configure interrupt pin
    uint8_t pin_settings[] = {0x37, 0b00010000} ;
    timeout = make_timeout_time_us(TIMEOUT) ;
    i2c_write_blocking_until(I2C_CHAN, ADDRESS, pin_settings, 2, false, timeout);

    // Configure data ready interrupt
    uint8_t int_config[] = {0x38, 0x01} ;
    timeout = make_timeout_time_us(TIMEOUT) ;
    i2c_write_blocking_until(I2C_CHAN, ADDRESS, int_config, 2, false, timeout);
}

void mpu6050_read_raw(fix15 accel[3], fix15 gyro[3]) {
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.

    uint8_t buffer[6];
    int16_t temp_accel, temp_gyro ;
    absolute_time_t timeout;

    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = 0x3B;
    timeout = make_timeout_time_us(TIMEOUT) ;
    i2c_write_blocking_until(I2C_CHAN, ADDRESS, &val, 1, true, timeout); // true to keep master control of bus
    timeout = make_timeout_time_us(TIMEOUT);
    i2c_read_blocking_until(I2C_CHAN, ADDRESS, buffer, 6, false, timeout);

    for (int i = 0; i < 3; i++) {
        temp_accel = (buffer[i<<1] << 8 | buffer[(i<<1) + 1]);
        accel[i] = temp_accel ;
        accel[i] <<= 2 ; // convert to g's (fixed point)
    }

    // Now gyro data from reg 0x43 for 6 bytes
    // The register is auto incrementing on each read
    val = 0x43;
    timeout = make_timeout_time_us(TIMEOUT) ;
    i2c_write_blocking_until(I2C_CHAN, ADDRESS, &val, 1, true, timeout);
    timeout = make_timeout_time_us(TIMEOUT);
    i2c_read_blocking_until(I2C_CHAN, ADDRESS, buffer, 6, false, timeout);  // False - finished with bus

    for (int i = 0; i < 3; i++) {
        temp_gyro = (buffer[i<<1] << 8 | buffer[(i<<1) + 1]);
        gyro[i] = temp_gyro ;
        gyro[i] = multfix15(gyro[i], 500<<16) ; // deg/sec
    }
}
/////////////////////////////////////////////////////////////////