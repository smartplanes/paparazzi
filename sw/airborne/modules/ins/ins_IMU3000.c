/*
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

#include <math.h>
#include "ins_IMU3000.h"
#include "mcu_periph/i2c.h"
#include "led.h"

// Downlink
#include "mcu_periph/uart.h"
#include "messages.h"
#include "downlink.h"

#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif

// Peripherials
#include "../../peripherals/itg3200.h"
#include "../../peripherals/adxl345.h"

#define ADXL345_SLAVE_ADDR 0x53

// Results
volatile bool_t gyr_valid;
volatile bool_t acc_valid;

// Communication

struct i2c_transaction passthru_adxl345;
struct i2c_transaction imu3000;

// Standalone option: run module only
#ifndef IMU_TYPE_H
struct Imu imu;
#endif

#ifndef PERIODIC_FREQUENCY
#define PERIODIC_FREQUENCY 60
#endif

void imu_impl_init(void)
{
  /////////////////////////////////////////////////////////////////////
  // IMU3000
  imu3000.type = I2CTransTx;
  imu3000.slave_addr = ITG3200_ADDR;
  imu3000.buf[0] = ITG3200_REG_DLPF_FS;
#if PERIODIC_FREQUENCY == 60
  /* set gyro range to 2000deg/s and low pass at 20Hz (< 60Hz/2) internal sampling at 1kHz */
  imu3000.buf[1] = (0x03<<3) | (0x04<<0);
#  warning Info: IMU3000 read at 50Hz
#else
#  if PERIODIC_FREQUENCY == 120
#  warning Info: IMU3000 read at 100Hz
  /* set gyro range to 2000deg/s and low pass at 42Hz (< 120Hz/2) internal sampling at 1kHz */
  imu3000.buf[1] = (0x03<<3) | (0x03<<0);
#  else
#  error PERIODIC_FREQUENCY should be either 60Hz or 120Hz. Otherwise manually fix the sensor rates
#  endif
#endif
  imu3000.len_w = 2;
  i2c_submit(&IMU3000_I2C_DEVICE,&imu3000);
    while(imu3000.status == I2CTransPending);

  /* set sample rate to 66Hz: so at 60Hz there is always a new sample ready and you loose little */
  imu3000.buf[0] = ITG3200_REG_SMPLRT_DIV;
#if PERIODIC_FREQUENCY == 60
  imu3000.buf[1] = 19;  // 50Hz
#else
  imu3000.buf[1] = 9;  // 100Hz
#endif
  i2c_submit(&IMU3000_I2C_DEVICE,&imu3000);
    while(imu3000.status == I2CTransPending);

  /* switch to gyroX clock */
  imu3000.buf[0] = ITG3200_REG_PWR_MGM;
  imu3000.buf[1] = 0x01;
  i2c_submit(&IMU3000_I2C_DEVICE,&imu3000);
    while(imu3000.status == I2CTransPending);

  /* no interrupts for now, but set data ready interrupt to enable reading status bits */
  imu3000.buf[0] = ITG3200_REG_INT_CFG;
  imu3000.buf[1] = 0x01;
  i2c_submit(&IMU3000_I2C_DEVICE,&imu3000);
    while(imu3000.status == I2CTransPending);


 // set accelerometer register data address
 imu3000.buf[0] = 0x18;
 imu3000.buf[1] = ADXL345_REG_DATA_X0;
  i2c_submit(&IMU3000_I2C_DEVICE,&imu3000);
    while(imu3000.status == I2CTransPending);

 // set accelerometer i2c slave address
 imu3000.buf[0] = 0x14;
 imu3000.buf[1] = ADXL345_SLAVE_ADDR;
  i2c_submit(&IMU3000_I2C_DEVICE,&imu3000);
    while(imu3000.status == I2CTransPending);

  /////////////////////////////////////////////////////////////////////
  // ADXL345

  // set passthrough mode
 imu3000.buf[0] = 0x3D;
 imu3000.buf[1] = 0x08;
  i2c_submit(&IMU3000_I2C_DEVICE,&imu3000);
    while(imu3000.status == I2CTransPending);

  // set data rate to 100Hz 
  passthru_adxl345.slave_addr = ADXL345_ADDR;
  passthru_adxl345.type = I2CTransTx;
  passthru_adxl345.buf[0] = ADXL345_REG_BW_RATE;
#if PERIODIC_FREQUENCY == 60
  passthru_adxl345.buf[1] = 0x09;  // normal power and 50Hz sampling, 50Hz BW
#else
  passthru_adxl345.buf[1] = 0x0a;  // normal power and 100Hz sampling, 50Hz BW
#endif
  passthru_adxl345.len_w = 2;
  i2c_submit(&IMU3000_I2C_DEVICE,&passthru_adxl345);
    while(passthru_adxl345.status == I2CTransPending);

  // switch to measurement mode 
  passthru_adxl345.type = I2CTransTx;
  passthru_adxl345.buf[0] = ADXL345_REG_POWER_CTL;
  passthru_adxl345.buf[1] = 1<<3;
  passthru_adxl345.len_w = 2;
  i2c_submit(&IMU3000_I2C_DEVICE,&passthru_adxl345);
    while(passthru_adxl345.status == I2CTransPending);

  // Set range to 16g but keeping full resolution of 3.9 mV/g 
  passthru_adxl345.type = I2CTransTx;
  passthru_adxl345.buf[0] = ADXL345_REG_DATA_FORMAT;
  passthru_adxl345.buf[1] = 1<<3 | 0<<2 | 0x03;  // bit 3 is full resolution bit, bit 2 is left justify bit 0,1 are range: 00=2g 01=4g 10=8g 11=16g
  passthru_adxl345.len_w = 2;
  i2c_submit(&IMU3000_I2C_DEVICE,&passthru_adxl345);
    while(passthru_adxl345.status == I2CTransPending);

  // release passthrough mode
 imu3000.buf[0] = 0x3D;
 imu3000.buf[1] = 0x28;
  i2c_submit(&IMU3000_I2C_DEVICE,&imu3000);
    while(imu3000.status == I2CTransPending);

}

void imu_periodic( void )
{
  // Start reading the latest gyroscope data
  imu3000.type = I2CTransTxRx;
  imu3000.len_r = 15;
  imu3000.len_w = 1;
  imu3000.buf[0] = ITG3200_REG_INT_STATUS;
  i2c_submit(&IMU3000_I2C_DEVICE, &imu3000);

  RunOnceEvery(10,imu3000_module_downlink_raw());
}

void imu3000_module_downlink_raw( void )
{
  DOWNLINK_SEND_IMU_GYRO_RAW(DefaultChannel,&imu.gyro_unscaled.p,&imu.gyro_unscaled.q,&imu.gyro_unscaled.r);
  DOWNLINK_SEND_IMU_ACCEL_RAW(DefaultChannel,&imu.accel_unscaled.x,&imu.accel_unscaled.y,&imu.accel_unscaled.z);
}

void imu3000_module_event( void )
{
  int32_t xg, yg, zg, xa, ya, za;

  // If the imu3000 I2C transaction has succeeded: convert the data
  if (imu3000.status == I2CTransSuccess)
  {
#define ITG_STA_DAT_OFFSET 3

    // gyro is MSB first
    xg = (int16_t) ((imu3000.buf[0+ITG_STA_DAT_OFFSET] << 8) | imu3000.buf[1+ITG_STA_DAT_OFFSET]);
    yg = (int16_t) ((imu3000.buf[2+ITG_STA_DAT_OFFSET] << 8) | imu3000.buf[3+ITG_STA_DAT_OFFSET]);
    zg = (int16_t) ((imu3000.buf[4+ITG_STA_DAT_OFFSET] << 8) | imu3000.buf[5+ITG_STA_DAT_OFFSET]);

    // accel is LSB first
    xa = (int16_t) ((imu3000.buf[7+ITG_STA_DAT_OFFSET] << 8) | imu3000.buf[6+ITG_STA_DAT_OFFSET]);
    ya = (int16_t) ((imu3000.buf[9+ITG_STA_DAT_OFFSET] << 8) | imu3000.buf[8+ITG_STA_DAT_OFFSET]);
    za = (int16_t) ((imu3000.buf[11+ITG_STA_DAT_OFFSET] << 8) | imu3000.buf[10+ITG_STA_DAT_OFFSET]);

    // Is this is new data
    if (imu3000.buf[0] & 0x01)
    {
      //LED_ON(3);
      gyr_valid = TRUE;
      acc_valid = TRUE;
      //LED_OFF(3);
    }
    else
    {
    }

    // Signs depend on the way sensors are soldered on the board: so they are hardcoded

    RATES_ASSIGN(imu.gyro_unscaled, yg, xg, -zg);
    VECT3_ASSIGN(imu.accel_unscaled, -ya, -xa, -za);

    imu3000.status = I2CTransDone;  // remove the I2CTransSuccess status, otherwise data ready will be triggered again without new data
  }



}
