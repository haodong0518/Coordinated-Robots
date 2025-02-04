// LSM303AGR driver for Microbit_v2
//
// Initializes sensor and communicates over I2C
// Capable of reading temperature, acceleration, and magnetic field strength

#include <stdbool.h>
#include <stdint.h>
#include "app_timer.h"
#include <math.h>
#include "lsm303agr.h"
#include "nrf_delay.h"


APP_TIMER_DEF(temp_timer);

// Pointer to an initialized I2C instance to use for transactions
static const nrf_twi_mngr_t* i2c_manager = NULL;

// Helper function to perform a 1-byte I2C read of a given register
//
// i2c_addr - address of the device to read from
// reg_addr - address of the register within the device to read
//
// returns 8-bit read value
static uint8_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr) {
  uint8_t rx_buf = 0;
  nrf_twi_mngr_transfer_t const read_transfer[] = {
    //TODO: implement me
    NRF_TWI_MNGR_WRITE(i2c_addr, &reg_addr, 1, NRF_TWI_MNGR_NO_STOP),
    NRF_TWI_MNGR_READ(i2c_addr, &rx_buf, 1, 0),
  };
  //2 transactions, 1 write and 1 read
  ret_code_t result = nrf_twi_mngr_perform(i2c_manager, NULL, read_transfer, 2, NULL);
  if (result != NRF_SUCCESS) {
    // Likely error codes:
    //  NRF_ERROR_INTERNAL            (0x0003) - something is wrong with the driver itself
    //  NRF_ERROR_INVALID_ADDR        (0x0010) - buffer passed was in Flash instead of RAM
    //  NRF_ERROR_BUSY                (0x0011) - driver was busy with another transfer still
    //  NRF_ERROR_DRV_TWI_ERR_OVERRUN (0x8200) - data was overwritten during the transaction
    //  NRF_ERROR_DRV_TWI_ERR_ANACK   (0x8201) - i2c device did not acknowledge its address
    //  NRF_ERROR_DRV_TWI_ERR_DNACK   (0x8202) - i2c device did not acknowledge a data byte
    printf("I2C transaction failed! Error: %lX\n", result);
  }

  return rx_buf;
}

// Helper function to perform a 1-byte I2C write of a given register
// i2c_addr - address of the device to write to
// reg_addr - address of the register within the device to write
static void i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t data) {
  //Note: there should only be a single two-byte transfer to be performed
  uint8_t tx_buf[2] = {reg_addr, data};//2 bytes write
  nrf_twi_mngr_transfer_t const write_transfer[] = {
    NRF_TWI_MNGR_WRITE(i2c_addr, tx_buf, 2, 0)
  };
  ret_code_t result = nrf_twi_mngr_perform(i2c_manager, NULL, write_transfer, 1, NULL);//1 transaction
  if (result != NRF_SUCCESS) {
    printf("I2C transaction failed! Error: %lX\n", result);
  }
}

void helper_callback(){
  lsm303agr_read_temperature();
  lsm303agr_read_accelerometer();
  lsm303agr_read_magnetometer();
  //tilt_angle();
  }

// Initialize and configure the LSM303AGR accelerometer/magnetometer
//
// i2c - pointer to already initialized and enabled twim instance
void lsm303agr_init(const nrf_twi_mngr_t* i2c) {
  i2c_manager = i2c;

  // ---Initialize Accelerometer---

  // Reboot acclerometer
  i2c_reg_write(LSM303AGR_ACC_ADDRESS, CTRL_REG5_A, 0x80);
  nrf_delay_ms(100); // needs delay to wait for reboot

  // Enable Block Data Update
  // Only updates sensor data when both halves of the data has been read
  i2c_reg_write(LSM303AGR_ACC_ADDRESS, CTRL_REG4_A, 0x80);

  // Configure accelerometer at 100Hz, normal mode (10-bit)
  // Enable x, y and z axes
  i2c_reg_write(LSM303AGR_ACC_ADDRESS, CTRL_REG1_A, 0x57);

  // Read WHO AM I register
  // Always returns the same value if working
  //TODO: read the Accelerometer WHO AM I register and check the result
  uint8_t acc_result = i2c_reg_read(LSM303AGR_ACC_ADDRESS, 0x0F);
  printf("Accelerometer WHO AM I: %X\n", acc_result);
  // ---Initialize Magnetometer---

  // Reboot magnetometer
  i2c_reg_write(LSM303AGR_MAG_ADDRESS, CFG_REG_A_M, 0x40);
  nrf_delay_ms(100); // needs delay to wait for reboot

  // Enable Block Data Update
  // Only updates sensor data when both halves of the data has been read
  i2c_reg_write(LSM303AGR_MAG_ADDRESS, CFG_REG_C_M, 0x10);

  // Configure magnetometer at 100Hz, continuous mode
  i2c_reg_write(LSM303AGR_MAG_ADDRESS, CFG_REG_A_M, 0x0C);

  // Read WHO AM I register
  //TODO: read the Magnetometer WHO AM I register and check the result

  uint8_t mag_result = i2c_reg_read(LSM303AGR_MAG_ADDRESS, 0x4F);
  printf("Magnetometer WHO AM I: %X\n", mag_result);

  // ---Initialize Temperature---

  // Enable temperature sensor
  i2c_reg_write(LSM303AGR_ACC_ADDRESS, TEMP_CFG_REG_A, 0xC0);
  app_timer_init();
  app_timer_create(&temp_timer, APP_TIMER_MODE_REPEATED, helper_callback);
  app_timer_start(temp_timer, 32768/2, NULL);
}

// Read the internal temperature sensor
//
// Return measurement as floating point value in degrees C
float lsm303agr_read_temperature(void) {
  //TODO: implement me
  uint8_t reg1_val = i2c_reg_read(LSM303AGR_ACC_ADDRESS, OUT_TEMP_L_A);//least significant 8 bits
  uint8_t reg2_val = i2c_reg_read(LSM303AGR_ACC_ADDRESS, OUT_TEMP_H_A);//most significant 8 bits
  // printf("Temperature: %X %X\n", reg1_val, reg2_val);
  int16_t temp = ((uint16_t)reg2_val << 8) | (uint16_t)reg1_val;//combine the two 8-bit values
  //convert to degrees C
  float temp_c = (float)temp * (1.0 / 256.0) + 25.0;
  printf("Temperature: %f celcius degrees\n", temp_c);
  return temp_c;
}

lsm303agr_measurement_t lsm303agr_read_accelerometer(void) {
  //TODO: implement me
  uint8_t regx1_val = i2c_reg_read(LSM303AGR_ACC_ADDRESS, OUT_X_L_A);
  uint8_t regx2_val = i2c_reg_read(LSM303AGR_ACC_ADDRESS, OUT_X_H_A);
  int16_t x = ((uint16_t)regx2_val << 8) | (uint16_t)regx1_val;
  float x_f = (float)(x >> 6) * 3.9;
  uint8_t regy1_val = i2c_reg_read(LSM303AGR_ACC_ADDRESS, OUT_Y_L_A);
  uint8_t regy2_val = i2c_reg_read(LSM303AGR_ACC_ADDRESS, OUT_Y_H_A);
  int16_t y = ((uint16_t)regy2_val << 8) | (uint16_t)regy1_val;
  float y_f = (float)(y >> 6) * 3.9;
  uint8_t regz1_val = i2c_reg_read(LSM303AGR_ACC_ADDRESS, OUT_Z_L_A);
  uint8_t regz2_val = i2c_reg_read(LSM303AGR_ACC_ADDRESS, OUT_Z_H_A);
  int16_t z = ((uint16_t)regz2_val << 8) | (uint16_t)regz1_val;
  float z_f = (float)(z >> 6) * 3.9;
  lsm303agr_measurement_t measurement = {x_f * 0.001, y_f * 0.001, z_f * 0.001};
  printf("Accelerometer x: %f g\n", measurement.x_axis);
  printf("Accelerometer y: %f g\n", measurement.y_axis);
  printf("Accelerometer z: %f g\n", measurement.z_axis);
  return measurement;
}

lsm303agr_measurement_t lsm303agr_read_magnetometer(void) {
  //TODO: implement me
  uint8_t regx1_val = i2c_reg_read(LSM303AGR_MAG_ADDRESS, OUTX_L_REG_M);
  uint8_t regx2_val = i2c_reg_read(LSM303AGR_MAG_ADDRESS, OUTX_H_REG_M);
  int16_t x = ((uint16_t)regx2_val << 8) | (uint16_t)regx1_val;
  float x_f = (float)x * 1.5; 
  uint8_t regy1_val = i2c_reg_read(LSM303AGR_MAG_ADDRESS, OUTY_L_REG_M);
  uint8_t regy2_val = i2c_reg_read(LSM303AGR_MAG_ADDRESS, OUTY_H_REG_M);
  int16_t y = ((uint16_t)regy2_val << 8) | (uint16_t)regy1_val;
  float y_f = (float)y * 1.5;
  uint8_t regz1_val = i2c_reg_read(LSM303AGR_MAG_ADDRESS, OUTZ_L_REG_M);
  uint8_t regz2_val = i2c_reg_read(LSM303AGR_MAG_ADDRESS, OUTZ_H_REG_M);
  int16_t z = ((uint16_t)regz2_val << 8) | (uint16_t)regz1_val;
  float z_f = (float)z * 1.5;
  lsm303agr_measurement_t measurement = {x_f/10.0, y_f/10.0, z_f/10.0};
  printf("Magnetometer x: %f microtesla\n", measurement.x_axis);
  printf("Magnetometer y: %f microtesla\n", measurement.y_axis);
  printf("Magnetometer z: %f microtesla\n", measurement.z_axis);
  return measurement;
}

void tilt_angle(void){
  lsm303agr_measurement_t measurement = lsm303agr_read_accelerometer();
  float x = measurement.x_axis;
  float y = measurement.y_axis;
  float z = measurement.z_axis;
  float theta = atan(x / sqrt(pow(y, 2) + pow(z, 2))) * 180 / 3.14159265;
  float psi = atan(y / sqrt(pow(x, 2) + pow(z, 2))) * 180 / 3.14159265;
  float phi = atan(sqrt(pow(x, 2) + pow(y, 2)) / z) * 180 / 3.14159265;
  printf("Theta: %f degrees, Psi: %f degrees, Phi: %f degrees", theta, psi, phi);
}

