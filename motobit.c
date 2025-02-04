#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "nrf_delay.h"
#include "lsm303agr.h"
#include "motobit.h"  // Include I2C library or implement I2C functions


static const nrf_twi_mngr_t* i2c_manager = NULL;
#define UNIT_CM 0
#define UNIT_INCH 1

uint8_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr) {
  uint8_t rx_buf = 0;
  nrf_twi_mngr_transfer_t const read_transfer[] = {
    NRF_TWI_MNGR_WRITE(i2c_addr, &reg_addr, 1, NRF_TWI_MNGR_NO_STOP),
    NRF_TWI_MNGR_READ(i2c_addr, &rx_buf, 1, 0)
  };
  nrf_twi_mngr_perform(i2c_manager, NULL, read_transfer, 2, NULL);
  return rx_buf;
}

uint16_t i2c_reg_read16(uint8_t i2c_addr, uint8_t reg_addr) {
  uint8_t raw_dis[2] = {};
  nrf_twi_mngr_transfer_t const read_transfer[] = {
    NRF_TWI_MNGR_WRITE(i2c_addr, &reg_addr, 1, NRF_TWI_MNGR_NO_STOP),
    NRF_TWI_MNGR_READ(i2c_addr, &raw_dis, 2, 0)
  };
  nrf_twi_mngr_perform(i2c_manager, NULL, read_transfer, 2, NULL);
  return (raw_dis[0] << 8) | raw_dis[1];
}


void i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t data) {
  uint8_t array[2] = {reg_addr, data};
  nrf_twi_mngr_transfer_t const write_transfer[] = {
    NRF_TWI_MNGR_WRITE(i2c_addr, array, 2, 0),
  };
  nrf_twi_mngr_perform(i2c_manager, NULL, write_transfer, 1, NULL);
}


void motobit_enable(const nrf_twi_mngr_t* i2c) {
    i2c_manager = i2c;    
    nrf_delay_ms(100);
    i2c_reg_write(MOTOBIT_I2C_ADDR, 0x70,0x01); // Your I2C write function here
}

void motobit_disable() {
    i2c_reg_write(MOTOBIT_I2C_ADDR, 0x70,0x00); // Your I2C write function here
}

void motobit_drive(uint8_t cmd_speed, int speed, int invert) {
    uint8_t flags = 0;
    if (invert) {
        speed = -speed;
    }
    if (speed >= 0) {
        flags |= FORWARD_FLAG;
    }
    speed = (int)((float)speed / 100 * 127);
    if (speed < -127) {
        speed = -127;
    }
    if (speed > 127) {
        speed = 127;
    }
    speed = (speed & 0x7F) | flags;
    i2c_reg_write(MOTOBIT_I2C_ADDR, cmd_speed,speed); // Your I2C write function here
}

void motobit_forward(int speed) {
    motobit_drive(CMD_SPEED_LEFT, speed, 0);
    motobit_drive(CMD_SPEED_RIGHT, speed, 0);
}

void motobit_reverse(int speed) {
    motobit_drive(CMD_SPEED_LEFT, speed, 1);
    motobit_drive(CMD_SPEED_RIGHT, speed, 1);
}
