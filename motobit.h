#pragma once

#include "nrf_twi_mngr.h"

static const uint8_t MOTOBIT_I2C_ADDR = 0x59;
static const uint8_t CMD_ENABLE       = 0x70;
static const uint8_t CMD_SPEED_LEFT   = 0x21;
static const uint8_t CMD_SPEED_RIGHT  = 0x20;
static const uint8_t FORWARD_FLAG     = 0x80;

void motobit_enable(const nrf_twi_mngr_t* i2c);
void motobit_disable();
void motobit_drive(uint8_t cmd_speed, int speed, int invert);
void motobit_forward(int speed);
void motobit_reverse(int speed);

void i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t data);
uint8_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr);
uint16_t i2c_reg_read16(uint8_t i2c_addr, uint8_t reg_addr);