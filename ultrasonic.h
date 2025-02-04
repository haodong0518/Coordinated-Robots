#include "nrf_twi_mngr.h"

static const uint8_t ULTRASONIC_ADDRESS = 0x00;
// 0x00 or 0x2F
// register definitions for ultrasonic sensor
// Find from the schematic diagram, PB1 - Din1, PD0 - Din2

static const uint8_t ULTRASONIC_REG_DIS = 0X01;