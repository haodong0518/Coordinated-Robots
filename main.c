// I2C sensors app
//
// Read from I2C accelerometer/magnetometer on the Microbit

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

#include "app_timer.h"

#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_twi_mngr.h"
#include "nrfx_pwm.h"

#include "microbit_v2.h"
#include "motobit.h"
#include "lsm303agr.h"

#include "ultrasonic.h"

APP_TIMER_DEF(my_timer_1);
APP_TIMER_DEF(my_timer_2);
APP_TIMER_DEF(my_timer_3);
// Global variables
// NRF_TWI_MNGR_DEF(twi_mngr_instance, 1, 0);
NRF_TWI_MNGR_DEF(motobit_twi_mngr_instance, 1, 0);
// NRF_TWI_MNGR_DEF(ultrasonic_twi_mngr_instance, 1, 0);

// Control the motors
static x = 0;
static y = 0;
static ultrasonic_dis = 0;
static void control_motors(void* _unused){
  int8_t directionL = 1;
  int8_t directionR = 0;
  int speedL = y-x;
  int speedR = (y+x)+8;
  motobit_drive(CMD_SPEED_LEFT, speedL, directionL);
  motobit_drive(CMD_SPEED_RIGHT, speedR, directionR);
  printf("SpeedL %d:SpeedR %d\n", speedL, speedR);
}


static void ultrasonic_callback(void* _unused){
  //i2c_reg_write(ULTRASONIC_ADDRESS, ULTRASONIC_REG_DIS, 0x01);
  uint16_t distance = i2c_reg_read16(ULTRASONIC_ADDRESS, ULTRASONIC_REG_DIS);
  ultrasonic_dis = distance/10;
  printf("Distance: %d\n", ultrasonic_dis);
  //nrf_delay_ms(250);
}

static void set_speed(int x_request, int y_request, int inverted){
  if (inverted==1){
    x_request = -x_request;
    y_request = -y_request;
  }
  x = x_request;
  y = y_request;
}

// static void control_loop(void* _unused){
//   int goal_distance = 12;
//   int error = ultrasonic_dis - goal_distance;
//   int Kp = 6;
//   int u = Kp * error;
//   printf("error: %d \n", error);
//   if(u < 0){
//     u = -u;
//     set_speed(x, u, 1);
//   }
//   else if (u>0)
//   {
//   set_speed(x , u, 0);
//   }}

static int previous_error = 0;  // Keep track of the previous error

static void control_loop(void* _unused) {
    int goal_distance = 15;
    int Kp = 4;  // Proportional gain
    int Kd = 3;  // Derivative gain

    int error = ultrasonic_dis - goal_distance;  // Compute the current error

    // Compute the derivative of the error
    int derivative = error - previous_error;

    // Compute the control signal as the sum of proportional and derivative terms
    int u = Kp * error + Kd * derivative;

    printf("error: %d, derivative: %d, control signal u: %d\n", error, derivative, u);
    
    // if the ultrasonic dis is between 5 or 35, the robot will move forward
    // elif the ultrasonic dis is out of the range, the robot will rotate
    if (ultrasonic_dis > 5 && ultrasonic_dis < 35) {
        if (u < 0) {
        u = -u;
        set_speed(x, u, 1);  // Reverse direction
    } else if (u > 0) {
        set_speed(x, u, 0);  // Forward direction
    }
    } else {
      // while (ultrasonic_dis < 5 || ultrasonic_dis > 35) {
      //   set_speed(20, -20, 0);
      //   //nrf_delay_ms(300);
      //   ultrasonic_callback(NULL); // Update ultrasonic_dis
      // }
    }
    
    // Update the previous error for the next iteration
    previous_error = error;
}


int main(void) {
  // initialize motobit
  nrf_drv_twi_config_t motobit_config = NRF_DRV_TWI_DEFAULT_CONFIG;
  motobit_config.scl = EDGE_P19;
  motobit_config.sda = EDGE_P20;
  motobit_config.frequency = NRF_TWI_FREQ_100K;
  motobit_config.interrupt_priority = 0;

  // nrf_drv_twi_config_t ultrasonic_config = NRF_DRV_TWI_DEFAULT_CONFIG;
  // ultrasonic_config.scl = EDGE_P19;
  // ultrasonic_config.sda = EDGE_P20;
  // ultrasonic_config.interrupt_priority = 2;
  nrf_twi_mngr_init(&motobit_twi_mngr_instance, &motobit_config);
  //nrf_twi_mngr_init(&ultrasonic_twi_mngr_instance, &ultrasonic_config);

  printf("Enabling MotoBit...\n");
  motobit_enable(&motobit_twi_mngr_instance);
  //nrf_delay_ms(1000);
  printf("MotoBit enabled.\n");
  
  // initialize app timers
  app_timer_init();
  app_timer_create(&my_timer_1, APP_TIMER_MODE_REPEATED, control_motors);
  app_timer_start(my_timer_1, 32768/100, NULL);
  app_timer_create(&my_timer_2, APP_TIMER_MODE_REPEATED, ultrasonic_callback);
  app_timer_start(my_timer_2, 32768/80, NULL);
  app_timer_create(&my_timer_3, APP_TIMER_MODE_REPEATED, control_loop);
  app_timer_start(my_timer_3, 32768/50, NULL);

  while (1) {
  }
}
