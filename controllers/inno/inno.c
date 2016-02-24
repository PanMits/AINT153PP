/*
 * File:           inno.c
 * Date:           23/02/2016
 * Description:    Robot controller for navigating competition map
 * Author:         Sam Holmes, Panos Mitseas, Paul Holland
 */

// Dependencies
#include <webots/robot.h>
#include <webots/motor.h> 
#include <webots/distance_sensor.h>
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <math.h>


// Macros
#define TIME_STEP 64
#define MAX_SPEED 5.24
#define RANGE (1024 / 2)
#define BOUND(x, a, b) (((x)<(a))?(a):((x)>(b))?(b):(x))

// Main function
int main(int argc, char *argv[])
{

  // Get vehicle weights
  int w_2_left = 1;
  int w_5_left = 1;
  int w_2_right = 1;
  int w_5_right = 1;

  // Initialize webots
  wb_robot_init();
  
  
  // Get robot wheels
  WbDeviceTag right_wheel = wb_robot_get_device("right_wheel");
  WbDeviceTag left_wheel  = wb_robot_get_device("left_wheel");
   
  // Get sensors
  WbDeviceTag ir_sensor_2 = wb_robot_get_device("so2");
  wb_distance_sensor_enable(ir_sensor_2, TIME_STEP);
  WbDeviceTag ir_sensor_5 = wb_robot_get_device("so5");
  wb_distance_sensor_enable(ir_sensor_5, TIME_STEP);
  
  // Prepare for velocity commands
  wb_motor_set_position(left_wheel, INFINITY);
  wb_motor_set_position(right_wheel, INFINITY);
  wb_motor_set_velocity(left_wheel, 0.0);
  wb_motor_set_velocity(right_wheel, 0.0);
  
  
  // Main loop to move the robot
  while (wb_robot_step(TIME_STEP) != -1) {
  
    // Get the sensor data
    double ir2_val = wb_distance_sensor_get_value(ir_sensor_2);
    double ir5_val = wb_distance_sensor_get_value(ir_sensor_5);
    printf("Distance sensor so2: %.2f.\n",ir2_val);
    printf("Distance sensor so5: %.2f.\n",ir5_val);
    
    // Convert the sensor data
    double ir2_conv = 1 - ir2_val/RANGE;
    double ir5_conv = 1 - ir5_val/RANGE;
    printf("Converted distance sensor so2: %.2f.\n",ir2_conv);
    printf("Converted distance sensor so5: %.2f.\n",ir5_conv);
    
    // Calculate the speeds
    double left_speed = ir2_conv * w_2_left + ir5_conv * w_5_left;
    double right_speed = ir2_conv * w_2_right + ir5_conv * w_5_right;
    left_speed = BOUND(left_speed, -MAX_SPEED, MAX_SPEED);
    right_speed = BOUND(right_speed, -MAX_SPEED, MAX_SPEED);
    
    // Set the motors to the calculated speeds
    wb_motor_set_velocity(left_wheel, left_speed);
    wb_motor_set_velocity(right_wheel, right_speed);
    
  }
  
  // Clean up webots
  wb_robot_cleanup();
  
  return 0;
}