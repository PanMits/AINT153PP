/*
 * File:          braitenberg.c
 * Date:          06/02/2016
 * Description:   A simple example of a Braitenberg vehicle type controller for the Pioneer robot, based on Webots example.
 * Author:        Martin F. Stoelen
 */
 

/*
 * You may need to add include files like <webots/distance_sensor.h> etc.
 */
#include <webots/robot.h>
#include <webots/motor.h> 
#include <webots/distance_sensor.h>
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <math.h>


/*
 * You may want to add defines macro here.
 */
#define TIME_STEP 64
#define MAX_SPEED 5.24
#define RANGE (1024 / 2)
#define BOUND(x, a, b) (((x)<(a))?(a):((x)>(b))?(b):(x))


int main(int argc, char **argv)
{

  if(argc < 4) {
    printf("Error: Need 4 Braitenberg vehicle weights as input parameters:\n");
    printf("1. Sensor 2 to left wheel.\n");
    printf("2. Sensor 5 to left wheel.\n");
    printf("3. Sensor 2 to right wheel.\n");
    printf("4. Sensor 5 to right wheel.\n");
    assert(false);
  }

  /* get Braitenberg vehicle weights */
  int w_2_left = atoi(argv[1]);
  int w_5_left = atoi(argv[2]);
  int w_2_right = atoi(argv[3]);
  int w_5_right = atoi(argv[4]);

  /* necessary to initialize webots stuff */
  wb_robot_init();
  
  
  /*
  * You should declare here DeviceTag variables for storing
  * robot devices like this:
  *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
  *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
  */
  WbDeviceTag left_wheel  = wb_robot_get_device("left_wheel");
  WbDeviceTag right_wheel = wb_robot_get_device("right_wheel");
  
  WbDeviceTag my_ir_sensor_2 = wb_robot_get_device("so2");
  wb_distance_sensor_enable(my_ir_sensor_2, TIME_STEP);
  WbDeviceTag my_ir_sensor_5 = wb_robot_get_device("so5");
  wb_distance_sensor_enable(my_ir_sensor_5, TIME_STEP);
  
  /*
  * Prepare robot for velocity commands
  */
  wb_motor_set_position(left_wheel, INFINITY);
  wb_motor_set_position(right_wheel, INFINITY);
  wb_motor_set_velocity(left_wheel, 0.0);
  wb_motor_set_velocity(right_wheel,0.0);
  
  
  /* 
  * Main loop.
  * Perform a simulation step of 64 milliseconds
  * and leave the loop when the simulation is over
  */
  while (wb_robot_step(TIME_STEP) != -1) {
  
    /* 
     * Read the sensors:
     * Enter here functions to read sensor data, like:
     *  double val = wb_distance_sensor_get_value(my_sensor);
    */
    double ir2_val = wb_distance_sensor_get_value(my_ir_sensor_2);
    double ir5_val = wb_distance_sensor_get_value(my_ir_sensor_5);
    printf("Distance sensor so2: %.2f.\n",ir2_val);
    printf("Distance sensor so5: %.2f.\n",ir5_val);
    
    /* 
     * Convert sensor readings to a useful range. 
     * So, 1 when out of range, decreasing with proximity, 
     * then negative when closer than RANGE.
    */
    double ir2_conv = 1 - ir2_val/RANGE;
    double ir5_conv = 1 - ir5_val/RANGE;
    printf("Converted distance sensor so2: %.2f.\n",ir2_conv);
    printf("Converted distance sensor so5: %.2f.\n",ir5_conv);
    
    /*
    * Enter here functions to send actuator commands.
    */
    double left_speed, right_speed;
    
    left_speed = ir2_conv * w_2_left + ir5_conv * w_5_left;
    right_speed = ir2_conv * w_2_right + ir5_conv * w_5_right;
    
    left_speed = BOUND(left_speed, -MAX_SPEED, MAX_SPEED);
    right_speed = BOUND(right_speed, -MAX_SPEED, MAX_SPEED);
    
    /*Set the motor speeds.*/
    wb_motor_set_velocity(left_wheel, left_speed);
    wb_motor_set_velocity(right_wheel, right_speed);
    
  }
  
  /* Necessary to cleanup webots stuff */
  wb_robot_cleanup();
  
  return 0;
}