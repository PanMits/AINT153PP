/*
 * File:          hello_world.c
 * Date:          06/02/2016
 * Description:   A simple hello world controller, with a tail-chasing robot!
 * Author:        Martin F. Stoelen
 */
 

/*
 * You may need to add include files like <webots/distance_sensor.h> etc.
 */
#include <webots/robot.h>
#include <webots/motor.h> 
#include <stdio.h>


/*
 * You may want to add defines macro here.
 */
#define TIME_STEP 64


int main(int argc, char **argv)
{

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
    * Enter here functions to send actuator commands.
    */
    double left_speed, right_speed;
    left_speed = 3.0;
    right_speed = 1.5;
    
    /*Set the motor speeds.*/
    wb_motor_set_velocity(left_wheel, left_speed);
    wb_motor_set_velocity(right_wheel, right_speed);
    
    /*Add your hello world below! 
    * For example: printf("Hello World from Martin!\n");
    */
    
    
  }
  
  /* Necessary to cleanup webots stuff */
  wb_robot_cleanup();
  
  return 0;
}