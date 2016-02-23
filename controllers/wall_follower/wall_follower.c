/*
 * File:          wall_follower.c
 * Date:          15/02/2016
 * Description:   A simple example of a wall follower for the Pioneer robot.
 * Author:        Martin F. Stoelen
 */
 

/*
 * You may need to add include files like <webots/distance_sensor.h> etc.
 */
#include <stdio.h>
#include <webots/robot.h>
#include <webots/motor.h> 
#include <webots/distance_sensor.h>


/*
 * You may want to add defines macro here.
 */
#define TIME_STEP 64
#define MAX_SPEED 3.0


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
  
  WbDeviceTag forward_left_sensor = wb_robot_get_device("so3");
  wb_distance_sensor_enable(forward_left_sensor, TIME_STEP);
  WbDeviceTag forward_right_sensor = wb_robot_get_device("so4");
  wb_distance_sensor_enable(forward_right_sensor, TIME_STEP);
  WbDeviceTag right_sensor = wb_robot_get_device("so6");
  wb_distance_sensor_enable(right_sensor, TIME_STEP);
  
    
  /*
  * Prepare robot for velocity commands
  */
  wb_motor_set_position(left_wheel, INFINITY);
  wb_motor_set_position(right_wheel, INFINITY);
  wb_motor_set_velocity(left_wheel, 0.0);
  wb_motor_set_velocity(right_wheel,0.0);
  
  
  // Begin in mode 0, moving forward
  int mode  = 0;
  
  /* 
  * Main loop.
  * Perform a simulation step of 64 milliseconds
  * and leave the loop when the simulation is over
  */
  while (wb_robot_step(TIME_STEP) != -1) {
  
    /* 
     * Read the sensors :
     * Enter here functions to read sensor data, like:
     *  double val = wb_distance_sensor_get_value(my_sensor);
    */
    double forward_left_value = wb_distance_sensor_get_value(forward_left_sensor);
    double forward_right_value = wb_distance_sensor_get_value(forward_right_sensor);
    double right_value = wb_distance_sensor_get_value(right_sensor);
    printf("Sensor input values:\n");
    printf("- Forward left: %.2f.\n",forward_left_value);
    printf("- Forward right: %.2f.\n",forward_right_value);
    printf("- Right: %.2f.\n",right_value);
    
    /*
    * Enter here functions to send actuator commands.
    */
    double left_speed, right_speed;
    left_speed = 0;
    right_speed = 0;
    
    printf("Mode: %d.\n", mode);
    printf("Action: ");
    if(mode == 0) { // Mode 0: Move forward
    
      printf("Moving forward.\n");
    
      left_speed = MAX_SPEED;
      right_speed = MAX_SPEED;
      
      // When sufficiently close to a wall in front of robot, switch mode to wall following
      if ((forward_right_value > 500) || (forward_left_value > 500)) {
        mode = 1;
      }
      
    } 
    else { // Mode 1: Wall following
        
      if ((forward_right_value > 200) || (forward_left_value > 200)) {
        printf("Backing up and turning left.\n"); 
        
        left_speed = - MAX_SPEED / 2.0;
        right_speed = MAX_SPEED / 4.0;
      }
      else {
      
        if (right_value > 300) {
          printf("Turning left away from wall.\n"); 
          
          left_speed = MAX_SPEED / 1.75;
          right_speed = MAX_SPEED;
        } 
        else {
        
          if (right_value < 200) {
            printf("Turning right towards wall.\n");
              
            left_speed = MAX_SPEED;
            right_speed = MAX_SPEED / 1.75;
          } 
          else {
            printf("Moving forward along wall.\n");
            
            left_speed = MAX_SPEED;
            right_speed = MAX_SPEED;
          } 
          
        }
        
      }
      
    }
     
    /*Set the motor speeds.*/
    wb_motor_set_velocity(left_wheel, left_speed);
    wb_motor_set_velocity(right_wheel, right_speed);
    
    /* 
     * Perform a simulation step of 64 milliseconds
     * and leave the loop when the simulation is over
     */
  } while (wb_robot_step(TIME_STEP) != -1);

  
  /* Necessary to cleanup webots stuff */
  wb_robot_cleanup();
  
  return 0;
}