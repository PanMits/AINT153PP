/*
 * File:          kinect.c
 * Date:          17/02/2016
 * Description:   A controller for showcasing the kinect, with another tail-chasing robot!
 * Author:        Martin F. Stoelen
 */
 

/*
 * You may need to add include files like <webots/distance_sensor.h> etc.
 */
#include <webots/robot.h>
#include <webots/motor.h> 
#include <webots/camera.h>
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
  
  WbDeviceTag kinect_colour = wb_robot_get_device("kinectColor");
  wb_camera_enable(kinect_colour, TIME_STEP);
  
  
  /*
  * Get image height and width, assumed same for depth and colour images
  */
  int image_width, image_height;
  image_width = wb_camera_get_width(kinect_colour);
  image_height = wb_camera_get_height(kinect_colour);
  
  
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
    const unsigned char* colour_image;
    colour_image = wb_camera_get_image(kinect_colour);
    
    
    /* 
    * Count number of pixels that are white(ish).
    * E.g. have pixel value > 200 out of 255 for all scolor components.
    */
    int sum = 0;
    int x, y;
    for(x=0; x<image_width; x++) {
      for(y=0; y<image_height; y++) {
        int green = wb_camera_image_get_green(colour_image, image_width, x, y);
        int red = wb_camera_image_get_red(colour_image, image_width, x, y);
        int blue = wb_camera_image_get_blue(colour_image, image_width, x, y);
        
        if((green > 200) && (red > 200) && (blue > 200)) {
          sum = sum + 1;
        }
      }
    }
    
    double percentage_white = (sum/ (float) (image_width*image_height))*100;
    printf("Percentage of white in kinect colour image: %.1f.\n", percentage_white);
    
    
    /*
    * Enter here functions to send actuator commands.
    */
    double left_speed, right_speed;
    left_speed = 3.0;
    right_speed = 1.5;
    
    /*Set the motor speeds.*/
    wb_motor_set_velocity(left_wheel, left_speed);
    wb_motor_set_velocity(right_wheel, right_speed);
    
    
  }
  
  /* Necessary to cleanup webots stuff */
  wb_robot_cleanup();
  
  return 0;
}