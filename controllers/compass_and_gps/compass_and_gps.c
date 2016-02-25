/*
 * File:          compass_and_gps.c
 * Date:          15/02/2016
 * Description:   A simple controller demonstrating the compass and the gps sensor. Drives along desired angle indicated.
 * Author:        Martin F. Stoelen
 */


/*
 * You may need to add include files like <webots/distance_sensor.h> etc.
 */
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/compass.h>
#include <webots/gps.h>
#include <stdio.h>
#include <math.h>


/*
 * You may want to add defines macro here.
 */
#define TIME_STEP 64
#define PI 3.14159265
#define MAX_SPEED 5.24
#define DESIRED_ANGLE 30.0


/*
 * An example for converting bearing vector from compass to angle (in degrees).
 */
double convert_bearing_to_degrees(const double* in_vector) {
  double rad = atan2(in_vector[0], in_vector[2]);
  double deg = rad * (180.0 / PI);
  return deg;
}


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

  WbDeviceTag compass = wb_robot_get_device("compass");
  wb_compass_enable(compass, TIME_STEP);
  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, TIME_STEP);

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
    const double *compass_val = wb_compass_get_values(compass);
    const double *gps_val = wb_gps_get_values(gps);

    /*
     * Convert compass bearing vector to angle, in degrees.
     * Print GPS values.
    */
    double compass_angle = convert_bearing_to_degrees(compass_val);
    printf("Compass angle (degrees): %.2f.\n", compass_angle);
    printf("GPS values (x,z): %.2f, %.2f.\n", gps_val[0], gps_val[2]);

    /*
    * Enter here functions to send actuator commands.
    */
    double left_speed, right_speed;

    if(compass_angle < (DESIRED_ANGLE - 1.0)) {
      // Turn right
      left_speed = MAX_SPEED;
      right_speed = MAX_SPEED / 1.5;
    }
    else {
      if(compass_angle > (DESIRED_ANGLE + 1.0)) {
        // Turn left
        left_speed = MAX_SPEED / 1.5;
        right_speed = MAX_SPEED;
      }
      else {
        // Move straight forward
        left_speed = MAX_SPEED;
        right_speed = MAX_SPEED;
      }
    }

    /*Set the motor speeds.*/
    wb_motor_set_velocity(left_wheel, left_speed);
    wb_motor_set_velocity(right_wheel, right_speed);

  }

  /* Necessary to cleanup webots stuff */
  wb_robot_cleanup();

  return 0;
}
