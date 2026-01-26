// basic library functionality
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
// socket communication functions and constants
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/sockios.h>

// additional standard library
#include <iostream>
#include <queue>
#include <chrono>
#include <memory>
#include <cmath>
// ros2 related imports
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <sensor_msgs/msg/magnetic_field.hpp>


extern "C" {
#include "ICM20948.h"
}

class IcmWrapper : public rclcpp::Node{
  public:
    //
    // FUNCTIONS
    //

    // constructor
    // creates the datastructures to store data and work as cross thread communication.
    // creates publishers for joystick and sasa sensor outputs
    IcmWrapper();

    // destructor
    ~IcmWrapper();

  private:
    //
    // VARIABLES
    //

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub;

    std::thread i2c_listener_thread_;


    //
    // FUNCTIONS
    //

    /*
    callback function for the CAN reading thread. Waits untill a message arrives to the previously intialized socket <s_> and then 
    starts the data parsing operations. Parsed data is read as is using the dbc file generated decoding functions and published over ros topics
    (oublishing as of writing (03.07.24) works on timers)
    The real parsing is done using the <can_parsing> function, see "CAN1_utils" for documentation

    inputs:
        None
                                            
    return:
        None, all modification operations are done using the pointer inputs. 

    aftermath:
        The contents of the received <frame> are parsed and dealt with in the manner dictated by the CAN ID
    */
    void i2c_read();
};