#include <boost/asio.hpp>
#include <iostream>
#include <pixhawk/mavlink.h>
#include "ros/ros.h"
#include <marvel_v_0_1/Autopilot.h>
#include <marvel_v_0_1/Guidance_Command.h>
//
// Initializing boost
//
using namespace boost;
asio::io_service ios;
asio::serial_port port(ios);
//
// Message initialization
//
marvel_v_0_1::Autopilot autopilot_msg;
marvel_v_0_1::Guidance_Command guidance_msg;
//
// Mavlink message receive and handle function
//
void msg_receive(uint8_t c) {
    mavlink_message_t msg;
    mavlink_status_t status;
    //
    // Try to get a new message
    //
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
    {   //
        // Handle message
        //
        switch(msg.msgid)
        {   case MAVLINK_MSG_ID_HEARTBEAT:

            break;
            case MAVLINK_MSG_ID_DEBUG_VECT:


            break;
        }
    }
}
//
// Mavlink radio message send function
//
void msg_send_radio(float throttle, float roll, float pitch, float yaw) {

}
//
// Mavlink arm message send function
//
void msg_send_arm() {

}
//
// Mavlink disarm message send function
//
void msg_send_disarm() {

}
//
// Mavlink mode change message send function
//
void msg_send_mode_change(int mode) {

}
//
// Guidance message callback function
//
void guidance_msg_Callback(const marvel_v_0_1::Guidance_Command::ConstPtr& msg) {

}

//
// Main program
//
int main(int argc, char **argv) {
    //
    // Port configuration
    //
    port.open("/dev/ttyACM0");
    port.set_option(asio::serial_port_base::baud_rate(115200));
    //
    // Ros configuration
    //
    ros::init(argc, argv, "server");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<marvel_v_0_1::Autopilot>("server", 1000);
    ros::Subscriber sub = n.subscribe("guidance_pack", 1000, guidance_msg_Callback);
    //
    // Main loop
    //
    char r_byte;
    while(1) {
        asio::read(port, asio::buffer(&r_byte,1));
        msg_receive(uint8_t(r_byte));

        ros::spinOnce();
        pub.publish(autopilot_msg);
    }
    return 0;
}
