#include <boost/asio.hpp>
#include <iostream>
#include <pixhawk/mavlink.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <marvel_v_0_1/OpticalFlow.h>
#include <time.h>
#include <cmath>
//
// Initializing boost
//
using namespace boost;
asio::io_service ios;
asio::serial_port port(ios);
//
// Message initialization
//
marvel_v_0_1::OpticalFlow optFlowMsg;
mavlink_optical_flow_t flow;
mavlink_debug_vect_t debug;
//
// Global variables
//
float x = 0, y = 0;
float last_x = 0, last_y = 0;
float xdot = 0, ydot = 0, zdot = 0;
float gyro_x = 0, gyro_y = 0;

double last_flow_time = 0;
double last_gyro_time = 0;
//
// Mavlink message receive and handle function
//
void msg_resolve(uint8_t c) {
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
        {   case MAVLINK_MSG_ID_OPTICAL_FLOW:
                mavlink_msg_optical_flow_decode(&msg, &flow);

                if((last_flow_time != 0) && ((flow.time_usec - last_flow_time) > 0)) {
                    if(abs(debug.y) < 0.5) {
                        x += flow.flow_comp_m_x * (flow.time_usec - last_flow_time) / 1000000;
                        optFlowMsg.velocity_x = flow.flow_comp_m_x;
                    }
                    if(abs(debug.x) < 0.5) {
                        y += flow.flow_comp_m_y * (flow.time_usec - last_flow_time) / 1000000;
                        optFlowMsg.velocity_y = flow.flow_comp_m_y;
                    }
                }

                last_flow_time = flow.time_usec;

                optFlowMsg.ground_distance = flow.ground_distance;
                std::cout << std::abs(debug.x) << " ** " << y <<std::endl;
                optFlowMsg.quality = flow.quality;
            break;
            case MAVLINK_MSG_ID_DEBUG_VECT:
                mavlink_msg_debug_vect_decode(&msg, &debug);

                if((last_gyro_time != 0) && ((debug.time_usec - last_gyro_time) > 0)) {
                    gyro_x += debug.x * (debug.time_usec - last_gyro_time) / 1000000;
                    gyro_y += debug.y * (debug.time_usec - last_gyro_time) / 1000000;
                    //x += -1 * tan(gyro_y) * flow.ground_distance / 4;
                    //y += tan(gyro_x) * flow.ground_distance / 4;
                }

                last_gyro_time = debug.time_usec;

            break;
        }
        optFlowMsg.flow_x = x;
        optFlowMsg.flow_y = y;
    }
}
//
// Main program
//
int main(int argc, char **argv)
{   //
    // Port configuration
    //
    port.open("/dev/ttyACM0");
    port.set_option(asio::serial_port_base::baud_rate(115200));
    //
    // Ros configuration
    //
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<marvel_v_0_1::OpticalFlow>("optical_flow", 1000);
    //
    // Main loop
    //
    char next_byte;
    while (ros::ok()) {
        asio::read(port, asio::buffer(&next_byte,1));
        msg_resolve(uint8_t(next_byte));
        ros::spinOnce();
        chatter_pub.publish(optFlowMsg);
    }

    port.close();
    return 0;
}

