#include <boost/asio.hpp>
#include <iostream>
#include <pixhawk/mavlink.h>
#include "ros/ros.h"
#include <marvel_v_0_1/Autopilot.h>
#include <marvel_v_0_1/Guidance_Command.h>
#include <radio.h>
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
// Global variables
//
bool last_arm_status = 0;
bool current_arm_status = 0;
int roll = 0, pitch = 0, yaw = 0, throttle = 0;
int mode = 0;
radio rc;
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
            case MAVLINK_MSG_ID_PARAM_VALUE:
				mavlink_param_value_t param_value_msg;
				mavlink_msg_param_value_decode(&msg, &param_value_msg);
				switch(int(param_value_msg.param_index)) {
					//
					// Extracting radio roll parameters
					//
					case 70:
					rc.roll.min = int(mavlink_msg_param_value_get_param_value(&msg));
					break;
					
					case 71:
					rc.roll.trim = int(mavlink_msg_param_value_get_param_value(&msg));
					break;
					
					case 72:
					rc.roll.max = int(mavlink_msg_param_value_get_param_value(&msg));
					break;
					
					case 73:
					rc.roll.rev = int(mavlink_msg_param_value_get_param_value(&msg));
					break;
					
					case 74:
					rc.roll.dz = int(mavlink_msg_param_value_get_param_value(&msg));
					break;
					//
					// Extracting radio pitch parameters
					//
					case 75:
					rc.pitch.min = int(mavlink_msg_param_value_get_param_value(&msg));
					break;
					
					case 76:
					rc.pitch.trim = int(mavlink_msg_param_value_get_param_value(&msg));
					break;
					
					case 77:
					rc.pitch.max = int(mavlink_msg_param_value_get_param_value(&msg));
					break;
					
					case 78:
					rc.pitch.rev = int(mavlink_msg_param_value_get_param_value(&msg));
					break;
					
					case 79:
					rc.pitch.dz = int(mavlink_msg_param_value_get_param_value(&msg));
					break;
					//
					// Extracting radio throttle parameters
					//
					case 80:
					rc.throttle.min = int(mavlink_msg_param_value_get_param_value(&msg));
					break;
					
					case 81:
					rc.throttle.trim = int(mavlink_msg_param_value_get_param_value(&msg));
					break;
					
					case 82:
					rc.throttle.max = int(mavlink_msg_param_value_get_param_value(&msg));
					break;
					
					case 83:
					rc.throttle.rev = int(mavlink_msg_param_value_get_param_value(&msg));
					break;
					
					case 84:
					rc.throttle.dz = int(mavlink_msg_param_value_get_param_value(&msg));
					break;
					//
					// Extracting radio yaw parameters
					//
					case 85:
					rc.yaw.min = int(mavlink_msg_param_value_get_param_value(&msg));
					break;
					
					case 86:
					rc.yaw.trim = int(mavlink_msg_param_value_get_param_value(&msg));
					break;
					
					case 87:
					rc.yaw.max = int(mavlink_msg_param_value_get_param_value(&msg));
					break;
					
					case 88:
					rc.yaw.rev = int(mavlink_msg_param_value_get_param_value(&msg));
					break;
					
					case 89:
					rc.yaw.dz = int(mavlink_msg_param_value_get_param_value(&msg));
					break;
					//
					// Extracting radio mode parameters
					//
					case 90:
					rc.mode.min = int(mavlink_msg_param_value_get_param_value(&msg));
					break;
					
					case 91:
					rc.mode.trim = int(mavlink_msg_param_value_get_param_value(&msg));
					break;
					
					case 92:
					rc.mode.max = int(mavlink_msg_param_value_get_param_value(&msg));
					break;
					
					case 93:
					rc.mode.rev = int(mavlink_msg_param_value_get_param_value(&msg));
					break;
					
					case 4:
					rc.mode.dz = int(mavlink_msg_param_value_get_param_value(&msg));
					break;
				}
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
	//
	// Command initializtion
	//
	mavlink_command_long_t arm_command_msg;
	arm_command_msg.command = MAV_CMD_COMPONENT_ARM_DISARM;
    arm_command_msg.target_system = 1;
    arm_command_msg.target_component = MAV_COMP_ID_SYSTEM_CONTROL;
    arm_command_msg.confirmation = 0;
    arm_command_msg.param1 = 1;
    arm_command_msg.param2 = 0;
    arm_command_msg.param3 = 0;
    arm_command_msg.param4 = 0;
    arm_command_msg.param5 = 0;
    arm_command_msg.param6 = 0;
    arm_command_msg.param7 = 0;
	//
	// Message pack and send
	// 
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	mavlink_msg_command_long_encode(1, 255, &msg, &arm_command_msg);
	unsigned len = mavlink_msg_to_send_buffer(buf, &msg);
	asio::write(port, asio::buffer(buf,len));
}
//
// Mavlink disarm message send function
//
void msg_send_disarm() {
	//
	// Command initializtion
	//
	mavlink_command_long_t arm_command_msg;
	arm_command_msg.command = MAV_CMD_COMPONENT_ARM_DISARM;
    arm_command_msg.target_system = 1;
    arm_command_msg.target_component = MAV_COMP_ID_SYSTEM_CONTROL;
    arm_command_msg.confirmation = 0;
    arm_command_msg.param1 = 0;
    arm_command_msg.param2 = 0;
    arm_command_msg.param3 = 0;
    arm_command_msg.param4 = 0;
    arm_command_msg.param5 = 0;
    arm_command_msg.param6 = 0;
    arm_command_msg.param7 = 0;
	//
	// Message pack and send
	//
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];	
	mavlink_msg_command_long_encode(1, 255, &msg, &arm_command_msg);
	unsigned len = mavlink_msg_to_send_buffer(buf, &msg);
	asio::write(port, asio::buffer(buf,len));
}
//
// Mavlink heartbeat message send function
//
void msg_send_heartbeat() {
	
}
//
// Mavlink request parameter message send fucntion
//
void msg_send_request_param() {
	//
	// Command initializtion
	//
	mavlink_param_request_list_t request_param__list_msg;
	request_param__list_msg.target_system = 1;
	request_param__list_msg.target_component = 255;	 
	//
	// Message pack and send
	//
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];	
	mavlink_msg_param_request_list_encode(1, 255, &msg, &request_param__list_msg);
	unsigned len = mavlink_msg_to_send_buffer(buf, &msg);
	asio::write(port, asio::buffer(buf,len));	
}
//
// Guidance message callback function
//
void guidance_msg_Callback(const marvel_v_0_1::Guidance_Command::ConstPtr& msg) {
	current_arm_status = msg->arm;
	throttle = rc.calc_throttle(int(msg->throttle));
	roll = rc.calc_roll(int(msg->roll));
	pitch = rc.calc_pitch(int(msg->pitch));
	yaw = rc.calc_yaw(int(msg->yaw));
	mode = rc.calc_mode(int(msg->mode));
	//std::cout << " " << roll << " " << pitch << " " << throttle << " " << yaw << " " << mode <<std::endl;
}
//
// Main program
//
int main(int argc, char **argv) {
    //
    // Port configuration
    //
    port.open("/dev/ttySAC0");
    port.set_option(asio::serial_port_base::baud_rate(57600));
    //
    // Ros configuration
    //
    ros::init(argc, argv, "server");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<marvel_v_0_1::Autopilot>("server", 1000);
    ros::Subscriber sub = n.subscribe("guidance_pack", 1000, guidance_msg_Callback);
    //
	// Read autopilot parameters
	//
	msg_send_request_param();
	//
    // Main loop
    //
	char r_byte;
    while(1) {
        asio::read(port, asio::buffer(&r_byte,1));
        msg_receive(uint8_t(r_byte));
		
	 	if((current_arm_status == 1)&&(last_arm_status == 0)) {
			last_arm_status = 1;
			msg_send_arm();
		}
		if((current_arm_status == 0)&&(last_arm_status == 1)) {
			last_arm_status = 0;
			msg_send_disarm();
		} 
		
        ros::spinOnce();
        pub.publish(autopilot_msg);
    }
    return 0;
}
