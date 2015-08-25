#include <stdlib.h>
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <../include/flight_plan.h>
#include <../include/guidance_pack.h>
#include <../include/pid.h>
#include <ros/ros.h>
#include <marvel_v_0_1/OpticalFlow.h>
#include <marvel_v_0_1/Autopilot.h>
#include <marvel_v_0_1/Guidance_Command.h>
#include <marvel_v_0_1/window_detector.h>
#include <marvel_v_0_1/obstacle_avoidance.h>
//
// Guidance variables
//
vertical_mode g_vertical_mode;
horizontal_mode g_horizontal_mode;
heading_mode g_heading_mode;
lock_param g_lock_param;

int quality;

float v_x_setpoint = 0, v_y_setpoint = 0, v_z_setpoint = 0;
float v_x, v_y, v_z;

float heading_setpoint = 0;
float heading;

float height_setpoint = 0;
float height;

float rate_setpoint = 0;
float rate;

float rotate_break_cond;
float move_oa_break_cond;

float window_deltax,window_deltay,window_pitch;

float obstacle_vx_setpoint,obstacle_vy_setpoint,obstacle_vz_setpoint,obstacle_psi_setpoint;
bool armed;

pid pid_x, pid_y, pid_z, pid_h;
//
// Flight plan variables
//
function f[MAX_FUNCTION_COUNT];
block b[MAX_BLOCK_COUNT];
unsigned short int function_count = 0;
unsigned short int block_count = 0;
unsigned short int current_function_no = 0;
//
// Flight plan initialization function
//
bool initialize_flight_plan() {
    std::string line;
    std::ifstream file ("var/flight_plan.txt");
    //
    // Reading blocks and functions from the file
    //
    if (file.is_open()) {

        while (getline(file,line)) {

            std::stringstream stream(line);
            std::string c;
            stream >> c;
            if (c == "block") {

                unsigned short int n;
                stream >> n;
                b[block_count].id = n;
                std::string name;
                stream >> name;
                b[block_count].name = name;
                b[block_count].start_function_number = function_count;
                block_count ++;
            }
            else if (c == "function") {

                std::string type;
                stream >> type;
                for(int i = 0; i < MAX_FUNCTION_ARG_COUNT; i++) {

                    float arg;
                    stream >> arg;
                    f[function_count].arg[i] = arg;
                }
                if (type == "take_off") f[function_count].type = take_off;
                if (type == "hold_position") f[function_count].type = hold_position;
                if (type == "set_heading") f[function_count].type = set_heading;
                if (type == "heading_lock") f[function_count].type = heading_lock;
                if (type == "heading_unlock") f[function_count].type = heading_unlock;
                if (type == "rotate") f[function_count].type = rotate;
                if (type == "move_oa") f[function_count].type = move_oa;
                if (type == "move") f[function_count].type = move;
                if (type == "go") f[function_count].type = go;
                if (type == "go_oa") f[function_count].type = go_oa;

                function_count ++;
            }
        }
        //
        // Printing functions and blocks to verify
        //
        std::cout << "Blockpiths are :" << std::endl;
        for(int i = 0; i < block_count; i++) {

            std::cout << "Block No. " << b[i].id << " : " << b[i].name <<" with start function No. " << b[i].start_function_number << std::endl;
        }
        std::cout << "Functions are :" << std::endl;
        for(int i = 0; i < function_count; i++) {

            std::cout << "Function No. " << i << " of type: " << f[i].type << std::endl;
        }
        file.close();
        return true;
    }

    else {
        return false;
    }

}
//
// Optical flow callback function
//
void optical_flow_Callback(const marvel_v_0_1::OpticalFlow::ConstPtr& msg) {
	
	v_x = msg->velocity_x;
	v_y = msg->velocity_y;
	height = msg->ground_distance;
	quality = msg->quality;
}
//
// Server receive callback function
//
void server_receive_Callback(const marvel_v_0_1::Autopilot::ConstPtr& msg) {
	heading = msg->heading;
	rate = msg->rate;
	v_z = msg->climb;
	armed = msg->armed;
}
//
// Window position data receive callback function
//
void window_postion_receive_Callback(const marvel_v_0_1::window_detector::ConstPtr& msg) {
	window_deltax = msg->deltax;
	window_deltay = msg->deltay;
	window_pitch = msg->pith;
}
//
// Obstacle Avoidance data receive callback function
//
void window_postion_receive_Callback(const marvel_v_0_1::obstacle_avoidance::ConstPtr& msg) {
	obstacle_vx_setpoint = msg->vx;
	obstacle_vy_setpoint = msg->vy;
	obstacle_vz_setpoint = msg->vz;
	obstacle_psi_setpoint = msg->psidot
}
//
// Main program start
//
int main(int argc, char **argv) {

    printf( " ********************************************** \n" );
    printf( " ********************************************** \n" );
    printf( " *                                            * \n" );
    printf( " *                                            * \n" );
    printf( " *                                            * \n" );
    printf( " *                                            * \n" );
    printf( " *                                            * \n" );
    printf( " *                                            * \n" );
    printf( " *                                            * \n" );
    printf( " *     This program debug and Develop by      * \n" );
    printf( " *          Mohammad Hossein Kazemi           * \n" );
    printf( " *               Ali Jameie                   * \n" );
    printf( " *        All Right reserved 2015-2016        * \n" );
    printf( " *      Email:Mhkazemi_engineer@yahoo.com     * \n" );
    printf( " *        Email:Celarco.Group@Gmail.com       * \n" );
    printf( " *     AmirKabir University of Technology     * \n" );
    printf( " *   AUT-MAV AUTONOMOUS AIRIAL VEHICLE TEAM   * \n" );
    printf( " *                                            * \n" );
    printf( " *                                            * \n" );
    printf( " *                                            * \n" );
    printf( " *                                            * \n" );
    printf( " ********************************************** \n" );
    printf( " ********************************************** \n" );

    //
    // Ros initialization
    //
    ros::init(argc, argv, "guidance_pack");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("optical_flow", 1000, optical_flow_Callback);
    ros::Subscriber sub2 = n.subscribe("server", 1000, server_receive_Callback);
	ros::Subscriber sub3 = n.subscribe("window_postion", 1000, window_postion_receive_Callback);
	ros::Subscriber sub4 = n.subscribe("obstacle_data", 1000, obstacle_avoidance_receive_Callback);
    ros::Publisher pub = n.advertise<marvel_v_0_1::Guidance_Command>("guidance_pack", 1000);
    marvel_v_0_1::Guidance_Command guidance_msg;
    
	//
    // Flight plan initialization
    //
	
    if(!(initialize_flight_plan())) {

        std::cout<<"Error: Flight plan couldn't be initialized"<<std::endl;
        return 0;
    }
    else {
        std::cout<<"Flight plan successfully initialized...!"<<std::endl;
    }
	//
	// Arming the quadcopter
	//
	guidance_msg.arm = 1;
	//
    // Guidance loop
    //
    while(1) {
        //
        // Handle function characteristics
        //
        if(f[current_function_no].done == true){
			current_function_no ++;
			system("rosservice call /qr_kill 1 1 ");
			system("rosservice call /window_kill 1 1");
		} 

        switch(f[current_function_no].type) {

        case take_off:
            g_vertical_mode = VERTICAL_CLIMB;
            g_horizontal_mode = HORIZONTAL_HOLD;
            g_heading_mode = HEADING_HOLD;
            v_z_setpoint = f[current_function_no].arg[0];
			height_setpoint = f[current_function_no].arg[1];
			if(height >= height_setpoint) {
				f[current_function_no].done = true;
				g_vertical_mode = VERTICAL_HOLD;
			}
				
        break;

        case hold_position:
            g_vertical_mode = VERTICAL_HOLD;
            g_horizontal_mode = HORIZONTAL_HOLD;
            g_heading_mode = HEADING_HOLD;
		break;

        case set_heading:
            g_heading_mode = HEADING_RATE;
            heading_setpoint = f[current_function_no].arg[0];
        break;

        case heading_lock:
            g_heading_mode = HEADING_LOCK;
            
        break;

        case heading_unlock:
            g_heading_mode = HEADING_HOLD;
        break;

        case rotate:
            g_heading_mode = HEADING_RATE;
            rate_setpoint = f[current_function_no].arg[0];
            rotate_break_cond = int(f[current_function_no].arg[1]);
        break;

        case move_oa:
            g_vertical_mode = VERTICAL_HOLD;
            g_horizontal_mode = HORIZONTAL_VELOCITY;
            v_x_setpoint = f[current_function_no].arg[0];
            v_y_setpoint = f[current_function_no].arg[1];
            move_oa_break_cond = int(f[current_function_no].arg[2]);
            height_setpoint = f[current_function_no].arg[3];
        break;

        case move:
            g_vertical_mode = VERTICAL_HOLD;
            g_horizontal_mode = HORIZONTAL_VELOCITY;
            v_x_setpoint = f[current_function_no].arg[0];
            v_y_setpoint = f[current_function_no].arg[1];
            move_oa_break_cond = int(f[current_function_no].arg[2]);
            height_setpoint = f[current_function_no].arg[3];
        break;

        case go:
            g_vertical_mode = VERTICAL_LOCK;
            g_horizontal_mode = HORIZONTAL_LOCK;
          
        break;

        case go_oa:
		
            g_vertical_mode = VERTICAL_LOCK;
            g_horizontal_mode = HORIZONTAL_LOCK;
            g_lock_param = lock_param(f[current_function_no].arg[0]);
			
			switch (g_lock_param){
			 
			case WINDOW_DETECTOR:
				system("roslaunch openni2_launch openni2.launch  &> /dev/null &");
				system("rosrun marvel_v_0_1 window_detector  &> /dev/null &");
				system("rosrun marvel_v_0_1 obstacle_avoidance  &> /dev/null &");
				guidance_msg.vx_uni=window_deltax*VELOCITY_FAKE_COEFF;
				guidance_msg.vy_uni=window_deltay*VELOCITY_FAKE_COEFF;
				guidance_msg.vz_uni=window_deltaz*VELOCITY_FAKE_COEFF;
				
				v_x_setpoint = obstacle_vx_setpoint;
				v_y_setpoint = obstacle_vy_setpoint;
				v_z_setpoint = obstacle_vz_setpoint;
				
			break;
				 
			case FLOWER:
			break;
				 
			case MARKER:
			break;
				 
			case ROPE:
			break; 
				 
			}
		break;
		
        }
		//
		// Vertical mode pd
		//
        switch(g_vertical_mode) {
        case VERTICAL_HOLD:
			pid_z.set_coeff(0.4,0);
			guidance_msg.throttle = 100 * pid_z.loop_once((height_setpoint - height),0) + 50;
        break;

        case VERTICAL_LOCK:


        break;
        case VERTICAL_CLIMB:
			pid_z.set_coeff(0.4,0);
			guidance_msg.throttle = 100 * pid_z.loop_once((v_z_setpoint - v_z),0) + 50; 
		break;
        }
		//
		// Horizontal pd
		//
		switch(g_horizontal_mode) {
		case HORIZONTAL_HOLD:
			pid_x.set_coeff(1,0);
			if(((0 - v_x) >= 0.01) || ((0 - v_x) <= -0.01)) {
				guidance_msg.roll = 100 * pid_x.loop_once((0 - v_x),0);
			}
			else if (((0 - v_x) <= 0.01) || ((0 - v_x) >= -0.01)) {
				guidance_msg.roll = 0;
			}
			
			pid_y.set_coeff(1,0);
			if(((0 - v_y) >= 0.01) || ((0 - v_y) <= -0.01)) {
				guidance_msg.pitch = 100 * pid_y.loop_once((0 - v_y),0);
			}
			else if (((0 - v_y) <= 0.01) || ((0 - v_y) >= -0.01)) {
				guidance_msg.pitch = 0;
			}
		break;
		case HORIZONTAL_LOCK:
		
		break;
		case HORIZONTAL_VELOCITY:
		
		break;
		}
		//
		// Heading pd
		//
		switch(g_heading_mode) {
		case HEADING_HOLD:
			pid_h.set_coeff(0.007,0);
			guidance_msg.yaw = 100 * pid_h.loop_once((heading_setpoint-heading),0);
		break;
		case HEADING_LOCK:
			
		break;
		case HEADING_RATE:
		
		break;
		}

        ros::spinOnce();
        pub.publish(guidance_msg);
    }

    return 0;
}

