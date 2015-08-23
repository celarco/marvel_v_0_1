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
//
// Guidance variables
//
vertical_mode g_vertical_mode;
horizontal_mode g_horizontal_mode;
heading_mode g_heading_mode;

int quality;

float v_x_setpoint = 0, v_y_setpoint = 0, v_z_setpoint = 0;
float v_x, v_y, v_z;

float heading_setpoint = 0;
float heading;

float height_setpoint = 0;
float height;

float rate_setpoint = 0;
float rate;

int heading_lock_param;
int horizontal_lock_param;
int vertical_lock_param;

float rotate_break_cond;
float move_oa_break_cond;

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
        std::cout << "Blocks are :" << std::endl;
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
        if(f[current_function_no].done == true) current_function_no ++;

        switch(f[current_function_no].type) {

        case take_off:
            g_vertical_mode = VERTICAL_CLIMB;
            g_horizontal_mode = HORIZONTAL_HOLD;
            g_heading_mode = HEADING_HOLD;
            v_z_setpoint = f[current_function_no].arg[0];
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
            heading_lock_param = int(f[current_function_no].arg[0]);
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
            horizontal_lock_param = int(f[current_function_no].arg[0]);
            vertical_lock_param = int(f[current_function_no].arg[1]);
        break;

        case go_oa:
            g_vertical_mode = VERTICAL_LOCK;
            g_horizontal_mode = HORIZONTAL_LOCK;
            horizontal_lock_param = int(f[current_function_no].arg[0]);
            vertical_lock_param = int(f[current_function_no].arg[1]);
        break;
        }
		//
		// Vertical mode pd
		//
        switch(g_vertical_mode) {
        case VERTICAL_HOLD:

        break;

        case VERTICAL_LOCK:


        break;
        case VERTICAL_CLIMB:

        break;
        }
		//
		// Horizontal pd
		//
		switch(g_horizontal_mode) {
		case HORIZONTAL_HOLD:
		
		break;
		case HORIZONTAL_LOCK:
		
		break;
		case HORIZONTAL_CLIMB:
		
		break;
		case HORIZONTAL_VELOCITY:
		
		break;
		}
		//
		// Heading pd
		//
		switch(g_heading_mode) {
		case HEADING_HOLD:
		
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

