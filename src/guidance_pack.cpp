#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <../include/flight_plan.h>
#include <../include/guidance_pack.h>
#include <../include/pid.h>
#include <ros/ros.h>
#include <marvel_v_0_1/OpticalFlow.h>
//
// Guidance variables
//
vertical_mode g_vertical_mode;
horizontal_mode g_horizontal_mode;
heading_mode g_heading_mode;
float v_x_setpoint = 0, v_y_setpoint = 0, v_z_setpoint = 0;
float heading_setpoint = 0;
float height_setpoint = 0;
float heading_lock_param;
float horizontal_lock_param;
float vertical_lock_param;
float move_oa_break_cond;
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
                if (type == "take_off") f[current_function_no].type = take_off;
                if (type == "hold_position") f[current_function_no].type = hold_position;
                if (type == "set_heading") f[current_function_no].type = set_heading;
                if (type == "heading_lock") f[current_function_no].type = heading_lock;
                if (type == "heading_unlock") f[current_function_no].type = heading_unlock;
                if (type == "rotate") f[current_function_no].type = rotate;
                if (type == "move_oa") f[current_function_no].type = move_oa;
                if (type == "move") f[current_function_no].type = move;
                if (type == "go") f[current_function_no].type = go;
                if (type == "go_oa") f[current_function_no].type = go_oa;

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
    if (msg->velocity_x != 0) {
        std::cout << msg->velocity_x << std::endl;
    }
}
//
// Main program start
//
int main(int argc, char **argv) {
    //
    // Ros initialization
    //
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("optical_flow", 1000, optical_flow_Callback);
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
    // Guidance loop
    //
    while(1) {
        //
        // Handle function characteristics
        //
        if(f[current_function_no].done == true) current_function_no ++;
        if (f[current_function_no].type = take_off) {

            g_vertical_mode = VERTICAL_CLIMB;
            g_horizontal_mode = HORIZONTAL_HOLD;
            g_heading_mode = HEADING_HOLD;
        }
        if (f[current_function_no].type = hold_position) {

            g_vertical_mode = VERTICAL_HOLD;
            g_horizontal_mode = HORIZONTAL_HOLD;
            g_heading_mode = HEADING_HOLD;
        }
        if (f[current_function_no].type = set_heading) {

            g_vertical_mode = VERTICAL_IDLE;
            g_horizontal_mode = HORIZONTAL_IDLE;
            g_heading_mode = HEADING_RATE;
        }

        if (f[current_function_no].type = heading_lock) {

            g_vertical_mode = VERTICAL_IDLE;
            g_horizontal_mode = HORIZONTAL_IDLE;
            g_heading_mode = HEADING_LOCK;
        }
        if (f[current_function_no].type = heading_unlock) {

            g_vertical_mode = VERTICAL_IDLE;
            g_horizontal_mode = HORIZONTAL_IDLE;
            g_heading_mode = HEADING_IDLE;
        }
        if (f[current_function_no].type = rotate) {

            g_vertical_mode = VERTICAL_IDLE;
            g_horizontal_mode = HORIZONTAL_IDLE;
            g_heading_mode = HEADING_RATE;
        }
        if (f[current_function_no].type = move_oa) {

            g_vertical_mode = VERTICAL_HOLD;
            g_horizontal_mode = HORIZONTAL_VELOCITY;
            g_heading_mode = HEADING_IDLE;
        }
        if (f[current_function_no].type = move) {

            g_vertical_mode = VERTICAL_HOLD;
            g_horizontal_mode = HORIZONTAL_VELOCITY;
            g_heading_mode = HEADING_IDLE;
        }
        if (f[current_function_no].type = go) {

            g_vertical_mode = VERTICAL_LOCK;
            g_horizontal_mode = HORIZONTAL_LOCK;
            g_heading_mode = HEADING_IDLE;
        }
        if (f[current_function_no].type = go_oa) {

            g_vertical_mode = VERTICAL_LOCK;
            g_horizontal_mode = HORIZONTAL_LOCK;
            g_heading_mode = HEADING_IDLE;
        }

        ros::spinOnce();
    }

    return 0;
}

