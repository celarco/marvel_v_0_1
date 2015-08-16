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
unsigned short int g_vertical_mode, g_horizontal_mode, g_heading_mode;
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
                if (type == "take_off") {
                    f[function_count].type = take_off;
                    f[function_count].v_mode = VERTICAL_CLIMB;
                    f[function_count].h_mode = HORIZONTAL_HOLD;
                    f[function_count].head_mode = HEADING_HOLD;
                }
                if (type == "hold_position") {
                    f[function_count].type = hold_position;
                    f[function_count].v_mode = VERTICAL_HOLD;
                    f[function_count].h_mode = HORIZONTAL_HOLD;
                    f[function_count].head_mode = HEADING_HOLD;
                }
                if (type == "set_heading") {
                    f[function_count].type = set_heading;
                    f[function_count].v_mode = VERTICAL_IDLE;
                    f[function_count].h_mode = HORIZONTAL_IDLE;
                    f[function_count].head_mode = HEADING_RATE;
                }

                if (type == "heading_lock") {
                    f[function_count].type = heading_lock;
                    f[function_count].v_mode = VERTICAL_IDLE;
                    f[function_count].h_mode = HORIZONTAL_IDLE;
                    f[function_count].head_mode = HEADING_LOCK;
                }
                if (type == "heading_unlock") {
                    f[function_count].type = heading_unlock;
                    f[function_count].v_mode = VERTICAL_IDLE;
                    f[function_count].h_mode = HORIZONTAL_IDLE;
                    f[function_count].head_mode = HEADING_IDLE;
                }
                if (type == "rotate") {
                    f[function_count].type = rotate;
                    f[function_count].v_mode = VERTICAL_IDLE;
                    f[function_count].h_mode = HORIZONTAL_IDLE;
                    f[function_count].head_mode = HEADING_RATE;
                }
                if (type == "move_oa") {
                    f[function_count].type = move_oa;

                }
                if (type == "move") f[function_count].type = move;
                if (type == "go") f[function_count].type = go;
                if (type == "go_oa") f[function_count].type = go_oa;
                for(int i = 0; i < MAX_FUNCTION_ARG_COUNT; i++) {
                    float arg;
                    stream >> arg;
                    f[function_count].arg[i] = arg;
                }
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
        if(f[current_function_no].done == true) current_function_no ++;

        ros::spinOnce();
    }

    return 0;
}

