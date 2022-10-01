#include "master/config.h"

int main(int argc, char **argv)
{
    emitter_ << YAML::BeginMap;
    ros::param::get("robot_num", robot_num);
    if(robot_num == "1")
        CONFIG_FILE_PATH = "../../../cfg/IRIS1.yaml";
    else if(robot_num == "2")
        CONFIG_FILE_PATH = "../../../cfg/IRIS2.yaml";
    else if(robot_num == "3")
        CONFIG_FILE_PATH = "../../../cfg/IRIS3.yaml";
    else if(robot_num == "4")
        CONFIG_FILE_PATH = "../../../cfg/IRIS4.yaml";
    else
        CONFIG_FILE_PATH = "../../../cfg/IRIS5.yaml";

    if(access(CONFIG_FILE_PATH.c_str(), F_OK) == 0)    
        load(CONFIG_FILE_PATH);    
    else
    {
        ROS_ERROR("Config file not found!");
        exit(1);
    }
}