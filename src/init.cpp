#include "master/config.h"


Config::Config()
{
    emitter_ << YAML::BeginMap;
    ros::param::get("robot_name", robot_name);

    std::cout << robot_name << std::endl;

    if(robot_name == "iris1")
        CONFIG_FILE_PATH = "../../config/IRIS1/";
    else if(robot_name == "iris2")
        CONFIG_FILE_PATH = "../../config/IRIS2/";
    else if(robot_name == "iris3")
        CONFIG_FILE_PATH = "../../config/IRIS3/";
    else if(robot_name == "iris4")
        CONFIG_FILE_PATH = "../../config/IRIS4/";
    else if(robot_name == "iris5")
        CONFIG_FILE_PATH = "../../config/IRIS5/";
    
    if(CONFIG_FILE_PATH.empty())
        ROS_FATAL("export ROBOT_NAME tidak terdeteksi");
    else
        ROS_INFO_ONCE("robot name: %s", robot_name.c_str());
}

Config::~Config()
{
    
}

void Config::load(std::string path)
{
    // Mengubah agar direktori diprogram menuju direktori config
    std::string current_dir = ros::package::getPath("master");

    if(chdir(current_dir.c_str()) != 0)
        perror("chdir() failed dir");

    if(chdir(CONFIG_FILE_PATH.c_str()) != 0)
        perror("chdir() failed cfg");

    // Menyimpan nama file direktori
    char get_dir[128];
    getcwd(get_dir, sizeof(get_dir));

    std::stringstream filename;
    filename << get_dir << "/" << path;

    node_parser_ = YAML::LoadFile(filename.str());
    while(node_stack_.size() > 0)
        node_stack_.pop();
    node_stack_.push(node_parser_);
}

// void Config::save(std::string path)
// {
//     // Mengubah agar direktori diprogram menuju direktori config
//     std::string current_dir = ros::package::getPath("master");

//     if(chdir(current_dir.c_str()) != 0)
//         perror("chdir() failed");

//     if(chdir(CONFIG_FILE_PATH.c_str()) != 0)
//         perror("chdir() failed");

//     // Menyimpan nama file direktori
//     char get_dir[128];
//     getcwd(get_dir, sizeof(get_dir));

//     std::stringstream filename;
//     filename << get_dir << "/" << path;

//     std::ofstream output(filename.str(), std::ofstream::out);
//     output << emitter_.c_str();
//     output.close();
// }