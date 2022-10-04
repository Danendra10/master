/**
 * @author IRIS ITS
 * @brief All data will be fuse in this Node
 * This Node will work in 50 Hz
 * This Node will produced a robot_action that will be send to Motion Control Node (1000 Hz)
 *
 * */

//---Role Packages
#include "goalkeeper/goalkeeper.h"
#include "attacker/attacker.h"
// #include "assist/assist.h"
// #include "defender/defender.h"

#include "ros/ros.h"
#include "comm/mc_in.h"
#include "master/BS_utils.h"
#include "master/master.h"
#include "yaml-cpp/yaml.h"
#include <chrono>
#include <string.h>
#include "ros/package.h"

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "decision_maker");
    ros::NodeHandle NH;
    ros::MultiThreadedSpinner spinner(2); // Sementara hanya punya 2 callback

    InitDefaultVar();

    sub_pc2bs = NH.subscribe("bs2pc", 10, CllbckPc2Bs);
    sub_vision_data = NH.subscribe("/vision_data", 10, CllbckVisionData);

    tim_decision_making = NH.createTimer(ros::Duration(0.02), CllbckDecMaking);
    tim_motor_control = NH.createTimer(ros::Duration(0.001), CllbckMotorControl);

    spinner.spin();

    return 0;
}

uint8_t kbhit()
{
    static const int STDIN = 0;
    static bool initialized = false;

    if (!initialized)
    {
        termios term;
        tcgetattr(STDIN, &term);
        term.c_lflag &= ~ICANON;
        tcsetattr(STDIN, TCSANOW, &term);
        setbuf(stdin, NULL);
        initialized = true;
    }

    int bytesWaiting;
    ioctl(STDIN, FIONREAD, &bytesWaiting);
    return bytesWaiting;
}

void GetKeyboard()
{
    if (kbhit())
    {
        char key = std::cin.get();

        switch (key)
        {
        case 'S':
            BS_cmd = status_iddle;
            break;
        case 's':
            BS_cmd = status_start;
            break;
        case '#':
            BS_cmd = status_callibration;
            break;
        case 'K':
            BS_cmd = status_preparation_kickoff_home;
            break;
        }
    }
}

void CllbckMotorControl(const ros::TimerEvent &msg)
{
}

void InitDefaultVar()
{
    gk_data.robot_x[1] = 400;
    gk_data.robot_y[1] = 0;
    gk_data.robot_th[1] = 90;
    gk_ret.vel_x_gain = 1;
    gk_ret.vel_y_gain = 1;
    gk_ret.vel_th_gain = 1;

    loadConfig();
}

void CllbckPc2Bs(const comm::mc_inConstPtr &msg)
{
    // Iam controlled by BS? (bit-selection)
    if ((msg->mux_control & robot_num_bin[robot_num]) >> (robot_num - 1))
    {
        BS_normal = msg->base * 0.1;
        BS_auto_callib = msg->base % 10;
        BS_cmd = msg->command;
        cout << "BS_normal: " << BS_normal << endl;
        cout << "BS CMD: " << BS_cmd << endl;
        style = msg->style;

        manual_x_bs = msg->manual_x;
        manual_y_bs = msg->manual_y;
        manual_th_bs = msg->manual_th;
        me_manual = (msg->manual_x % 10 && robot_num);

        data_mux1 = msg->data_mux1;
        data_mux2 = msg->data_mux2;
        mux_control = msg->mux_control;

        static uint8_t offset_me;
        static int16_t prev_offset_x;
        static int16_t prev_offset_y;
        static int16_t prev_offset_th;

        offset_me = msg->offset_x % 10;
        if ((offset_me == robot_num) &&
            (prev_offset_x == 0 && msg->offset_x != 0) &&
            (prev_offset_y == 0 && msg->offset_y != 0) &&
            (prev_offset_th == 0 && msg->offset_th != 0))
        {
            // set offset..
            // printf("BS menyuruh offset\n");
        }
    }
}

void GameProcess()
{
    switch (robot_role)
    {
    case 1:
        // gk_data.game_status = 'K';
        gk_data.robot_num = robot_num;
        gk_data.robot_x[1] = 12;
        gk_data.robot_y[1] = 32;
        gk_data.ball_x = ball_on_field[0];
        gk_data.ball_y = ball_on_frame[1];
        gk_data.game_status = game_status;
        GkRun(&gk_data, &gk_ret);

        // printf("ret: %d %d %d\n", gk_ret.act_type, gk_ret.target_x, gk_ret.target_y);

        // Ret send to Motion Control node

        break;
        // case 2:
        //     att_data.robot_num = robot_num;
        //     att_data.robot_x[1] = 12;
        //     att_data.robot_y[1] = 32;
        //     att_data.ball_x = 123;
        //     att_data.ball_y = 234;
        //     att_data.game_status = game_status;
        //     att_run(&att_data, &robot_action);

        //     // Ret send to Motion Control node

        //     break;
    }

    printf("robot_act: %d\n", robot_action);
}

void CllbckDecMaking(const ros::TimerEvent &msg)
{
    GetKeyboard();
    // switch (BS_cmd)
    // {
    // case status_iddle:
    //     printf("ini status iddle\n");
    //     break;
    // case status_callibration:
    //     printf("ini status callibration\n");
    //     break;
    // default:
    //     break;
    // }
    static uint8_t prev_BS_cmd = 0;
    static uint8_t prev_prev_BS_cmd = 0;
    // command pertama pasti preparation tapi diawali dengan stop dari basestation
    // command kedua bisa jadi start atau stop
    // if(prev_BS_cmd != BS_cmd && BS_cmd != status_iddle && BS_cmd != status_start)
    //     game_status = BS_cmd;
    // if(prev_prev_BS_cmd != BS_cmd && BS_cmd == status_start)
    //     game_status += 128;
    // if(BS_cmd == status_iddle)
    //     game_status = BS_cmd;
    // prev_prev_BS_cmd = prev_BS_cmd;
    // prev_BS_cmd = BS_cmd;
    // if (BS_normal)
    // {

    // Prep..
    if (prev_BS_cmd != BS_cmd && BS_cmd != 's' && BS_cmd != 'S')
    {
        game_status = BS_cmd;
    }

    // Start
    if (prev_BS_cmd != BS_cmd && BS_cmd == 's')
    {
        // Safety tambahan ketika belum prep
        if (game_status > 0 && game_status <= 127)
            game_status += 128;
    }

    // Stop
    if (prev_BS_cmd != BS_cmd && BS_cmd == 'S')
    {
        game_status = 0;
    }
    prev_prev_BS_cmd = prev_BS_cmd;
    prev_BS_cmd = BS_cmd;
    // }
    // else
    // {
    //     if (me_manual == robot_num)
    //     {
    //         // ....
    printf("Aku manual..\n");
    //     }
    // }
    // game_status = 'K';
    printf("Status game: %d\n", game_status);
    GameProcess();
}

void CllbckVisionData(const master::VisionConstPtr &msg)
{
    ball_on_field[0] = msg->ball_on_field_x;
    ball_on_field[1] = msg->ball_on_field_y;
    ball_on_field[2] = msg->ball_on_field_theta;
    ball_on_field[3] = msg->ball_on_field_dist;

    ball_on_frame[0] = msg->ball_on_frame_x;
    ball_on_frame[1] = msg->ball_on_frame_y;
    ball_on_frame[2] = msg->ball_on_frame_theta;
    ball_on_frame[3] = msg->ball_on_frame_dist;
    // printf("Ball on frame: %f %f %f %f", ball_on_frame[0], ball_on_frame[1], ball_on_frame[2], ball_on_frame[3]);
}

void loadConfig()
{
    // printf("Loading configuration..\n");
    // char *robot_num = getenv("ROBOT");
    // char config_file[100];
    // std::string current_dir = ros::package::getPath("master");
    // sprintf(config_file, "%s/../../config/IRIS%s/multicast.yaml", current_dir.c_str(), robot_num);
    // printf("config file: %s\n", config_file);

    // YAML::Node config = YAML::LoadFile(config_file);
    // strcpy(nw_config.identifier, config["identifier"].as<std::string>().c_str());
    // strcpy(nw_config.iface, config["iface"].as<std::string>().c_str());
    // strcpy(nw_config.multicast_ip, config["multicast_ip"].as<std::string>().c_str());

    // nw_config.port = config["port"].as<int>();

    // printf("identifier: %s\n", nw_config.identifier);
    // printf("iface: %s\n", nw_config.iface);
    // printf("multicast_ip: %s\n", nw_config.multicast_ip);
    // printf("port: %d\n", nw_config.port);
}