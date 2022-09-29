/**
 * @author IRIS ITS
 * @brief All data will be fuse in this Node
 * This Node will work in 50 Hz
 * This Node will produced a robot_action that will be send to Motion Control Node (1000 Hz)
 *
 * */

#include "ros/ros.h"
#include "comm/mc_in.h"
#include "master/BS_utils.h"
#include "goalkeeper/goalkeeper.h"
#include "attacker/attacker.h"

// ROS
ros::Timer main_prog;

// BS data
uint8_t status_control_BS;
uint8_t BS_normal;
uint8_t BS_auto_callib;
uint8_t BS_cmd;
uint8_t style;
int16_t manual_x_bs;
int16_t manual_y_bs;
int16_t manual_th_bs;
uint16_t data_mux1;
uint16_t data_mux2;
uint16_t mux_control;
uint8_t me_manual;

// Multirole data
gk_data_t gk_data;
gk_ret_t gk_ret;
att_data_t att_data;

// Data sintetis sementara
uint8_t robot_num = 1;
uint8_t robot_role = 1;
uint8_t robot_num_bin[6] = {0b00000, 0b00001, 0b00010, 0b00100, 0b01000, 0b100000};

// MS
uint8_t game_status;
uint8_t robot_action;

void init_default_var()
{
    gk_data.robot_x[1] = 400;
    gk_data.robot_y[1] = 0;
    gk_data.robot_th[1] = 90;
    gk_ret.vel_x_gain = 1;
    gk_ret.vel_y_gain = 1;
    gk_ret.vel_th_gain = 1;
}

void cllbck_pc2bs(const comm::mc_inConstPtr &msg)
{
    // Iam controlled by BS? (bit-selection)
    if ((msg->mux_control & robot_num_bin[robot_num]) >> (robot_num - 1))
    {
        BS_normal = msg->base / 10;
        BS_auto_callib = msg->base % 10;
        BS_cmd = msg->command;
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
            printf("BS menyuruh offset\n");
        }
    }
}

void game_process()
{
    switch (robot_role)
    {
    case 1:
        // gk_data.game_status = 'K';
        gk_data.robot_num = robot_num;
        gk_data.robot_x[1] = 12;
        gk_data.robot_y[1] = 32;
        gk_data.ball_x = 123;
        gk_data.ball_y = 300;
        gk_data.game_status = game_status;
        gk_run(&gk_data, &gk_ret);

        printf("ret: %d %d %d\n", gk_ret.act_type, gk_ret.target_x, gk_ret.target_y);

        // Ret send to Motion Control node

        break;
    case 2:
        att_data.robot_num = robot_num;
        att_data.robot_x[1] = 12;
        att_data.robot_y[1] = 32;
        att_data.ball_x = 123;
        att_data.ball_y = 234;
        att_data.game_status = game_status;
        att_run(&att_data, &robot_action);

        // Ret send to Motion Control node

        break;
    }

    // printf("robot_act: %d\n", robot_action);
}

void cllbck_main(const ros::TimerEvent &)
{
    // static uint8_t prev_BS_cmd = 0;
    // static uint8_t prev_prev_BS_cmd = 0;
    // if (BS_normal)
    // {
    //     // Prep..
    //     if (prev_BS_cmd != BS_cmd && BS_cmd != 's' && BS_cmd != 'S')
    //     {
    //         game_status = BS_cmd;
    //     }

    //     // Start
    //     if (prev_BS_cmd != BS_cmd && BS_cmd == 's')
    //     {
    //         // Safety tambahan ketika belum prep
    //         if (game_status > 0 && game_status <= 127)
    //             game_status += 128;
    //     }

    //     // Stop
    //     if (prev_BS_cmd != BS_cmd && BS_cmd == 'S')
    //     {
    //         game_status = 0;
    //     }
    //     prev_prev_BS_cmd = prev_BS_cmd;
    //     prev_BS_cmd = BS_cmd;
    // }
    // else
    // {
    //     if (me_manual == robot_num)
    //     {
    //         // ....
    //         printf("Aku manual..\n");
    //     }
    // }
    game_status = 'K';
    // printf("Status game: %d\n", game_status);
    game_process();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "decision_maker");
    ros::NodeHandle NH;
    ros::MultiThreadedSpinner spinner(2); // Sementara hanya punya 2 callback

    init_default_var();

    ros::Subscriber sub_pc2bs = NH.subscribe("bs2pc", 10, cllbck_pc2bs);

    main_prog = NH.createTimer(ros::Duration(0.02), cllbck_main);

    spinner.spin();

    return 0;
}