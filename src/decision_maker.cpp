#include "ros/ros.h"
#include "comm/mc_in.h"
#include "master/BS_utils.h"

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

// Data sintetis sementara
uint8_t robot_num = 1;
uint8_t robot_num_bin[6] = {0b00000, 0b00001, 0b00010, 0b00100, 0b01000, 0b100000};

// MS
uint8_t game_status;
uint8_t new_set_piece;

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
        if (offset_me &&
            (prev_offset_x == 0 && msg->offset_x != 0) &&
            (prev_offset_y == 0 && msg->offset_y != 0) &&
            (prev_offset_th == 0 && msg->offset_th != 0))
        {
            // set offset..
            printf("BS menyuruh offset\n");
        }
    }
}

void cllbck_main(const ros::TimerEvent &)
{
    static uint8_t prev_BS_cmd = 0;
    static uint8_t prev_prev_BS_cmd = 0;
    if (BS_normal)
    {
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
    }
    else
    {
        if (me_manual)
        {
            // ....
            printf("Aku manual..\n");
        }
    }
    printf("Status game: %d\n", game_status);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "decision_maker");
    ros::NodeHandle NH;
    ros::MultiThreadedSpinner spinner(1); // Sementara hanya punya 1 callback

    ros::Subscriber sub_pc2bs = NH.subscribe("bs2pc", 10, cllbck_pc2bs);

    main_prog = NH.createTimer(ros::Duration(0.02), cllbck_main);

    spinner.spin();

    return 0;
}