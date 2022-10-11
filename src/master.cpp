/**
 * @author IRIS ITS
 * @brief All data will be fuse in this Node
 * This Node will work in 50 Hz
 * This Node will produced a robot_action that will be send to Motion Control Node (1000 Hz)
 *
 * */

//---Role Packages
#include "ros/ros.h"
#include "comm/mc_in.h"
#include "master/BS_utils.h"
#include "master/master.h"
#include "yaml-cpp/yaml.h"
#include <chrono>
#include <string.h>
#include "ros/package.h"
#include "std_msgs/Int16MultiArray.h"

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "decision_maker");
    ros::NodeHandle NH;
    ros::MultiThreadedSpinner spinner(4); // Sementara hanya punya 2 callback

    tim_decision_making = NH.createTimer(ros::Duration(0.01), CllbckDecMaking);
    tim_10_hz = NH.createTimer(ros::Duration(0.1), Cllbck10Hz);

    sub_bs2pc = NH.subscribe("bs2pc", 10, CllbckBs2Pc);
    sub_vision_data = NH.subscribe("/vision_data", 10, CllbckVisionData);
    sub_stm_data = NH.subscribe("/odometry_data", 10, CllbckStm2Pc);

    // tim_motor_control = NH.createTimer(ros::Duration(0.02), CllbckMotorControl);

    /**
     * TODO: Get data from motion, pub in transmit all
    */
    pub_vel_motor = NH.advertise<geometry_msgs::Twist>("/cmd_vel_motor", 10);
    pub_mc_out = NH.advertise<comm::mc_out>("mc_out", 10);
    pub_pc2stm = NH.advertise<comm::pc2stm>("pc2stm", 10);
    

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
        case 'j':
            BS_cmd = status_keyboard_maju;
            break;
        case 'b':
            BS_cmd = status_keyboard_kiri;
            break;
        case 'n':
            BS_cmd = status_keyboard_mundur;
            break;
        case 'm':
            BS_cmd = status_keyboard_kanan;
            break;
        case '0':
            BS_cmd = status_keyboard_rotasi_kanan;
            break;
        case '9':
            BS_cmd = status_keyboard_rotasi_kiri;
            break;
        case ' ':
            // BS_cmd = status_iddle_2;
            SetOffsetRobot(0, 0, 100);
            break;
        case 'o':
            buzzer(10, 100);
            break;
        }
    }
}

void SetPosOffset(int16_t x, int16_t y, int16_t th)
{
    printf("KENEK\n");
}

void CllbckMotorControl(const ros::TimerEvent &msg)
{
}

void CllbckOdometryData(const geometry_msgs::Pose2DConstPtr &msg)
{
    SetOdometryBuffer(msg->x, msg->y, msg->theta);
}

void SetOdometryBuffer(float _x, float _y, float _theta)
{
    SetPosXBuffer(_x);
    SetPosYBuffer(_y);
    SetPosThBuffer(_theta);
}

void SetPosXBuffer(float _val)
{
    pos_buffer[0] = _val;
    pos_robot[0] = pos_buffer[0] - robot_on_field_offset[0];
}
void SetPosYBuffer(float _val)
{
    pos_buffer[1] = _val;
    pos_robot[1] = pos_buffer[1] - robot_on_field_offset[1];
}
void SetPosThBuffer(float _val)
{
    pos_buffer[2] = _val;
    pos_robot[2] = robot_on_field_offset[2] - pos_buffer[2];
    while (pos_robot[2] > 180)
        pos_robot[2] -= 360;
    while (pos_robot[2] < -180)
        pos_robot[2] += 360;
}
void SetOffsetRobot(float _x, float _y, float _th)
{
    SetOffsetRobotX(_x);
    SetOffsetRobotY(_y);
    SetOffsetRobotTh(_th);
}
void SetOffsetRobotX(float _val)
{
    robot_on_field_offset[0] = pos_buffer[0] - _val;
}
void SetOffsetRobotY(float _val)
{
    robot_on_field_offset[1] = pos_buffer[1] - _val;
}
void SetOffsetRobotTh(float _val)
{
    robot_on_field_offset[2] = pos_buffer[2] + _val;
    while (robot_on_field_offset[2] > 180)
        robot_on_field_offset[2] -= 360;
    while (robot_on_field_offset[2] < -180)
        robot_on_field_offset[2] += 360;
}

void InitDefaultVar()
{
    robot_on_field_offset[0] = 0;
    robot_on_field_offset[1] = 0;
    robot_on_field_offset[2] = 0;
}

void CllbckBs2Pc(const comm::mc_inConstPtr &msg)
{
    // Iam controlled by BS? (bit-selection)
    if ((msg->mux_control & robot_num_bin[robot_num]) >> (robot_num - 1))
    {
        BS_manual = msg->base * 0.1;
        BS_auto_callib = msg->base % 10;
        BS_cmd = msg->command;
        style = msg->style;

        manual_x_bs = (int)(msg->manual_x * 0.1);
        manual_y_bs = (int)(msg->manual_y * 0.1);
        manual_th_bs = (int)(msg->manual_th * 0.1);
        me_manual = ((msg->manual_x % 10) == robot_num);

        /**
         * data is being devided by 6^x started from 0
         * n_robot_active = how much active robot
         * n_robot_near_ball = num of robot near ball
         * n_robot_has_ball = num of robot has ball
         * n_robot_pass = num of robot pass
         * n_robot_recv = num of robot recv
         */
        n_robot_aktif = msg->data_mux1 % 6;
        n_robot_dekat_bola = (int)(msg->data_mux1 * 0.1666666666) % 6;
        n_robot_dapat_bola = (int)(msg->data_mux1 * 0.02777777) % 6;
        n_robot_umpan = (int)(msg->data_mux1 * 0.0046296) % 6;
        n_robot_terima = (int)(msg->data_mux1 * 0.00077160493) % 6;

        /**
         * Switching role will be executed by BS
         */
        n_defender_left = msg->data_mux2 % 6;
        n_defender_right = (int)(msg->data_mux2 * 0.1666666) % 6;
        n_attacker = (int)(msg->data_mux2 * 0.02777777) % 6;
        n_assist = (int)(msg->data_mux2 * 0.0046296) % 6;

        static uint8_t offset_me;
        static int16_t prev_offset_x = 0;
        static int16_t prev_offset_y = 0;
        static int16_t prev_offset_th = 0;

        offset_me = ((msg->offset_x % 10) == robot_num);
        if (offset_me &&
                (prev_offset_x == 0 && msg->offset_x != 0) ||
            (prev_offset_y == 0 && msg->offset_y != 0) ||
            (prev_offset_th == 0 && msg->offset_th != 0))
            SetOffsetRobot((int)(msg->offset_x * 0.1), (int)(msg->offset_y * 0.1), (int)(msg->offset_th * 0.1));

        prev_offset_x = msg->offset_x;
        prev_offset_y = msg->offset_y;
        prev_offset_th = msg->offset_th;
    }
}

void GameProcess()
{
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

    ball_status = msg->ball_status;

    obs_on_field = msg->obs_on_field;
}

uint8_t GetButton()
{
    if ((button & 0x01) != 0x01)
        return 0;
    if ((button & 0x02) != 0x02)
        return 1;
    if ((button & 0x04) != 0x04)
        return 2;
    if ((button & 0x08) != 0x08)
        return 3;
    if ((button & 0x10) != 0x10)
        return 4;
    if ((button & 0x20) != 0x20)
        return 5;
    if ((button & 0x40) != 0x40)
        return 6;
    if ((button & 0x80) != 0x80)
        return 7;
}

// void CllbckLineSensor(const std_msgs::UInt8ConstPtr &msg)
// {
//     // printf("line sensor: %d\n", msg->data);
//     SetLineSensor(msg->data);
// }

void CllbckDecMaking(const ros::TimerEvent &msg)
{
    // GetKeyboard();
    static uint8_t prev_BS_cmd = 0;
    static uint8_t prev_prev_BS_cmd = 0;
    // command pertama pasti preparation tapi diawali dengan stop dari basestation
    // command kedua bisa jadi start atau stop
    if (!BS_manual)
    {
        if (prev_BS_cmd != BS_cmd)
        {
            if (BS_cmd == 's')
            {
                if (game_status > 0 && game_status <= 127)
                    game_status += 128;
                robot_base_action = 1;
            }
            else if (BS_cmd == 'S')
            {
                if (game_status > 127 && game_status <= 255)
                    game_status -= 128;
                robot_base_action = 0;
            }
            else
            {
                game_status = BS_cmd;
                robot_base_action = 1;
            }
        }
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

    transmitAll();

    // printf("game_status: %d %d\n", game_status, robot_base_action);

    // GameProcess();
}

//---TODO: taruh utils
void buzzer(uint8_t time, uint8_t n)
{
    tim = time;
    tot = n;
    tim_10_hz.start();
}

void BuzzerControl()
{
    if ((tim != 0) || (tot != 0))
    {
        buzzer(0, 0);
    }
    tim_10_hz.stop();
}

void Cllbck10Hz(const ros::TimerEvent &msg)
{
    BuzzerControl();
}

void CllbckStm2Pc(const comm::stm2pcConstPtr &msg)
{
    SetOdometryBuffer(msg->odom_buffer_x, msg->odom_buffer_y, msg->odom_buffer_th);
}

void transmitAll()
{
    comm::mc_out msg_mc_out;
    msg_mc_out.pos_x = pos_robot[0];
    msg_mc_out.pos_y = pos_robot[1];
    msg_mc_out.theta = pos_robot[2];
    msg_mc_out.ball_x = ball_on_field[0];
    msg_mc_out.ball_y = ball_on_field[1];
    msg_mc_out.ball_status = ball_status;
    msg_mc_out.pass_target = 0;
    msg_mc_out.robot_cond = robot_action;
    pub_mc_out.publish(msg_mc_out);

    comm::pc2stm msg_pc2stm;
    msg_pc2stm.buzzer_cnt = buzzer_cnt;
    msg_pc2stm.buzzer_time = buzzer_time;
    msg_pc2stm.kicker_mode = kicker_mode;
    msg_pc2stm.kicker_position = kicker_position;
    msg_pc2stm.kicker_power = kicker_power;
    msg_pc2stm.odom_offset_th = odom_offset_th;
    msg_pc2stm.odom_offset_x = odom_offset_x;
    msg_pc2stm.odom_offset_y = odom_offset_y;
    pub_pc2stm.publish(msg_pc2stm);
}
// void CllbckButtons(const std_msgs::UInt8ConstPtr &msg)
// {
//     button = msg->data;
// }

// void CllbckBallSensor(const std_msgs::UInt8MultiArrayConstPtr &msg)
// {
//     SetBallSensor(msg->data);
// }