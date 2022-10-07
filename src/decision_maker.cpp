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
    ros::MultiThreadedSpinner spinner(0); // Sementara hanya punya 2 callback

    InitDefaultVar();
    tim_decision_making = NH.createTimer(ros::Duration(0.01), CllbckDecMaking);

    sub_pc2bs = NH.subscribe("bs2pc", 10, CllbckPc2Bs);
    sub_vision_data = NH.subscribe("/vision_data", 16, CllbckVisionData);
    sub_odometry_data = NH.subscribe("/odometry_data", 16, CllbckOdometryData);
    sub_buttons = NH.subscribe("/button", 1, CllbckButtons);
    sub_line_sensor = NH.subscribe("/line_sensor", 1, CllbckLineSensor);
    sub_ball_sensor = NH.subscribe("/ball_sensor", 2, CllbckBallSensor);

    // tim_motor_control = NH.createTimer(ros::Duration(0.02), CllbckMotorControl);

    pub_vel_motor = NH.advertise<geometry_msgs::Twist>("/cmd_vel_motor", 16);
    pub_offset_robot = NH.advertise<geometry_msgs::Pose2D>("/offset_robot", 16);

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
            BS_cmd = status_iddle_2;
            break;
        case 'o':
            SetPosOffset(0, 0, 90);
            break;
        }
    }
}

void SetPosOffset(int16_t x, int16_t y, int16_t th)
{
    printf("KENEK\n");
    geometry_msgs::Pose2D pos_offset;
    pos_offset.x = x;
    pos_offset.y = y;
    pos_offset.theta = th;
    pub_offset_robot.publish(pos_offset);
}

void CllbckMotorControl(const ros::TimerEvent &msg)
{
}

void CllbckOdometryData(const geometry_msgs::Pose2DConstPtr &msg)
{
    setOdometryBuffer(msg->x, msg->y, msg->theta);
}

void setOdometryBuffer(float _x, float _y, float _theta)
{
    SetPosXBuffer(_x);
    SetPosYBuffer(_y);
    SetPosThBuffer(_theta);
}

void SetPosXBuffer(float _val) { pos_robot[0] = _val - robot_on_field_offset[0]; }
void SetPosYBuffer(float _val) { pos_robot[1] = _val - robot_on_field_offset[1]; }
void SetPosThBuffer(float _val)
{
    pos_robot[2] *= -1;
    pos_robot[2] = _val - robot_on_field_offset[2];
    while (pos_robot[2] > 180)
        pos_robot[2] -= 360;
    while (pos_robot[2] < -180)
        pos_robot[2] += 360;
}

void InitDefaultVar()
{
    // pos_robot_offset[0] = 0;
    // pos_robot_offset[1] = 0;
    // pos_robot_offset[2] = 0;
    robot_on_field_offset[0] = 0;
    robot_on_field_offset[1] = 0;
    robot_on_field_offset[2] = 0;
    gk_data.robot_x[1] = 0;
    gk_data.robot_y[1] = 0;
    gk_data.robot_th[1] = 90;
    gk_ret.vel_x_gain = 0;
    gk_ret.vel_y_gain = 0;
    gk_ret.vel_th_gain = 0;
    // loadConfig();
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
        gk_data.robot_x[1] = robot_on_field[0];
        gk_data.robot_y[1] = robot_on_field[1];
        gk_data.robot_th[1] = robot_on_field[2];
        gk_data.ball_x = ball_on_field[0];
        gk_data.ball_y = ball_on_frame[1];
        gk_data.game_status = game_status;
        GkRun(&gk_data, &gk_ret);

        // printf("GK RET: %f %f %f\n", gk_ret.vel_x_gain, gk_ret.vel_y_gain, gk_ret.vel_th_gain);

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

    // printf("robot_act: %d\n", robot_action);
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

void CllbckButtons(const std_msgs::UInt8ConstPtr &msg){
    button = msg->data;
}

uint8_t GetButton()
{
    if((button & 0x01) != 0x01)
        return 0;
    if((button & 0x02) != 0x02)
        return 1;
    if((button & 0x04) != 0x04)
        return 2;
    if((button & 0x08) != 0x08)
        return 3;
    if((button & 0x10) != 0x10)
        return 4;
    if((button & 0x20) != 0x20)
        return 5;
    if((button & 0x40) != 0x40)
        return 6;
    if((button & 0x80) != 0x80)
        return 7;
}

void CllbckLineSensor(const std_msgs::UInt8ConstPtr &msg){
    // printf("line sensor: %d\n", msg->data);
    SetLineSensor(msg->data);
}

void CllbckDecMaking(const ros::TimerEvent &msg)
{
    // printf("odom: %f %f %f\n", pos_robot[0], pos_robot[1], pos_robot[2]);
    GetKeyboard();
    if(LeftLineSensorDetected())
    {
        printf("Left Line Sensor Detected\n");
    }
    if(RightLineSensorDetected())
    {
        printf("Right Line Sensor Detected\n");
    }
    if(LeftLineSensorDetected() && RightLineSensorDetected())
    {
        printf("Both Line Sensor Detected\n");
    }
    // geometry_msgs::Pose2D _offset;
    // _offset.x = gk_data.robot_x[1];
    // _offset.y = gk_data.robot_y[1];
    // _offset.theta = gk_data.robot_th[1];
    // pub_offset_robot.publish(_offset);
    geometry_msgs::Twist msg_vel;
    msg_vel.linear.x = gk_ret.vel_x_gain;
    msg_vel.linear.y = gk_ret.vel_y_gain;
    msg_vel.angular.z = gk_ret.vel_th_gain;
    // printf("vel: %f %f %f\n", msg_vel.linear.x, msg_vel.linear.y, msg_vel.angular.z);
    pub_vel_motor.publish(msg_vel);

    // ObstacleCheck(90, 90, 60);
    // printf("Pos robot on dec maker : %d %d %d \n", pos_robot[0], pos_robot[1], pos_robot[2]);
    // GetKeyboard();
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
        game_status = status_iddle;
    }
    prev_prev_BS_cmd = prev_BS_cmd;
    prev_BS_cmd = BS_cmd;
    // }
    // else
    // {
    //     if (me_manual == robot_num)
    //     {
    //         // ....
    // printf("Aku manual..\n");
    //     }
    // }
    // game_status = 'K';
    printf("Ball sensors: %d %d\n", ball_sensor[0], ball_sensor[1]);
    GameProcess();
}

void CllbckBallSensor(const std_msgs::UInt8MultiArrayConstPtr &msg){
    SetBallSensor(msg->data);
}