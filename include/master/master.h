#ifndef MASTER_H_
#define MASTER_H_

#include "ros/ros.h"
#include "master/Vision.h"
//include termios
#include <termios.h>
#include <sys/ioctl.h>

//---Enumeration
//==============

enum robot_state{
    //---General Cmd
    status_iddle = 83, // S | 0x53
    status_start = 115, // s | 0x73

    //---Home Cmd
    status_preparation_kickoff_home = 75, // K | 0x4B
    status_preparation_freekick_home = 70, // F | 0x46
    status_preparation_goalkick_home = 71, // G | 0x47
    status_preparation_cornerkick_home = 67, // C | 0x43
    status_preparation_penaltykick_home = 80, // P | 0x50
    status_preparation_throwin_home = 84, // T | 0x54

    //---All Cmd
    status_preparation_dropball = 78, // N | 0x4E
    status_callibration = 35, // # | 0x23
    status_park = 76, // L | 0x4C

    //---Away Cmd
    status_preparation_kickoff_away = 107, // k | 0x6B
    status_preparation_freekick_away = 102, // f | 0x66
    status_preparation_goalkick_away = 103, // g | 0x67
    status_preparation_cornerkick_away = 99, // c | 0x63
    status_preparation_penaltykick_away = 112, // p | 0x70
    status_preparation_throwin_away = 116, // t | 0x74

};

//---Timer
//========
ros::Timer tim_decision_making;
ros::Timer tim_motor_control;

//---Subscriber---
//================
ros::Subscriber sub_pc2bs;
ros::Subscriber sub_vision_data;
//---Publisher---

//---BS data
//==========
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

//---Multirole data
//=================
gk_data_t gk_data;
gk_ret_t gk_ret;
att_data_t att_data;

//---Data sintetis sementara
//==========================
uint8_t robot_num = 1;
uint8_t robot_role = 1;
uint8_t robot_num_bin[6] = {0b00000, 0b00001, 0b00010, 0b00100, 0b01000, 0b100000};

//---MS
//=====
uint8_t game_status;
uint8_t robot_action;

//---Ball's datas
//==============
float ball_on_field[4];
float ball_on_frame[4];
uint8_t ball_status;

//--Prototypes
//============
void CllbckVisionData(const master::VisionConstPtr &msg);
void CllbckMotorControl(const ros::TimerEvent &msg);
void CllbckPc2Bs(const comm::mc_inConstPtr &msg);
void CllbckDecMaking(const ros::TimerEvent &msg);
void InitDefaultVar();
void GameProcess();
void GetKeyboard();
void loadConfig();
uint8_t kbhit();
#endif