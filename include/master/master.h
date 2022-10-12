#ifndef MASTER_H_
#define MASTER_H_

#include "ros/ros.h"
#include "master/Vision.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include <termios.h>
#include <sys/ioctl.h>
#include <string.h>

//---Vision
#include "master/Vision.h"

//---Role Switching
#include "multirole_func/multirole_func.h"

//---Motion
#include "motion/motion.h"

//---Communication
#include "comm/mc_in.h"
#include "comm/mc_out.h"
#include "comm/stm2pc.h"
#include "comm/pc2stm.h"

//---Redis Cpp
// #include <redis-cpp/stream.h>
// #include <redis-cpp/execute.h>

using namespace std;

//=-----------------Global Variable-----------------=
//===================================================
extern float pos_robot[3];
int16_t pos_robot_offset[3];

//---Timer
//========
ros::Timer tim_decision_making;
ros::Timer tim_motor_control;
ros::Timer tim_10_hz;

//---Subscriber---
//================
ros::Subscriber sub_bs2pc;
ros::Subscriber sub_vision_data;
ros::Subscriber sub_stm_data;


//---Publisher---
//===============
ros::Publisher pub_vel_motor;
ros::Publisher pub_mc_out;
ros::Publisher pub_pc2stm;

//---BS data
//==========
uint8_t status_control_BS;
uint8_t BS_manual;
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

uint8_t n_active_robot;
uint8_t n_robot_dekat_bola;
uint8_t n_robot_dapat_bola;
uint8_t n_robot_umpan;
uint8_t n_robot_terima;

uint8_t n_defender_left;
uint8_t n_defender_right;
uint8_t n_attacker;
uint8_t n_assist;

uint8_t robot_base_action;

//---Pc2STM
//=========
int16_t odom_offset_x;
int16_t odom_offset_y;
int16_t odom_offset_th;
uint8_t kicker_mode;
uint8_t kicker_power;
uint16_t kicker_position;
uint8_t buzzer_cnt;
uint8_t buzzer_time;

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
extern float ball_on_field[4];
float ball_on_frame[4];
extern uint8_t ball_status;

//---Robot's datas
//===============
float robot_on_field[4];
float robot_on_field_offset[4];

extern std::vector<uint16_t> obs_on_field;
extern uint8_t total_obs;

//---STM Datas
//============
uint8_t button;
uint8_t tim;
uint8_t tot;

/* Give the current real position from buffer positions */
uint16_t pos_buffer[3];

MultiRole roles[] =
    {
        GkRun,
        AttRun,
        DefRun,
        AssistRun};

void SetPosOffset(int16_t x, int16_t y, int16_t th);
void SetOdometryBuffer(float _x, float _y, float _th);
void SetPosXBuffer(float _val);
void SetPosYBuffer(float _val);
void SetPosThBuffer(float _val);

void SetOffsetRobot(float _x, float _y, float _th);
void SetOffsetRobotX(float _val);
void SetOffsetRobotY(float _val);
void SetOffsetRobotTh(float _val);

void InitDefaultVar();
void GameProcess();
void GetKeyboard();
void loadConfig();
uint8_t kbhit();

/**
 * @brief Will return a number from 0 - 7 based on the button pressed
 */
uint8_t GetButton();
void buzzer(uint8_t time, uint8_t n);
void BuzzerControl();
void transmitAll();
int getDataByKey(std::string key);

//--Ros Callback Prototypes
//=========================
void CllbckMotorControl(const ros::TimerEvent &msg);
void CllbckDecMaking(const ros::TimerEvent &msg);
void CllbckVisionData(const master::VisionConstPtr &msg);
void Cllbck10Hz(const ros::TimerEvent &msg);
void CllbckBs2Pc(const comm::mc_inConstPtr &msg);
void CllbckStm2Pc(const comm::stm2pcConstPtr &msg);
#endif