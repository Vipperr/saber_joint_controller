#ifndef __CYBERGEAR_CONTROL_HPP__
#define __CYBERGEAR_CONTROL_HPP__

#include <math.h>
#include <linux/can.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <cstdint>
#include <unistd.h>
#include <string.h>
#include <iostream>

#define P_MIN -12.5f 
#define P_MAX 12.5f 
#define V_MIN -30.0f 
#define V_MAX 30.0f 
#define KP_MIN 0.0f 
#define KP_MAX 500.0f 
#define KD_MIN 0.0f 
#define KD_MAX 5.0f 
#define T_MIN -12.0f 
#define T_MAX 12.0f 

#define run_mode 0X7005
#define	iq_ref 0X7006
#define	spd_ref 0X700A
#define	imit_torque 0X700B
#define	cur_kp 0X7010
#define	cur_ki 0X7011
#define	cur_filt_gain 0X7014
#define	loc_ref 0X7016
#define	limit_spd 0X7017
#define	limit_cur 0X7018
#define	mechPos 0x7019
#define	iqf 0x701A
#define	mechVel 0x701B
#define	VBUS 0x701C
// #define	rotation 0x701D
#define	loc_kp 0x701E
#define	spd_kp 0x701F
#define	spd_ki 0x7020

#define angle_min -4*PI
#define angle_max 4*PI
#define angular_velocity_min -30.0f
#define angular_velocity_max 30.0f
#define moment_min -12.0f
#define moment_max 12.0f

#define master1_id 0
#define motor1_id 1
#define motor2_id 2
#define motor3_id 3
#define motor4_id 4
#define motor5_id 5
#define motor6_id 6


struct exCanIdInfo{ 
uint32_t motor_id:8; 
uint32_t data:16; 
uint32_t mode:5; 
uint32_t res:3; 
};


#define txCanIdEx (((struct exCanIdInfo *)&(TxMessage.can_id))) 
#define rxCanIdEx (((struct exCanIdInfo *)&(RxMessage.can_id))) //将扩展帧id 解析为自定义数据结构
#define PI  M_PI


int float_to_uint(float x,float x_min,float x_max,int bits);
float data_conversion(float data,float min,float max);
void acquire_device_id(uint8_t motor_id, uint16_t master_id, int sock);
void motor_controlmode(uint8_t motor_id, float torque, float MechPosition, float speed, float kp, float kd, int sock);
void motor_enable(uint8_t motor_id, uint16_t master_id, int sock);
void motor_stop(uint8_t motor_id, uint16_t master_id, int sock);
void set_mechzero(uint8_t motor_id, uint16_t master_id, int sock);
void set_id(uint8_t new_id,uint8_t motor_id, uint16_t master_id, int sock);
void read_single_paramter(uint16_t index,uint8_t motor_id, uint16_t master_id, int sock);
void motor_modechange(uint16_t index,uint8_t runmode,uint8_t motor_id, uint16_t master_id, int sock);
void motor_write(uint16_t index,float ref ,uint8_t motor_id, uint16_t master_id, int sock);
void current_mode_write(uint8_t motor_id, uint16_t master_id,float ref, int sock);
void speed_mode_write(uint8_t motor_id, uint16_t master_id,float current_limit,float ref, int sock);
void location_mode_write(uint8_t motor_id, uint16_t master_id,float speed_limit,float ref);

#endif