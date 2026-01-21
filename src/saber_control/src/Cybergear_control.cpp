#include "Cybergear_control.hpp"

using namespace std;
static struct can_frame TxMessage;
static struct can_frame RxMessage;

int float_to_uint(float x,float x_min,float x_max,int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    if (x > x_max) x = x_max;
    else if (x < x_min) x = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}

float data_conversion(float data,float min,float max)
{
    float span=max-min;
    return (float) ( min+data/((1<<16)-1)*span );
}

//获取设备ID（通信类型0）
void acquire_device_id(uint8_t motor_id, uint16_t master_id, int sock)
{
	txCanIdEx->mode = 0; 
	txCanIdEx->motor_id = motor_id; 
	txCanIdEx->res = 4; 
	txCanIdEx->data = master_id;
	TxMessage.can_dlc = 8; 
	if (write(sock, &TxMessage, sizeof(TxMessage)) == -1)
	{
		cout << "Failed to send CAN frame." << endl;
	}
}

//运控模式电机控制指令（通信类型1）
void motor_controlmode(uint8_t motor_id, float torque, float MechPosition, float speed, float kp, float kd, int sock) 
{ 
	txCanIdEx->mode = 1; 
	txCanIdEx->motor_id = motor_id; 
	txCanIdEx->res = 4; 
	txCanIdEx->data = float_to_uint(torque,T_MIN,T_MAX,16); 
	TxMessage.can_dlc = 8; 
	TxMessage.data[0]=float_to_uint(MechPosition,P_MIN,P_MAX,16)>>8; 
	TxMessage.data[1]=float_to_uint(MechPosition,P_MIN,P_MAX,16); 
	TxMessage.data[2]=float_to_uint(speed,V_MIN,V_MAX,16)>>8; 
	TxMessage.data[3]=float_to_uint(speed,V_MIN,V_MAX,16); 
	TxMessage.data[4]=float_to_uint(kp,KP_MIN,KP_MAX,16)>>8; 
	TxMessage.data[5]=float_to_uint(kp,KP_MIN,KP_MAX,16); 
	TxMessage.data[6]=float_to_uint(kd,KD_MIN,KD_MAX,16)>>8; 
	TxMessage.data[7]=float_to_uint(kd,KD_MIN,KD_MAX,16); 
	if (write(sock, &TxMessage, sizeof(TxMessage)) == -1)
	{
		cout << "Failed to send CAN frame." << endl;
	}
}

//电机使能运行帧（通信类型3）
void motor_enable(uint8_t motor_id, uint16_t master_id, int sock) 
{ 
	txCanIdEx->mode = 3; 
	txCanIdEx->motor_id = motor_id; 
	txCanIdEx->res = 4; 
	txCanIdEx->data = master_id; 
	TxMessage.can_dlc = 8; 
	for(uint8_t i=0;i<8;i++) 
	{ 
		TxMessage.data[i]=0; 
	} 
	if (write(sock, &TxMessage, sizeof(TxMessage)) == -1)
	{
		cout << "Failed to send CAN frame." << endl;
	}
}

//电机停止运行（通信类型4）
void motor_stop(uint8_t motor_id, uint16_t master_id, int sock) 
{ 
	txCanIdEx->mode = 4; 
	txCanIdEx->motor_id = motor_id; 
	txCanIdEx->res = 4; 
	txCanIdEx->data = master_id; 
	TxMessage.can_dlc = 8; 
	for(uint8_t i=0;i<8;i++) 
	{ 
		TxMessage.data[i]=0; 
	} 
	if (write(sock, &TxMessage, sizeof(TxMessage)) == -1)
	{
		cout << "Failed to send CAN frame." << endl;
	}
}

//设置电机机械零位（通信类型6）
void set_mechzero(uint8_t motor_id, uint16_t master_id, int sock)
{
	txCanIdEx->mode = 6; 
	txCanIdEx->motor_id = motor_id; 
	txCanIdEx->res = 4; 
	txCanIdEx->data = master_id; 
	TxMessage.can_dlc = 8; 
	TxMessage.data[0]=1; 
	for(uint8_t i=1;i<8;i++) 
	{ 
		TxMessage.data[i]=0; 
	} 
	if (write(sock, &TxMessage, sizeof(TxMessage)) == -1)
	{
		cout << "Failed to send CAN frame." << endl;
	}
}

//设置电机CAN_ID（通信类型7）
void set_id(uint8_t new_id,uint8_t motor_id, uint16_t master_id, int sock)
{
	txCanIdEx->mode = 7; 
	txCanIdEx->motor_id = motor_id; 
	txCanIdEx->res = 4; 
	txCanIdEx->data = master_id | (new_id<<8) ; 
	TxMessage.can_dlc = 8; 
	if (write(sock, &TxMessage, sizeof(TxMessage)) == -1)
	{
		cout << "Failed to send CAN frame." << endl;
	}
}

//单个参数读取（通信类型17）
void read_single_paramter(uint16_t index,uint8_t motor_id, uint16_t master_id, int sock)
{
	txCanIdEx->mode = 0x11; 
	txCanIdEx->motor_id = motor_id; 
	txCanIdEx->res = 4; 
	txCanIdEx->data = master_id ; 
	TxMessage.can_dlc = 8; 
	for(uint8_t i=0;i<8;i++) 
	{ 
		TxMessage.data[i]=0; 
	}
	memcpy(&TxMessage.data[0],&index,2);
	if (write(sock, &TxMessage, sizeof(TxMessage)) == -1)
	{
		cout << "Failed to send CAN frame." << endl;
	}
}

//单个参数写入（通信类型18）
void motor_modechange(uint16_t index,uint8_t runmode,uint8_t motor_id, uint16_t master_id, int sock)
{
	txCanIdEx->mode = 0x12; 
	txCanIdEx->motor_id = motor_id; 
	txCanIdEx->res = 4; 
	txCanIdEx->data = master_id ; 
	TxMessage.can_dlc = 8; 
	for(uint8_t i=0;i<8;i++) 
	{ 
		TxMessage.data[i]=0; 
	}
	memcpy(&TxMessage.data[0],&index,2); 
	memcpy(&TxMessage.data[4],&runmode,1);
	if (write(sock, &TxMessage, sizeof(TxMessage)) == -1)
	{
		cout << "Failed to send CAN frame." << endl;
	}
}

void motor_write(uint16_t index,float ref ,uint8_t motor_id, uint16_t master_id, int sock)
{
	txCanIdEx->mode = 0x12; 
	txCanIdEx->motor_id = motor_id; 
	txCanIdEx->res = 4; 
	txCanIdEx->data = master_id ; 
	TxMessage.can_dlc = 8; 
	for(uint8_t i=0;i<8;i++) 
	{ 
		TxMessage.data[i]=0; 
	}
	memcpy(&TxMessage.data[0],&index,2); 
	memcpy(&TxMessage.data[4],&ref,4);
	if (write(sock, &TxMessage, sizeof(TxMessage)) == -1)
	{
		cout << "Failed to send CAN frame." << endl;
	}
}

//Current mode 电流模式
void current_mode_write(uint8_t motor_id, uint16_t master_id,float ref, int sock)
{
	motor_modechange(run_mode,3,motor_id,master_id, sock);		
	motor_enable(motor_id,master_id, sock);
	motor_write(iq_ref,ref,motor_id, master_id, sock);
}

// //Speed mode 速度模式
void speed_mode_write(uint8_t motor_id, uint16_t master_id,float current_limit,float ref, int sock)
{
	motor_modechange(run_mode,2,motor_id,master_id, sock);		
	motor_enable(motor_id,master_id, sock);
	motor_write(limit_cur,current_limit,motor_id, master_id, sock);
	motor_write(spd_ref,ref,motor_id, master_id, sock);
}

// //Location mode 位置模式
void location_mode_write(uint8_t motor_id, uint16_t master_id,float speed_limit,float ref, int sock)
{
	motor_modechange(run_mode,1,motor_id,master_id, sock);		
	motor_enable(motor_id,master_id,sock);
	motor_write(limit_spd,speed_limit,motor_id, master_id,sock);
	motor_write(loc_ref,ref,motor_id, master_id, sock);
}



