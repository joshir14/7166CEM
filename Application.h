#include "can_wrap.h"
#include "candata.h"

struct candata_vcu_battery_t VBATT;
struct can_frame frame, TxFrame;
float batteryVoltage;
struct candata_vcu_wheel_speeds_t wheelSpeed;
struct candata_ai_drive_request_t driveRequest;
uint16_t rr_wheelSpeed, rl_wheelSpeed, fr_wheelSpeed, fl_wheelSpeed;
float frontCurrent, rearCurrent;
float steeringReq, rearTrq, frontTrq;
struct candata_motor_current_t currentData;
uint8_t currentDataBuffer[8];

void vcu_battery_frame_process(struct candata_vcu_battery_t* VBATT, struct can_frame frame, float* batteryVoltage);
void ai_drive_request_frame_process(struct candata_ai_drive_request_t* driveRequest, struct can_frame frame, float* frontTrq, float* rearTrq, float* steeringReq);
void vcu_wheel_speed_frame_process(struct candata_vcu_wheel_speeds_t* wheelSpeed, struct can_frame frame, uint16_t* rr_wheelSpeed, uint16_t* rl_wheelSpeed, uint16_t* fr_wheelSpeed, uint16_t* fl_wheelSpeed);
int process_motor_current_request(float frontCurrent, struct candata_motor_current_t* currentData, float rearCurrent, uint8_t* currentDataBuffer);
int torque_limit_rear(uint16_t rr_wheelSpeed, uint16_t rl_wheelSpeed, float rearTrq, float batteryVoltage);
int torque_limit_front(uint16_t fr_wheelSpeed, uint16_t fl_wheelSpeed, float frontTrq, float batteryVoltage);
float calculate_front_current(int tr, float v, uint16_t r);
float calculate_rear_current(int tr, float v, uint16_t r);

float calculate_front_current(int tf, float v, uint16_t rf)
{
	float cf;
	if(v != 0.0f)
	{
		cf = (tf/v) * (1 + (0.879 * 0.879 * (tf/100)) + (rf * (0.6729/100) * tf));
	}
	else
	{
		cf = 0.0f;
	}
	return cf;
}

float calculate_rear_current(int tr, float v, uint16_t rr)
{
	float cr;
	if( v != 0.0f)
	{
		cr = (tr/v) * (1 + (0.879 * 0.879 * (tr/100)) + (rr * (0.6729/100) * tr));
	}
	else
	{
		cr = 0.0f;
	}
	return cr;
}

int torque_limit_front(uint16_t fr_wheelSpeed, uint16_t fl_wheelSpeed, float frontTrq, float batteryVoltage)
{
	int tf,tf1;
	int x = (fr_wheelSpeed >= fl_wheelSpeed) ? fr_wheelSpeed:fl_wheelSpeed;
	if(batteryVoltage > 2.8)
	{
		if(x > 700)
		{
			tf = 50;
		}
		else if((x > 600) && (x <= 700))
		{
			tf = 85;
		}
		else if((x > 500) && (x <= 600))
		{
			tf = 100;
		}
		else if((x > 400) && (x <= 500))
		{
			tf = 120;
		}
		else if((x > 300) && (x <= 400))
		{
			tf = 150;
		}
		else
		{
			tf = (int)(frontTrq);
		}
	}
	else
	{
		tf = 20;
	}
	tf1 = (tf <= (int)(frontTrq)) ? tf:(int)(frontTrq);
	return tf1;
}

int torque_limit_rear(uint16_t rr_wheelSpeed, uint16_t rl_wheelSpeed, float rearTrq, float batteryVoltage)
{
	int tr,tr1;
	int x = (rr_wheelSpeed >= rl_wheelSpeed) ? rr_wheelSpeed:rl_wheelSpeed;
	if(batteryVoltage > 2.8)
	{
		if(x > 700)
		{
			tr = 50;
		}
		else if((x > 600) && (x <= 700))
		{
			tr = 85;
		}
		else if((x > 500) && (x <= 600))
		{
			tr = 100;
		}
		else if((x > 400) && (x <= 500))
		{
			tr = 120;
		}
		else if((x > 300) && (x <= 400))
		{
			tr = 150;
		}
		else
		{
			tr = (int)(rearTrq);
		}
	}
	else
	{
		tr = 20;
	}
	tr1 = (tr <= (int)(rearTrq)) ? tr:(int)(rearTrq);
	return tr1;
}

int process_motor_current_request(float frontCurrent, struct candata_motor_current_t* currentData, float rearCurrent, uint8_t* currentDataBuffer)
{
	int dlc;
	currentData->front_current = candata_motor_current_front_current_encode(frontCurrent);
	currentData->rear_current = candata_motor_current_rear_current_encode(rearCurrent);
	dlc = candata_motor_current_pack(currentDataBuffer, currentData, 4);
	return dlc;
}

void ai_drive_request_frame_process(struct candata_ai_drive_request_t* driveRequest, struct can_frame frame, float* frontTrq, float* rearTrq, float* steeringReq)
{
	if(candata_ai_drive_request_unpack(driveRequest, frame.data, frame.can_dlc)==0)
	{
		*frontTrq = candata_ai_drive_request_front_trq_request_decode(driveRequest->front_trq_request);
		*rearTrq = candata_ai_drive_request_rear_trq_request_decode(driveRequest->rear_trq_request);
		*steeringReq = candata_ai_drive_request_steering_request_decode(driveRequest->steering_request);
	}
}

void vcu_battery_frame_process(struct candata_vcu_battery_t *VBATT, struct can_frame frame, float* batteryVoltage)
{
	
	if(candata_vcu_battery_unpack(VBATT, frame.data, frame.can_dlc) == 0)
	{
		*batteryVoltage = candata_vcu_battery_voltage_decode(VBATT->voltage);
	}
}

void vcu_wheel_speed_frame_process(struct candata_vcu_wheel_speeds_t* wheelSpeed, struct can_frame frame, uint16_t* rr_wheelSpeed, uint16_t* rl_wheelSpeed, uint16_t* fr_wheelSpeed, uint16_t* fl_wheelSpeed)
{
	if(candata_vcu_wheel_speeds_unpack(wheelSpeed, frame.data, frame.can_dlc)== 0)
	{
		*rr_wheelSpeed = candata_vcu_wheel_speeds_rr_wheel_speed_decode(wheelSpeed->rr_wheel_speed);
		*rl_wheelSpeed = candata_vcu_wheel_speeds_rl_wheel_speed_decode(wheelSpeed->rl_wheel_speed);
		*fr_wheelSpeed = candata_vcu_wheel_speeds_fr_wheel_speed_decode(wheelSpeed->fr_wheel_speed);
		*fl_wheelSpeed = candata_vcu_wheel_speeds_fl_wheel_speed_decode(wheelSpeed->fl_wheel_speed);
	}
}

