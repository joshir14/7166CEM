#include "can_wrap.h"
#include "candata.h"

struct candata_vcu_battery_t VBATT;
struct can_frame frame;
float batteryVoltage;
struct candata_vcu_wheel_speeds_t wheelSpeed;
float frontTrq;
struct candata_ai_drive_request_t driveRequest;
uint16_t rr_wheelSpeed, rl_wheelSpeed, fr_wheelSpeed, fl_wheelSpeed;
float rearTrq;
float steeringReq;

void vcu_battery_frame_process(struct candata_vcu_battery_t* VBATT, struct can_frame frame, float* batteryVoltage);
void ai_drive_request_frame_process(struct candata_ai_drive_request_t* driveRequest, struct can_frame frame, float* frontTrq, float* rearTrq, float* steeringReq);
void vcu_wheel_speed_frame_process(struct candata_vcu_wheel_speeds_t* wheelSpeed, struct can_frame frame, uint16_t* rr_wheelSpeed, uint16_t* rl_wheelSpeed, uint16_t* fr_wheelSpeed, uint16_t* fl_wheelSpeed);

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

