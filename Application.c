#include "can_wrap.h"
#include "candata.h"

void vcu_battery_frame_process(struct candata_vcu_battery_t* VBATT, struct can_frame frame, float* batteryVoltage);
void ai_drive_request_frame_process(struct candata_ai_drive_request_t* driveRequest, struct can_frame frame, double* frontTrq, double* rearTrq, double* steeringReq);
void vcu_wheel_speed_frame_process(struct candata_vcu_wheel_speeds_t* wheelSpeed, struct can_frame frame, uint16_t* rr_wheelSpeed, uint16_t* rl_wheelSpeed, uint16_t* fr_wheelSpeed, uint16_t* fl_wheelSpeed);

int main()
{
	struct candata_vcu_battery_t VBATT;
	struct can_frame frame;
	float batteryVoltage;
	struct candata_vcu_wheel_speeds_t wheelSpeed;
	double frontTrq;
	struct candata_ai_drive_request_t driveRequest;
	double rr_wheelSpeed, rl_wheelSpeed, fr_wheelSpeed, fl_wheelSpeed;
	double rearTrq;
	double steeringReq;
	const int socket_id = can_connect("vcan0",0);
	while(1)
	{
		if(can_read(socket_id,&frame)==1)
		{
			if((frame.can_id == 0x526) && (frame.can_dlc > 0))
			{
				vcu_battery_frame_process(&VBATT, frame, &batteryVoltage);
				//printf("Raw data is %d\n", VBATT.voltage);
				printf("Battery voltage is %f\n", batteryVoltage);
			}
			else if((frame.can_id == 0x521) && (frame.can_dlc > 0))
			{
				ai_drive_request_frame_process(&driveRequest, frame, &frontTrq, &rearTrq, &steeringReq);
				//printf("Raw data is %d\n", driveRequest.front_trq_request);
				printf("Front torque is %f\n", frontTrq);
				printf("Rear torque is %f\n", rearTrq);
				printf("Steering request is %f\n", steeringReq);
			}
			else if((frame.can_id == 0x521) && (frame.can_dlc > 0))
			{
				
			}
			else
			{
				
			}
		}
	}
}

void ai_drive_request_frame_process(struct candata_ai_drive_request_t* driveRequest, struct can_frame frame, double* frontTrq, double* rearTrq, double* steeringReq)
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
