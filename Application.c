#include "Application.h"

int16_t calculateCurrent(float batteryVoltage);

int main()
{
	
	const int socket_id = can_connect("vcan0",0);
	int16_t t;
	while(1)
	{
		if(can_read(socket_id,&frame)==1)
		{
			if((frame.can_id == 0x526) && (frame.can_dlc > 0))
			{
				vcu_battery_frame_process(&VBATT, frame, &batteryVoltage);
				printf("Battery voltage is %f\n", batteryVoltage);
			}
			else if((frame.can_id == 0x521) && (frame.can_dlc > 0))
			{
				ai_drive_request_frame_process(&driveRequest, frame, &frontTrq, &rearTrq, &steeringReq);
				printf("Front torque is %f\n", frontTrq);
				printf("Rear torque is %f\n", rearTrq);
				printf("Steering request is %f\n", steeringReq);
			}
			else if((frame.can_id == 0x525) && (frame.can_dlc > 0))
			{
				vcu_wheel_speed_frame_process(&wheelSpeed, frame, &rr_wheelSpeed, &rl_wheelSpeed, &fr_wheelSpeed, &fl_wheelSpeed);
				printf("Front right wheel speed is %d\n", fr_wheelSpeed);
				printf("Rear right wheel speed is %d\n", rr_wheelSpeed);
				printf("Rear left wheel speed is %d\n", rl_wheelSpeed);
				printf("Front left wheel speed is %d\n", fl_wheelSpeed);
			}
			else
			{
				
			}
			t = calculateCurrent(batteryVoltage);
			printf("Torque limited to %d\n", t);
		}
	}
}

int16_t calculateCurrent(float batteryVoltage)
{
	if(batteryVoltage <= 2.8)
	{
		return 20;
	}
}

