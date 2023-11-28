#include "Application.h"

int main()
{
	const int socket_id = can_connect("vcan0",0);
	float cf1, cr1;
	uint8_t count1, count2, count3;
	struct can_frame frame, TxFrame;
	float batteryVoltage;
	int dlc;
	uint16_t rr_wheelSpeed, rl_wheelSpeed, fr_wheelSpeed, fl_wheelSpeed;
	float frontCurrent, rearCurrent;
	float steeringReq, rearTrq, frontTrq;

	uint8_t currentDataBuffer[8];
	int tf1, tr1;
	while(1)
	{
		if(can_read(socket_id,&frame) == 1)
		{
			if((frame.can_id == 0x526) && (frame.can_dlc > 0))
			{
				count1 = count1 + 1;
				vcu_battery_frame_process(frame, &batteryVoltage);
				printf("Battery voltage is %f\n", batteryVoltage);
			}
			else if((frame.can_id == 0x521) && (frame.can_dlc > 0))
			{
				count2 = count2 + 1;
				ai_drive_request_frame_process(frame, &frontTrq, &rearTrq, &steeringReq);
				printf("Front torque is %f\n", frontTrq);
				printf("Rear torque is %f\n", rearTrq);
				printf("Steering request is %f\n", steeringReq);
			}
			else if((frame.can_id == 0x525) && (frame.can_dlc > 0))
			{
				count3 = count3 + 1;
				vcu_wheel_speed_frame_process(frame, &rr_wheelSpeed, &rl_wheelSpeed, &fr_wheelSpeed, &fl_wheelSpeed);
				printf("Front right wheel speed is %d\n", fr_wheelSpeed);
				printf("Rear right wheel speed is %d\n", rr_wheelSpeed);
				printf("Rear left wheel speed is %d\n", rl_wheelSpeed);
				printf("Front left wheel speed is %d\n", fl_wheelSpeed);
			}

			if((count1 > 0) && (count2 > 0) && (count3 > 0))
			{
				count1 = 0;
				count2 = 0;
				count3 = 0;
				tf1 = torque_limit_front(fr_wheelSpeed, fl_wheelSpeed, frontTrq, batteryVoltage);
				tr1 = torque_limit_rear(rr_wheelSpeed, rl_wheelSpeed, rearTrq, batteryVoltage);
				printf("Front torque limit %d\n", tf1);
				printf("Rear torque limit %d\n", tr1);
				cf1 = calculate_front_current(tf1, batteryVoltage, 300);
				cr1 = calculate_rear_current(tr1, batteryVoltage, 400);
				printf("Rear current %f\n", cr1);
				printf("Front current %f\n", cf1);
				dlc = process_motor_current_request(cf1, cr1, currentDataBuffer);
				printf("DLC is %d\n", dlc);
				
				generate_can_frame(&TxFrame, currentDataBuffer, 0x320, dlc);
				if(!can_write(socket_id, &TxFrame))
				{
					printf("Error\n");
				}
			}
		}
	}
}




