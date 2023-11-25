#include "Application.h"

void copyData(struct can_frame* frame, uint8_t *currentDataBuffer, int dlc);
int dlc;


int main()
{
	const int socket_id = can_connect("vcan0",0);
	float cf1, cr1;
	int tf1, tr1;
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
			tf1 = torque_limit_front(fr_wheelSpeed, fl_wheelSpeed, frontTrq, batteryVoltage);
			tr1 = torque_limit_rear(rr_wheelSpeed, rl_wheelSpeed, rearTrq, batteryVoltage);
			dlc = process_motor_current_request(4.5, &currentData, 6.77,  currentDataBuffer);
			printf("DLC is %d\n", dlc);
			//currentData->front_current = candata_motor_current_front_current_encode(7.8);
			
			TxFrame.can_id = 0x320;
			TxFrame.can_dlc = dlc;
			//TxFrame.data[0] = currentDataBuffer[0];
			//TxFrame.data[1] = currentDataBuffer[1];
			//TxFrame.data[2] = currentDataBuffer[2];
			//TxFrame.data[3] = currentDataBuffer[3];
			copyData(&TxFrame, currentDataBuffer, dlc);
			if(!can_write(socket_id, &TxFrame))
			{
				printf("Error\n");
			}
		}
	}
}

void copyData(struct can_frame* frame, uint8_t *currentDataBuffer, int dlc)
{
	if(dlc > 0)
	{
		for (int i = 0; i < dlc; i++)
		{
			frame->data[i] = currentDataBuffer[i];
		}
	}	
}


