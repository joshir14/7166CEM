#include "Application.h"

void copyData(struct can_frame* frame, uint8_t *currentDataBuffer, int dlc);
int dlc;


int main()
{
	const int socket_id = can_connect("vcan0",0);
	float cf1, cr1;
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
			cf1 = calculate_front_current(tf1, batteryVoltage, 300);
			cr1 = calculate_rear_current(tr1, batteryVoltage, 400);
			printf("Rear current %f\n", cr1);
			printf("Front current %f\n", cf1);
			dlc = process_motor_current_request(4.5, &currentData, 6.77,  currentDataBuffer);
			printf("DLC is %d\n", dlc);
			//currentData->front_current = candata_motor_current_front_current_encode(7.8);
			
			generate_can_frame(&TxFrame, currentDataBuffer, 0x320, dlc);
			if(!can_write(socket_id, &TxFrame))
			{
				printf("Error\n");
			}
		}
	}
}




