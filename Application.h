#include "can_wrap.h"
#include "candata.h"

/**
 * Processes VCU_BATTERY message and stores battery voltage.
 *
 * @param[in] frame CAN frame received over CAN bus.
 * @param[in] batteryVoltage Pointer to varibale indicating battery voltage.
 * 
 * @return 0 or negative error code
 */
int vcu_battery_frame_process(struct can_frame frame, float* batteryVoltage);

/**
 * Processes AI_DRIVE_REQUESTS message and stores torque values.
 *
 * @param[in] frame CAN frame received over CAN bus.
 * @param[in] frontTrq Pointer to variable indicating front drive torque
 * @param[in] rearTrq Pointer to variable indicating rear drive torque
 * @param[in] steeringReq Pointer to variable indicating steering request.
 * 
 * @return 0 or negative error code
 */
int ai_drive_request_frame_process(struct can_frame frame, float* frontTrq, float* rearTrq, float* steeringReq);

/**
 * Processes VCU_WHEEL_SPEEDS message and stores wheel speed values.
 *
 * @param[in] frame CAN frame received over CAN bus.
 * @param[in] rr_wheelSpeed Pointer to variable indicating rear right wheel speed.
 * @param[in] rl_wheelSpeed Pointer to variable indicating rear left wheel speed.
 * @param[in] fr_wheelSpeed Pointer to variable indicating front right wheel speed.
 * @param[in] fl_wheelSpeed Pointer to variable indicating front left wheel speed.
 * 
 * @return 0 or negative error code
 */
int vcu_wheel_speed_frame_process(struct can_frame frame, uint16_t* rr_wheelSpeed, uint16_t* rl_wheelSpeed, uint16_t* fr_wheelSpeed, uint16_t* fl_wheelSpeed);

/**
 * Genrates data frame to be transmitted for MOTOR_CURRENT (0x320) frame.
 *
 * @param[in] frontCurrent Calculated front drive current from calculate_front_current function.
 * @param[in] rearCurrent Calculated rear drive current from calculate_rear_current function.
 * @param[in] currentBuffer Pointer to data buffer to be filled with generated data.
 * 
 * @return Size of genrated data or negative error code.
 */
int process_motor_current_request(float frontCurrent, float rearCurrent, uint8_t* currentDataBuffer);

/**
 * Calculate rear torque limit.
 *
 * @param[in] rr_wheelSpeed Rear right wheel RPM received over CAN bus.
 * @param[in] rl_wheelSpeed Rear left wheel RPM received over CAN bus.
 * @param[in] rearTrq Rear drive torque request received over CAN bus.
 * @param[in] batteryVoltage Battery voltage received over CAN bus.
 * 
 * @return Calculated rear torque limit.
 */
int torque_limit_rear(uint16_t rr_wheelSpeed, uint16_t rl_wheelSpeed, float rearTrq, float batteryVoltage);

/**
 * Calculate front torque limit.
 *
 * @param[in] fr_wheelSpeed Front right wheel RPM received over CAN bus.
 * @param[in] fl_wheelSpeed Front left wheel RPM received over CAN bus.
 * @param[in] frontTrq Front drive torque request received over CAN bus.
 * @param[in] batteryVoltage Battery voltage received over CAN bus.
 * 
 * @return Calculated front torque limit.
 */
int torque_limit_front(uint16_t fr_wheelSpeed, uint16_t fl_wheelSpeed, float frontTrq, float batteryVoltage);

/**
 * Calculate front current to be transmitted on CAN message 0x320
 *
 * @param[in] tf Maximum Front torque limit.
 * @param[in] v Battery Voltage received over CAN bus.
 * @param[in] rf Maximum RPM value between front right and front left wheel speed in RPM.
 * 
 * @return Calculated front current.
 */
float calculate_front_current(int tf, float v, uint16_t rf);

/**
 * Calculate rear current to be transmitted on CAN message 0x320
 *
 * @param[in] tr Maximum Rear torque limit.
 * @param[in] v Battery Voltage received over CAN bus.
 * @param[in] rr Maximum RPM value between rear right and rear left wheel speed in RPM.
 * 
 * @return Calculated rear current.
 */
float calculate_rear_current(int tr, float v, uint16_t rr);


/**
 * Generate CAN frame to be transmitted.
 *
 * @param[in] frame Pointer to CAN frame to be modified.
 * @param[in] currentDataBuffer Data to be added in CAN frame.
 * @param[in] can_id CAN ID of message.
 * @param[in] dlc Length of data in CAN message.
 * 
 * @return 1 if frame is generated, 0 if frame is not genrated.
 */
 uint8_t generate_can_frame(struct can_frame* frame, uint8_t *currentDataBuffer, uint16_t can_id, int dlc);

float calculate_front_current(int tf, float v, uint16_t rf)
{
	float cf;
	if(v != 0)
	{
		cf = (tf/v) * (1 + (0.879 * 0.879 * (tf/100)) + (rf * (0.6729/100) * tf));
		if(cf > 30000)
		{
			cf = 30000;
		}
		if(cf < -30000)
		{
			cf = -30000;
		}
	}
	else
	{
		cf = 0;
	}
	return cf;
}

float calculate_rear_current(int tr, float v, uint16_t rr)
{
	float cr;
	if( v != 0.0f)
	{
		cr = (tr/v) * (1 + (0.879 * 0.879 * (tr/100)) + (rr * (0.6729/100) * tr));
		if(cr > 30000)
		{
			cr = 30000;
		}
		if(cr < -30000)
		{
			cr = -30000;
		}
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

int process_motor_current_request(float frontCurrent, float rearCurrent, uint8_t* currentDataBuffer)
{
	int dlc;
	struct candata_motor_current_t currentData;
	currentData.front_current = candata_motor_current_front_current_encode(frontCurrent);
	currentData.rear_current = candata_motor_current_rear_current_encode(rearCurrent);
	dlc = candata_motor_current_pack(currentDataBuffer, &currentData, 4);
	return dlc;
}

int ai_drive_request_frame_process(struct can_frame frame, float* frontTrq, float* rearTrq, float* steeringReq)
{
	struct candata_ai_drive_request_t driveRequest;
	int code = candata_ai_drive_request_unpack(&driveRequest, frame.data, frame.can_dlc);
	if(code == 0)
	{
		*frontTrq = candata_ai_drive_request_front_trq_request_decode(driveRequest.front_trq_request);
		*rearTrq = candata_ai_drive_request_rear_trq_request_decode(driveRequest.rear_trq_request);
		*steeringReq = candata_ai_drive_request_steering_request_decode(driveRequest.steering_request);
	}
	return code;
}

int vcu_battery_frame_process(struct can_frame frame, float* batteryVoltage)
{
	struct candata_vcu_battery_t VBATT;
	int code = candata_vcu_battery_unpack(&VBATT, frame.data, frame.can_dlc);
	if(code == 0)
	{
		*batteryVoltage = candata_vcu_battery_voltage_decode(VBATT.voltage);
	}
	return code;
}

int vcu_wheel_speed_frame_process(struct can_frame frame, uint16_t* rr_wheelSpeed, uint16_t* rl_wheelSpeed, uint16_t* fr_wheelSpeed, uint16_t* fl_wheelSpeed)
{
	struct candata_vcu_wheel_speeds_t wheelSpeed;
	int code = candata_vcu_wheel_speeds_unpack(&wheelSpeed, frame.data, frame.can_dlc);
	if(code == 0)
	{
		*rr_wheelSpeed = candata_vcu_wheel_speeds_rr_wheel_speed_decode(wheelSpeed.rr_wheel_speed);
		*rl_wheelSpeed = candata_vcu_wheel_speeds_rl_wheel_speed_decode(wheelSpeed.rl_wheel_speed);
		*fr_wheelSpeed = candata_vcu_wheel_speeds_fr_wheel_speed_decode(wheelSpeed.fr_wheel_speed);
		*fl_wheelSpeed = candata_vcu_wheel_speeds_fl_wheel_speed_decode(wheelSpeed.fl_wheel_speed);
	}
	return code;
}

uint8_t generate_can_frame(struct can_frame* frame, uint8_t *currentDataBuffer, uint16_t can_id, int dlc)
{
	frame->can_id = can_id;
	frame->can_dlc = dlc;
	if(dlc > 0)
	{
		for (int i = 0; i < dlc; i++)
		{
			frame->data[i] = currentDataBuffer[i];
		}
		return 1;
	}
	else
	{
		return 0;
	}
}
