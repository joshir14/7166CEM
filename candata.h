/**
 * The MIT License (MIT)
 *
 * Copyright (c) 2018-2019 Erik Moqvist
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * This file was generated by cantools version 37.2.0 Tue Nov 14 16:48:17 2023.
 */

#ifndef CANDATA_H
#define CANDATA_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifndef EINVAL
#    define EINVAL 22
#endif

/* Frame ids. */
#define CANDATA_AI_DRIVE_REQUEST_FRAME_ID (0x521u)
#define CANDATA_VCU_WHEEL_SPEEDS_FRAME_ID (0x525u)
#define CANDATA_VCU_BATTERY_FRAME_ID (0x526u)
#define CANDATA_MOTOR_CURRENT_FRAME_ID (0x320u)

/* Frame lengths in bytes. */
#define CANDATA_AI_DRIVE_REQUEST_LENGTH (6u)
#define CANDATA_VCU_WHEEL_SPEEDS_LENGTH (8u)
#define CANDATA_VCU_BATTERY_LENGTH (2u)
#define CANDATA_MOTOR_CURRENT_LENGTH (4u)

/* Extended or standard frame types. */
#define CANDATA_AI_DRIVE_REQUEST_IS_EXTENDED (0)
#define CANDATA_VCU_WHEEL_SPEEDS_IS_EXTENDED (0)
#define CANDATA_VCU_BATTERY_IS_EXTENDED (0)
#define CANDATA_MOTOR_CURRENT_IS_EXTENDED (0)

/* Frame cycle times in milliseconds. */


/* Signal choices. */


/**
 * Signals in message AI_DRIVE_REQUEST.
 *
 * All signal values are as on the CAN bus.
 */
struct candata_ai_drive_request_t {
    /**
     * Range: -32768..32767 (-32768..32767 Nm)
     * Scale: 1
     * Offset: 0
     */
    int16_t front_trq_request;

    /**
     * Range: -35..35 (-0.35..0.35 rad)
     * Scale: 0.01
     * Offset: 0
     */
    int16_t steering_request;

    /**
     * Range: -32768..32767 (-32768..32767 Nm)
     * Scale: 1
     * Offset: 0
     */
    int16_t rear_trq_request;
};

/**
 * Signals in message VCU_WHEEL_SPEEDS.
 *
 * All signal values are as on the CAN bus.
 */
struct candata_vcu_wheel_speeds_t {
    /**
     * Range: 0..655350 (0..65535 rpm)
     * Scale: 0.1
     * Offset: 0
     */
    uint16_t fl_wheel_speed;

    /**
     * Range: 0..655350 (0..65535 rpm)
     * Scale: 0.1
     * Offset: 0
     */
    uint16_t fr_wheel_speed;

    /**
     * Range: 0..655350 (0..65535 rpm)
     * Scale: 0.1
     * Offset: 0
     */
    uint16_t rl_wheel_speed;

    /**
     * Range: 0..655350 (0..65535 rpm)
     * Scale: 0.1
     * Offset: 0
     */
    uint16_t rr_wheel_speed;
};

/**
 * Signals in message VCU_BATTERY.
 *
 * All signal values are as on the CAN bus.
 */
struct candata_vcu_battery_t {
    /**
     * Range: 0..13000 (0..13 V)
     * Scale: 0.001
     * Offset: 0
     */
    uint16_t voltage;
};

/**
 * Signals in message MOTOR_CURRENT.
 *
 * All signal values are as on the CAN bus.
 */
struct candata_motor_current_t {
    /**
     * Range: -30000..30000 (-30000..30000 mA)
     * Scale: 1
     * Offset: 0
     */
    int16_t front_current;

    /**
     * Range: -30000..30000 (-30000..30000 mA)
     * Scale: 1
     * Offset: 0
     */
    int16_t rear_current;
};

/**
 * Pack message AI_DRIVE_REQUEST.
 *
 * @param[out] dst_p Buffer to pack the message into.
 * @param[in] src_p Data to pack.
 * @param[in] size Size of dst_p.
 *
 * @return Size of packed data, or negative error code.
 */
int candata_ai_drive_request_pack(
    uint8_t *dst_p,
    const struct candata_ai_drive_request_t *src_p,
    size_t size);

/**
 * Unpack message AI_DRIVE_REQUEST.
 *
 * @param[out] dst_p Object to unpack the message into.
 * @param[in] src_p Message to unpack.
 * @param[in] size Size of src_p.
 *
 * @return zero(0) or negative error code.
 */
int candata_ai_drive_request_unpack(
    struct candata_ai_drive_request_t *dst_p,
    const uint8_t *src_p,
    size_t size);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
int16_t candata_ai_drive_request_front_trq_request_encode(float value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
float candata_ai_drive_request_front_trq_request_decode(int16_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool candata_ai_drive_request_front_trq_request_is_in_range(int16_t value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
int16_t candata_ai_drive_request_steering_request_encode(float value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
float candata_ai_drive_request_steering_request_decode(int16_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool candata_ai_drive_request_steering_request_is_in_range(int16_t value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
int16_t candata_ai_drive_request_rear_trq_request_encode(float value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
float candata_ai_drive_request_rear_trq_request_decode(int16_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool candata_ai_drive_request_rear_trq_request_is_in_range(int16_t value);

/**
 * Pack message VCU_WHEEL_SPEEDS.
 *
 * @param[out] dst_p Buffer to pack the message into.
 * @param[in] src_p Data to pack.
 * @param[in] size Size of dst_p.
 *
 * @return Size of packed data, or negative error code.
 */
int candata_vcu_wheel_speeds_pack(
    uint8_t *dst_p,
    const struct candata_vcu_wheel_speeds_t *src_p,
    size_t size);

/**
 * Unpack message VCU_WHEEL_SPEEDS.
 *
 * @param[out] dst_p Object to unpack the message into.
 * @param[in] src_p Message to unpack.
 * @param[in] size Size of src_p.
 *
 * @return zero(0) or negative error code.
 */
int candata_vcu_wheel_speeds_unpack(
    struct candata_vcu_wheel_speeds_t *dst_p,
    const uint8_t *src_p,
    size_t size);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint16_t candata_vcu_wheel_speeds_fl_wheel_speed_encode(float value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
float candata_vcu_wheel_speeds_fl_wheel_speed_decode(uint16_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool candata_vcu_wheel_speeds_fl_wheel_speed_is_in_range(uint16_t value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint16_t candata_vcu_wheel_speeds_fr_wheel_speed_encode(float value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
float candata_vcu_wheel_speeds_fr_wheel_speed_decode(uint16_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool candata_vcu_wheel_speeds_fr_wheel_speed_is_in_range(uint16_t value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint16_t candata_vcu_wheel_speeds_rl_wheel_speed_encode(float value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
float candata_vcu_wheel_speeds_rl_wheel_speed_decode(uint16_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool candata_vcu_wheel_speeds_rl_wheel_speed_is_in_range(uint16_t value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint16_t candata_vcu_wheel_speeds_rr_wheel_speed_encode(float value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
float candata_vcu_wheel_speeds_rr_wheel_speed_decode(uint16_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool candata_vcu_wheel_speeds_rr_wheel_speed_is_in_range(uint16_t value);

/**
 * Pack message VCU_BATTERY.
 *
 * @param[out] dst_p Buffer to pack the message into.
 * @param[in] src_p Data to pack.
 * @param[in] size Size of dst_p.
 *
 * @return Size of packed data, or negative error code.
 */
int candata_vcu_battery_pack(
    uint8_t *dst_p,
    const struct candata_vcu_battery_t *src_p,
    size_t size);

/**
 * Unpack message VCU_BATTERY.
 *
 * @param[out] dst_p Object to unpack the message into.
 * @param[in] src_p Message to unpack.
 * @param[in] size Size of src_p.
 *
 * @return zero(0) or negative error code.
 */
int candata_vcu_battery_unpack(
    struct candata_vcu_battery_t *dst_p,
    const uint8_t *src_p,
    size_t size);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint16_t candata_vcu_battery_voltage_encode(float value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
float candata_vcu_battery_voltage_decode(uint16_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool candata_vcu_battery_voltage_is_in_range(uint16_t value);

/**
 * Pack message MOTOR_CURRENT.
 *
 * @param[out] dst_p Buffer to pack the message into.
 * @param[in] src_p Data to pack.
 * @param[in] size Size of dst_p.
 *
 * @return Size of packed data, or negative error code.
 */
int candata_motor_current_pack(
    uint8_t *dst_p,
    struct candata_motor_current_t *src_p,
    size_t size);

/**
 * Unpack message MOTOR_CURRENT.
 *
 * @param[out] dst_p Object to unpack the message into.
 * @param[in] src_p Message to unpack.
 * @param[in] size Size of src_p.
 *
 * @return zero(0) or negative error code.
 */
int candata_motor_current_unpack(
    struct candata_motor_current_t *dst_p,
    const uint8_t *src_p,
    size_t size);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
int16_t candata_motor_current_front_current_encode(float value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
float candata_motor_current_front_current_decode(int16_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool candata_motor_current_front_current_is_in_range(int16_t value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
int16_t candata_motor_current_rear_current_encode(float value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
float candata_motor_current_rear_current_decode(int16_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool candata_motor_current_rear_current_is_in_range(int16_t value);

#include <string.h>

#include "candata.h"

static inline uint8_t pack_left_shift_u16(
    uint16_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint8_t)((uint8_t)(value << shift) & mask);
}

static inline uint8_t pack_right_shift_u16(
    uint16_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint8_t)((uint8_t)(value >> shift) & mask);
}

static inline uint16_t unpack_left_shift_u16(
    uint8_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint16_t)((uint16_t)(value & mask) << shift);
}

static inline uint16_t unpack_right_shift_u16(
    uint8_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint16_t)((uint16_t)(value & mask) >> shift);
}

int candata_ai_drive_request_pack(
    uint8_t *dst_p,
    const struct candata_ai_drive_request_t *src_p,
    size_t size)
{
    uint16_t front_trq_request;
    uint16_t rear_trq_request;
    uint16_t steering_request;

    if (size < 6u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 6);

    front_trq_request = (uint16_t)src_p->front_trq_request;
    dst_p[0] |= pack_left_shift_u16(front_trq_request, 0u, 0xffu);
    dst_p[1] |= pack_right_shift_u16(front_trq_request, 8u, 0xffu);
    steering_request = (uint16_t)src_p->steering_request;
    dst_p[2] |= pack_left_shift_u16(steering_request, 0u, 0xffu);
    dst_p[3] |= pack_right_shift_u16(steering_request, 8u, 0xffu);
    rear_trq_request = (uint16_t)src_p->rear_trq_request;
    dst_p[4] |= pack_left_shift_u16(rear_trq_request, 0u, 0xffu);
    dst_p[5] |= pack_right_shift_u16(rear_trq_request, 8u, 0xffu);

    return (6);
}

int candata_ai_drive_request_unpack(
    struct candata_ai_drive_request_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    uint16_t front_trq_request;
    uint16_t rear_trq_request;
    uint16_t steering_request;

    if (size < 6u) {
        return (-EINVAL);
    }

    front_trq_request = unpack_right_shift_u16(src_p[0], 0u, 0xffu);
    front_trq_request |= unpack_left_shift_u16(src_p[1], 8u, 0xffu);
    dst_p->front_trq_request = (int16_t)front_trq_request;
    steering_request = unpack_right_shift_u16(src_p[2], 0u, 0xffu);
    steering_request |= unpack_left_shift_u16(src_p[3], 8u, 0xffu);
    dst_p->steering_request = (int16_t)steering_request;
    rear_trq_request = unpack_right_shift_u16(src_p[4], 0u, 0xffu);
    rear_trq_request |= unpack_left_shift_u16(src_p[5], 8u, 0xffu);
    dst_p->rear_trq_request = (int16_t)rear_trq_request;

    return (0);
}

int16_t candata_ai_drive_request_front_trq_request_encode(float value)
{
    return (int16_t)(value);
}

float candata_ai_drive_request_front_trq_request_decode(int16_t value)
{
    return ((float)value);
}

bool candata_ai_drive_request_front_trq_request_is_in_range(int16_t value)
{
    (void)value;

    return (true);
}

int16_t candata_ai_drive_request_steering_request_encode(float value)
{
    return (int16_t)(value / 0.01);
}

float candata_ai_drive_request_steering_request_decode(int16_t value)
{
    return ((float)value * 0.01);
}

bool candata_ai_drive_request_steering_request_is_in_range(int16_t value)
{
    return ((value >= -35) && (value <= 35));
}

int16_t candata_ai_drive_request_rear_trq_request_encode(float value)
{
    return (int16_t)(value);
}

float candata_ai_drive_request_rear_trq_request_decode(int16_t value)
{
    return ((float)value);
}

bool candata_ai_drive_request_rear_trq_request_is_in_range(int16_t value)
{
    (void)value;

    return (true);
}

int candata_vcu_wheel_speeds_pack(
    uint8_t *dst_p,
    const struct candata_vcu_wheel_speeds_t *src_p,
    size_t size)
{
    if (size < 8u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 8);

    dst_p[0] |= pack_left_shift_u16(src_p->fl_wheel_speed, 0u, 0xffu);
    dst_p[1] |= pack_right_shift_u16(src_p->fl_wheel_speed, 8u, 0xffu);
    dst_p[2] |= pack_left_shift_u16(src_p->fr_wheel_speed, 0u, 0xffu);
    dst_p[3] |= pack_right_shift_u16(src_p->fr_wheel_speed, 8u, 0xffu);
    dst_p[4] |= pack_left_shift_u16(src_p->rl_wheel_speed, 0u, 0xffu);
    dst_p[5] |= pack_right_shift_u16(src_p->rl_wheel_speed, 8u, 0xffu);
    dst_p[6] |= pack_left_shift_u16(src_p->rr_wheel_speed, 0u, 0xffu);
    dst_p[7] |= pack_right_shift_u16(src_p->rr_wheel_speed, 8u, 0xffu);

    return (8);
}

int candata_vcu_wheel_speeds_unpack(
    struct candata_vcu_wheel_speeds_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    if (size < 8u) {
        return (-EINVAL);
    }

    dst_p->fl_wheel_speed = unpack_right_shift_u16(src_p[0], 0u, 0xffu);
    dst_p->fl_wheel_speed |= unpack_left_shift_u16(src_p[1], 8u, 0xffu);
    dst_p->fr_wheel_speed = unpack_right_shift_u16(src_p[2], 0u, 0xffu);
    dst_p->fr_wheel_speed |= unpack_left_shift_u16(src_p[3], 8u, 0xffu);
    dst_p->rl_wheel_speed = unpack_right_shift_u16(src_p[4], 0u, 0xffu);
    dst_p->rl_wheel_speed |= unpack_left_shift_u16(src_p[5], 8u, 0xffu);
    dst_p->rr_wheel_speed = unpack_right_shift_u16(src_p[6], 0u, 0xffu);
    dst_p->rr_wheel_speed |= unpack_left_shift_u16(src_p[7], 8u, 0xffu);

    return (0);
}

uint16_t candata_vcu_wheel_speeds_fl_wheel_speed_encode(float value)
{
    return (uint16_t)(value / 0.1);
}

float candata_vcu_wheel_speeds_fl_wheel_speed_decode(uint16_t value)
{
    return ((float)value * 0.1);
}

bool candata_vcu_wheel_speeds_fl_wheel_speed_is_in_range(uint16_t value)
{
    (void)value;

    return (true);
}

uint16_t candata_vcu_wheel_speeds_fr_wheel_speed_encode(float value)
{
    return (uint16_t)(value / 0.1);
}

float candata_vcu_wheel_speeds_fr_wheel_speed_decode(uint16_t value)
{
    return ((float)value * 0.1);
}

bool candata_vcu_wheel_speeds_fr_wheel_speed_is_in_range(uint16_t value)
{
    (void)value;

    return (true);
}

uint16_t candata_vcu_wheel_speeds_rl_wheel_speed_encode(float value)
{
    return (uint16_t)(value / 0.1);
}

float candata_vcu_wheel_speeds_rl_wheel_speed_decode(uint16_t value)
{
    return ((float)value * 0.1);
}

bool candata_vcu_wheel_speeds_rl_wheel_speed_is_in_range(uint16_t value)
{
    (void)value;

    return (true);
}

uint16_t candata_vcu_wheel_speeds_rr_wheel_speed_encode(float value)
{
    return (uint16_t)(value / 0.1);
}

float candata_vcu_wheel_speeds_rr_wheel_speed_decode(uint16_t value)
{
    return ((float)value * 0.1);
}

bool candata_vcu_wheel_speeds_rr_wheel_speed_is_in_range(uint16_t value)
{
    (void)value;

    return (true);
}

int candata_vcu_battery_pack(
    uint8_t *dst_p,
    const struct candata_vcu_battery_t *src_p,
    size_t size)
{
    if (size < 2u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 2);

    dst_p[0] |= pack_left_shift_u16(src_p->voltage, 0u, 0xffu);
    dst_p[1] |= pack_right_shift_u16(src_p->voltage, 8u, 0xffu);

    return (2);
}

int candata_vcu_battery_unpack(
    struct candata_vcu_battery_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    if (size < 2u) {
        return (-EINVAL);
    }

    dst_p->voltage = unpack_right_shift_u16(src_p[0], 0u, 0xffu);
    dst_p->voltage |= unpack_left_shift_u16(src_p[1], 8u, 0xffu);

    return (0);
}

uint16_t candata_vcu_battery_voltage_encode(float value)
{
    return (uint16_t)(value / 0.001);
}

float candata_vcu_battery_voltage_decode(uint16_t value)
{
    return ((float)value * 0.001);
}

bool candata_vcu_battery_voltage_is_in_range(uint16_t value)
{
    return (value <= 13000u);
}

int candata_motor_current_pack(
    uint8_t *dst_p,
    struct candata_motor_current_t *src_p,
    size_t size)
{
    uint16_t front_current;
    uint16_t rear_current;

    if (size < 4u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 4);

    front_current = (uint16_t)src_p->front_current;
    dst_p[0] |= pack_left_shift_u16(front_current, 0u, 0xffu);
    dst_p[1] |= pack_right_shift_u16(front_current, 8u, 0xffu);
    rear_current = (uint16_t)src_p->rear_current;
    dst_p[2] |= pack_left_shift_u16(rear_current, 0u, 0xffu);
    dst_p[3] |= pack_right_shift_u16(rear_current, 8u, 0xffu);

    return (4);
}

int candata_motor_current_unpack(
    struct candata_motor_current_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    uint16_t front_current;
    uint16_t rear_current;

    if (size < 4u) {
        return (-EINVAL);
    }

    front_current = unpack_right_shift_u16(src_p[0], 0u, 0xffu);
    front_current |= unpack_left_shift_u16(src_p[1], 8u, 0xffu);
    dst_p->front_current = (int16_t)front_current;
    rear_current = unpack_right_shift_u16(src_p[2], 0u, 0xffu);
    rear_current |= unpack_left_shift_u16(src_p[3], 8u, 0xffu);
    dst_p->rear_current = (int16_t)rear_current;

    return (0);
}

int16_t candata_motor_current_front_current_encode(float value)
{
    return (int16_t)(value);
}

float candata_motor_current_front_current_decode(int16_t value)
{
    return ((float)value);
}

bool candata_motor_current_front_current_is_in_range(int16_t value)
{
    return ((value >= -30000) && (value <= 30000));
}

int16_t candata_motor_current_rear_current_encode(float value)
{
    return (int16_t)(value);
}

float candata_motor_current_rear_current_decode(int16_t value)
{
    return ((float)value);
}

bool candata_motor_current_rear_current_is_in_range(int16_t value)
{
    return ((value >= -30000) && (value <= 30000));
}



#ifdef __cplusplus
}
#endif

#endif
