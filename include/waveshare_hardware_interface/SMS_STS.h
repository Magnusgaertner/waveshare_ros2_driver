#pragma once

//-------EPROM(read only)--------
#define SMS_STS_FIRMWARE_VER_L 0
#define SMS_STS_FIRMWARE_VER_H 1
#define SMS_STS_MODEL_L 3
#define SMS_STS_MODEL_H 4

//-------EPROM(read/write)--------
#define SMS_STS_ID 5
#define SMS_STS_BAUD_RATE 6
#define SMS_STS_RETURN_DELAY 7
#define SMS_STS_RESPONSE_STATUS_LEVEL 8
#define SMS_STS_MIN_ANGLE_LIMIT_L 9
#define SMS_STS_MIN_ANGLE_LIMIT_H 10
#define SMS_STS_MAX_ANGLE_LIMIT_L 11
#define SMS_STS_MAX_ANGLE_LIMIT_H 12
#define SMS_STS_MAX_TEMPERATURE_LIMIT 13
#define SMS_STS_MAX_INPUT_VOLT 14
#define SMS_STS_MIN_INPUT_VOLT 15
#define SMS_STS_MAX_TORQUE_L 16
#define SMS_STS_MAX_TORQUE_H 17
#define SMS_STS_PHASE 18
#define SMS_STS_UNLOADING_CONDITION 19
#define SMS_STS_LED_ALARM_CONDITION 20
#define SMS_STS_P_COEF 21
#define SMS_STS_D_COEF 22
#define SMS_STS_I_COEF 23
#define SMS_STS_MINIMUM_STARTUP_FORCE_L 24
#define SMS_STS_MINIMUM_STARTUP_FORCE_H 25

#define SMS_STS_CW_DEAD 26   // CW: Clockwise
#define SMS_STS_CCW_DEAD 27  // CCW: CounterClockwise

#define SMS_STS_PROTECTION_CURRENT_L 28
#define SMS_STS_PROTECTION_CURRENT_H 29
#define SMS_STS_ANGULAR_RESOLUTION 30

#define SMS_STS_OFS_L 31  // Position Correction
#define SMS_STS_OFS_H 32
#define SMS_STS_MODE 33

#define SMS_STS_PROTECTIVE_TORQUE 34
#define SMS_STS_PROTECTION_TIME 35
#define SMS_STS_OVERLOAD_TORQUE 36
#define SMS_STS_SPEED_CLOSED_LOOP_P_COEF 37
#define SMS_STS_OVER_CURRENT_PROTECTION_TIME 38
#define SMS_STS_VELOCITY_CLOSED_LOOP_I_COEF 39

//-------SRAM(read/write)--------
#define SMS_STS_TORQUE_ENABLE 40
#define SMS_STS_ACC 41
#define SMS_STS_GOAL_POSITION_L 42
#define SMS_STS_GOAL_POSITION_H 43
#define SMS_STS_GOAL_TIME_L 44
#define SMS_STS_GOAL_TIME_H 45
#define SMS_STS_GOAL_SPEED_L 46

#define SMS_STS_GOAL_SPEED_H 47
#define SMS_STS_TORQUE_LIMIT_L 48
#define SMS_STS_TORQUE_LIMIT_H 49
#define SMS_STS_LOCK 55

//-------SRAM(read only)--------
#define SMS_STS_PRESENT_POSITION_L 56
#define SMS_STS_PRESENT_POSITION_H 57
#define SMS_STS_PRESENT_SPEED_L 58
#define SMS_STS_PRESENT_SPEED_H 59
#define SMS_STS_PRESENT_LOAD_L 60
#define SMS_STS_PRESENT_LOAD_H 61
#define SMS_STS_PRESENT_VOLTAGE 62
#define SMS_STS_PRESENT_TEMPERATURE 63
#define SMS_STS_MOVING 66
#define SMS_STS_PRESENT_CURRENT_L 69
#define SMS_STS_PRESENT_CURRENT_H 70
