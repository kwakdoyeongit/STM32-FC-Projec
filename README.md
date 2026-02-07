기존 드론 시스템의 PID 게인값이 코드 내 매크로(#define)로 고정되어 있어
비행 중 게인 수정 시 매번 코드를 수정하고 다시 업로드해야 하는 번거로움이 있어서 
이를 해결하기 위해 텔레메트리 통신을 통한 실시간 PID 튜닝 시스템을 구축

[Step 1] 파라미터 구조체(Param_type.h) 확장
기존 문제: param.pid.RATE.roll 처럼 축별로 단일 float 변수만 존재하여 P값만 수용 가능
I나 D값을 수정 필요
-> 단일 변수를 kp, ki, kd 세 개의 멤버를 가진 PID_VAL 구조체로 변경
-> 각 제어 축(Roll/Pitch/Yaw)마다 독립적인 P, I, D 저장 공간이 확보

[기존 코드]

```c
struct __attribute__((packed)) {

float roll;

float pitch;

float yaw;

} ANGLE;

struct __attribute__((packed)) {

float roll;

float pitch;

float yaw;

} RATE;

} pid;

} PARAM;
#endif // PARAM_TYPE_H_
```

[수정 코드] 

```c
/*
 * =================================================================================
 * AUTO-GENERATED FILE. (Modified for PID Tuning)
 * =================================================================================
 */

#ifndef PARAM_TYPE_H_
#define PARAM_TYPE_H_

#define _MINILINK_PARAM_XML_VERSION    (0.1)

#include <stdint.h>


typedef struct __attribute__((packed)) {
    float kp;
    float ki;
    float kd;
} PID_VAL;

.
.
.
    struct __attribute__((packed)) {
        struct __attribute__((packed)) {
            PID_VAL roll;
            PID_VAL pitch;
            PID_VAL yaw;
        } ANGLE;
        struct __attribute__((packed)) {
            PID_VAL roll;
            PID_VAL pitch;
            PID_VAL yaw;
        } RATE;
    } pid;

} PARAM;

#endif // PARAM_TYPE_H_
```

[Step 2] calculateServoOutput() 함수 로직 재설계
함수 내부에서 고정된 매크로 값을 사용하여 실시간 변화에 대응할 수 없었음. 
통신부(Serial.c)가 노트북에서 받은 값을 주머니에 채워주면, 제어부(Servo.c)는 그 주머니를 열어 즉시 계산에 사용

[Serial.c]

```c
case 250:
		// 외부 제어기(각도) 제어 이득 설정
		if(serialRX.header.length-MINILINK_MIN_PACKET_SIZE != sizeof(param.pid.ANGLE)) break;
		memcpy(&param.pid.ANGLE, serialRX.payload, sizeof(param.pid.ANGLE));

		break;

	case 251:
		// 내부 제어기(각속도) 제어 이득 설정
		if(serialRX.header.length-MINILINK_MIN_PACKET_SIZE != sizeof(param.pid.RATE)) break;
		memcpy(&param.pid.RATE, serialRX.payload, sizeof(param.pid.RATE));

		break;

    case 252:
        // PID 제어 이득 설정
		if(serialRX.header.length-MINILINK_MIN_PACKET_SIZE != sizeof(param.pid)) break;
        memcpy(&param.pid, serialRX.payload, sizeof(param.pid));

        break;
	}

	return 0;
}
```

Serial.c의 case 251 부분을 보면 memcpy(&param.pid.RATE, ...) 코드가 있음
여기서 param.pid.RATE는 노트북에서 보내는 각속도 PID 패킷이 도착하는 정확한 주소값

[기존 코딩]

```c
#include <FC_Servo/Servo_module.h>
#include <math.h>
#include <string.h>
#include "FC_AHRS/AHRS.h"
.
.
void calculateServoOutput(void)
{
	float dt_s = msg.timing.dt_imu_s; // IMU 갱신값 기준 dt
	if (dt_s <= 0.0f || dt_s > 0.05f) dt_s = 0.001f;
    /* D LPF alpha 업데이트 */
    float alpha = lpf_alpha(D_CUTOFF_HZ, dt_s);
    pid_roll.d_alpha  = alpha;
    pid_pitch.d_alpha = alpha;
    pid_yaw.d_alpha   = alpha;
.
.
//오차 계산
    float sp_roll  = clampf(ANGLE_KP_RATE_PER_RAD * (sp_roll_angle  - roll_meas_rad),
                            -MAX_ROLL_RATE_RAD_S,  MAX_ROLL_RATE_RAD_S);
    float sp_pitch = clampf(ANGLE_KP_RATE_PER_RAD * (sp_pitch_angle - pitch_meas_rad),
                            -MAX_PITCH_RATE_RAD_S, MAX_PITCH_RATE_RAD_S);

    float sp_yaw   = (yaw_cmd / 500.0f) * MAX_YAW_RATE_RAD_S;
```

[수정 코드]

```c
.
.
void calculateServoOutput(void)
{      // Roll 축 업데이트
	    pid_roll.kp = param.pid.RATE.roll.kp;
	    pid_roll.ki = param.pid.RATE.roll.ki;
	    pid_roll.kd = param.pid.RATE.roll.kd;

	    // Pitch 축 업데이트
	    pid_pitch.kp = param.pid.RATE.pitch.kp;
	    pid_pitch.ki = param.pid.RATE.pitch.ki;
	    pid_pitch.kd = param.pid.RATE.pitch.kd;

	    // Yaw 축 업데이트
	    pid_yaw.kp = param.pid.RATE.yaw.kp;
	    pid_yaw.ki = param.pid.RATE.yaw.ki;
	    pid_yaw.kd = param.pid.RATE.yaw.kd;

	float dt_s = msg.timing.dt_imu_s; // IMU 갱신값 기준 dt
	if (dt_s <= 0.0f || dt_s > 0.05f) dt_s = 0.001f;
    /* D LPF alpha 업데이트 */
    float alpha = lpf_alpha(D_CUTOFF_HZ, dt_s);
    pid_roll.d_alpha  = alpha;
    pid_pitch.d_alpha = alpha;
    pid_yaw.d_alpha   = alpha;
.
.

//오차 계산
    float sp_roll  = clampf(param.pid.ANGLE.roll.kp * (sp_roll_angle  - roll_meas_rad),
                            -MAX_ROLL_RATE_RAD_S,  MAX_ROLL_RATE_RAD_S);
    float sp_pitch = clampf(param.pid.ANGLE.pitch.kp * (sp_pitch_angle - pitch_meas_rad),
                            -MAX_PITCH_RATE_RAD_S, MAX_PITCH_RATE_RAD_S);

    float sp_yaw   = (yaw_cmd / 500.0f) * MAX_YAW_RATE_RAD_S;

```

-> 통신 인터럽트로 인해 param 값이 바뀌는 즉시, 다음 제어 루프 계산에 반영
param 주머니에 있는 값을 로컬 제어 변수로 복사해와서 계산
