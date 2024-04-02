#ifndef GIMBAL_APP_H
#define GIMBAL_APP_H

#include "stm32f4xx_hal.h"
#include "struct_typedef.h"
#include "pid.h"
#include "remote_app.h"
#include "INS_task.h"


#define GIMBAL_TASK_TIME_1MS                  (1u)           

#define ROBOT_COMMON_MODE_KEY                 KEY_PRESSED_OFFSET_V            
#define ROBOT_ROTATE_STOP_MODE_KEY            KEY_PRESSED_OFFSET_E        
#define ROBOT_ROTATE_MOTION_MODE_KEY          KEY_PRESSED_OFFSET_Q 

#define GIMBAL_TASK_INIT_TIME                 (3005)

#define GAMBAL_YAW_MAX_ANGLE_SET_FACT         (0.0002f)   //閬ユ帶閬ユ潌鐏垫晱搴�0.0005
#define GAMBAL_PITCH_MAX_ANGLE_SET_FACT       (0.0002f)

#define GAMBAL_MOUSE_PITCH_MAX_ANGLE_SET_FACT (-0.008f)    //澶т綅绉婚紶鏍囩伒鏁忓害
#define GAMBAL_MOUSE_YAW_MAX_ANGLE_SET_FACT   (-0.008f)
#define GAMBAL_MOUSE_PITCH_MIN_ANGLE_SET_FACT (-0.005f)    //灏忎綅绉婚紶鏍囩伒鏁忓害
#define GAMBAL_MOUSE_YAW_MIN_ANGLE_SET_FACT   (-0.005f)

#define GAMBAL_YAW_INIT_ENCODE_VALUE          (3570.0f)//7678
#define GAMBAL_PITCH_INIT_ENCODE_VALUE        (1340.0f) //

#define GAMBAL_YAW_angle_VALUE     (22.755555f)                    //yaw
#define GAMBAL_PITCH_angle_VALUE   (22.755555f)                    //pitch
#define CAMBAL_PITHC_LEVEL_ANGLE_TO_ENCODE    (3700.0f)//浜戝彴姘村钩鍊�
#define CAMBAL_PITCH_MIN_ANGLE_TO_ENCODE      (3200.0f)
#define CAMBAL_PITCH_MAX_ANGLE_TO_ENCODE      (4285.0f)//浣庡ご涓嬮檺

#define GAMBAL_ENCODE_TO_ANGLE                ((float)0.0439506775729459)  //   360.0/8191.0

/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W            ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S            ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A            ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D            ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT        ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL         ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q            ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E            ((uint16_t)1 << 7)

#define KEY_PRESSED_OFFSET_R            ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F            ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G            ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z            ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X            ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C            ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V            ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B            ((uint16_t)1 << 15)
   


typedef enum
{
	ROBOT_CALI_MODE = 0,     //璋冭瘯妯″紡
	ROBOT_INIT_MODE,	     //鍒濆鍖�
	ROBOT_INIT_END_MODE,     //鍒濆鍖栫粨鏉熷垏鎹㈢偣
	ROBOT_COMMON_MODE,       //鏅€氬簳鐩樿窡闅忔ā寮�
	ROBOT_ROTATE_STOP_MODE,  //闈欐灏忛檧铻�
	ROBOT_ROTATE_MOTION_MODE,//杩愬姩灏忛檧铻�
	ROBOT_VISION_MODE,       //瑙嗚鑷瀯妯″紡
	ROBOT_ERROR_MODE,        //閿欒
}robot_work_mode_e;

typedef enum
{
	GIMBAL_CALI_MODE = 0,
	GIMBAL_INIT_MODE,  
	GIMBAL_ABSOLUTE_ANGLE_MODE,
	GIMBAL_RELATIVE_ANGLE_MODE, 
	GIMBAL_ROTATE_MODE,
//	GIMBAL_MOTIONLESS, 	
}gimbal_work_mode_e;
  
typedef enum
{
    GIMBAL_MOTOR_RAW = 0,    //鐢垫満鍘熷鍊兼帶鍒�
    GIMBAL_MOTOR_GYRO,       //鐢垫満闄€铻轰华瑙掑害鎺у埗
    GIMBAL_MOTOR_ENCONDE,    //鐢垫満缂栫爜鍊艰搴︽帶鍒�
}gimbal_motor_feedback_mode_e;

typedef enum
{
	KEY_MOUSE_MODE = 0,
	REMOTE_MODE,
	GUI_CALI_MODE,
}robot_control_mode_e;

typedef struct
{
	int16_t yaw_motor;
	int16_t pitch_motor;
	
	int16_t trigger_motor;
	int16_t fric1_motor;
	int16_t fric2_motor;
}given_current_t;

typedef struct
{
	cascade_pid_t yaw_pid;	
	cascade_pid_t pitch_pid;

}gimbal_pid_t;

typedef struct
{
	RC_ctrl_t *rc_ctrl;
	const INS_t *gimbal_INS;
	motor_msg_t *gimbal_yaw_motor_msg;
	motor_msg_t *gimbal_pitch_motor_msg;
	
	given_current_t given_current;
	
	gimbal_motor_feedback_mode_e yaw_motor_fdb_mode;
	gimbal_motor_feedback_mode_e pitch_motor_fdb_mode;
	
	float gimbal_yaw_set;
	float gimbal_yaw_fdb;
	float gimbal_pitch_set;
	float gimbal_pitch_fdb;

	int16_t vision_yaw;
	int16_t vision_pitch;
}gimbal_control_data_t;


extern given_current_t given_current;
uint8_t get_robot_control_mode(void);
uint8_t get_robot_work_mode(void);
uint8_t get_gimbal_work_mode(void);
void set_robot_control_mode(robot_control_mode_e mode);
float shoot_abs(float value);
void gimbal_set_and_fdb_update(gimbal_control_data_t *gimbal_control_data,   \
							   robot_control_mode_e robot_control_mode, 
							   _tx2_control_data control_data);
#endif
