#ifndef __COM_FLIGHT_CONFIG_H
#define __COM_FLIGHT_CONFIG_H

/**
 * ============================================================
 *  飞控端 关键控制参数配置文件
 *  集中管理PID参数、滤波器参数、电机限幅等可调常量
 * ============================================================
 */

/* ==================== 姿态环 PID 系数 ==================== */
/* 俯仰 外环(角度) */
#define PITCH_OUT_KP        (-7.0f)
#define PITCH_OUT_KI        (0.0f)
#define PITCH_OUT_KD        (0.0f)

/* 俯仰 内环(角速率) */
#define PITCH_IN_KP         (2.0f)
#define PITCH_IN_KI         (0.0f)
#define PITCH_IN_KD         (0.1f)

/* 横滚 外环(角度) */
#define ROLL_OUT_KP         (-7.0f)
#define ROLL_OUT_KI         (0.0f)
#define ROLL_OUT_KD         (0.0f)

/* 横滚 内环(角速率) */
#define ROLL_IN_KP          (-2.0f)
#define ROLL_IN_KI          (0.0f)
#define ROLL_IN_KD          (-0.1f)

/* 偏航 外环(角度) */
#define YAW_OUT_KP          (-2.2f)
#define YAW_OUT_KI          (0.0f)
#define YAW_OUT_KD          (0.0f)

/* 偏航 内环(角速率) */
#define YAW_IN_KP           (-1.5f)
#define YAW_IN_KI           (0.0f)
#define YAW_IN_KD           (0.0f)

/* 定高 外环(高度) */
#define HEIGHT_OUT_KP       (-1.4f)
#define HEIGHT_OUT_KI       (0.0f)
#define HEIGHT_OUT_KD       (0.0f)

/* 定高 内环(Z速率) */
#define ZSPEED_IN_KP        (-1.3f)
#define ZSPEED_IN_KI        (0.0f)
#define ZSPEED_IN_KD        (-0.08f)

/* ==================== PID 安全限幅参数 ==================== */

/* 外环(角度环): 期望角度范围约为 ±20deg */
#define POSTURE_OUT_INTEGRAL_MAX   (50.0f)   /* 积分限幅 */
#define POSTURE_OUT_OUTPUT_MAX     (200.0f)  /* 输出限幅 (期望角速率 deg/s) */
#define POSTURE_OUT_DEADBAND       (0.5f)    /* 微分死区 (deg) */

/* 内环(角速率环) */
#define GYRO_IN_INTEGRAL_MAX       (100.0f)  /* 积分限幅 */
#define GYRO_IN_OUTPUT_MAX         (400.0f)  /* 输出限幅 (PWM增量) */
#define GYRO_IN_DEADBAND           (0.0f)    /* 微分死区 */

/* 定高外环 */
#define HEIGHT_OUT_INTEGRAL_MAX    (50.0f)   /* 积分限幅 */
#define HEIGHT_OUT_OUTPUT_MAX      (100.0f)  /* 输出限幅 */
#define HEIGHT_OUT_DEADBAND        (0.5f)    /* 微分死区 */

/* 定高内环(Z速率环) */
#define ZSPEED_IN_INTEGRAL_MAX     (80.0f)   /* 积分限幅 */
#define ZSPEED_IN_OUTPUT_MAX       (150.0f)  /* 输出限幅 */
#define ZSPEED_IN_DEADBAND         (0.0f)    /* 微分死区 */

/* ==================== 摇杆与控制常量 ==================== */
#define JOYSTICK_MIDPOINT          (500)     /* 摇杆中位值 */
#define JOYSTICK_ANGLE_SCALE       (0.04f)   /* 摇杆->期望角度缩放 (500*0.04=20度) */
#define MOTOR_THR_SCALE            (0.7f)    /* 油门PWM缩放系数 */
#define MOTOR_OUTPUT_LIMIT         (1000)    /* 单路电机PWM上限 */

/* ==================== 传感器滤波参数 ==================== */
#define LP_FILTER_ALPHA            (0.8f)    /* 一阶低通滤波系数 (越大越平滑) */

/* 卡尔曼滤波器 初始化参数 */
#define KF_INIT_Q                  (0.02f)   /* 过程噪声协方差 */
#define KF_INIT_R                  (0.001f)  /* 测量噪声协方差 */

/* ==================== IMU 互补滤波 参数 ==================== */
#define IMU_KP_DEF                 (0.8f)    /* Mahony滤波 比例增益 */
#define IMU_KI_DEF                 (0.0003f) /* Mahony滤波 积分增益 */
#define IMU_KI_LIMIT               (0.3f)    /* Mahony滤波 积分误差限幅 */

/* ==================== 高度传感器 参数 ==================== */
#define HEIGHT_JUMP_THRESHOLD      (500)     /* 高度跳变阈值 (mm) */
#define HEIGHT_HORIZONTAL_THRESHOLD (100)    /* 水平偏移阈值 (摇杆单位) */
#define HEIGHT_PID_DIVIDER         (5)       /* 定高PID分频系数 */

/* ==================== 失控保护 参数 ==================== */
#define FAILSAFE_TIMEOUT_CNT       (200)     /* 失控超时计数 (200*6ms=1.2s) */
#define ARM_THR_HIGH               (960)     /* 解锁: 油门高位阈值 */
#define ARM_THR_LOW                (20)      /* 解锁: 油门低位阈值 */
#define ARM_DURATION_CNT           (200)     /* 解锁: 保持时间计数 */
#define DISARM_DURATION_CNT        (200*50)  /* 上锁: 保持低油门时间 (约1分钟) */

/* ==================== 无线通讯协议参数 ==================== */
/* 2.4G 帧头标识 (与遥控器端必须一致) */
#define FRAME_0                  (0x11)
#define FRAME_1                  (0x22)
#define FRAME_2                  (0x33)

/* ==================== 电机安全阈值 ==================== */
#define MOTOR_MIN_THR              (30)      /* 最小油门: 低于此值电机关闭 */
#define HEIGHT_PID_Z_LIMIT_LOW     (-150)    /* 定高PID Z轴输出下限 */
#define HEIGHT_PID_Z_LIMIT_HIGH    (150)     /* 定高PID Z轴输出上限 */

#endif /* __COM_FLIGHT_CONFIG_H */
