#ifndef __COM_REMOTE_CONFIG_H
#define __COM_REMOTE_CONFIG_H

/**
 * ============================================================
 *  遥控器端 关键控制参数配置文件
 *  集中管理摇杆校准、死区、按键防抖、无线协议等可调常量
 * ============================================================
 */

/* ==================== 摇杆采集与校准参数 ==================== */
#define JOYSTICK_RANGE_MAX       (1000)  /* 摇杆量程最大值 */
#define JOYSTICK_RANGE_MIN       (0)     /* 摇杆量程最小值 */
#define JOYSTICK_MIDPOINT        (500)   /* 摇杆中位值 */
#define JOYSTICK_ADC_MAX         (4095)  /* ADC原始值上限 (12位) */
#define JOYSTICK_DEADZONE        (8)     /* 摇杆死区: 中位±8以内强制归中 */
#define JOYSTICK_THR_DEADZONE    (16)    /* 油门死区: 低于16强制归零 */
#define JOYSTICK_BIAS_SAMPLES    (100)   /* 校准采样次数 */

/* ==================== 按键防抖参数 ==================== */
#define KEY_DEBOUNCE_MS          (30)    /* 按键软件防抖延迟 (ms) */
#define KEY_SHORT_PRESS_MAX      (5)     /* 短按最大时间 (x100ms = 500ms) */
#define KEY_LONG_PRESS_TIMEOUT   (12)    /* 长按超时时间 (x100ms = 1.2s) */
#define KEY_RELEASE_TIMEOUT      (100)   /* 等待释放超时次数 (防止卡键) */

/* ==================== 无线通讯协议参数 ==================== */
/* 2.4G 帧头标识 (与飞控端必须一致) */
#define FRAME_0                  (0x11)
#define FRAME_1                  (0x22)
#define FRAME_2                  (0x33)

/* ==================== 电源管理参数 ==================== */
#define POWER_KEEPALIVE_CYCLE    (10 * 1000)  /* 电源保活周期 (ms) */

#endif /* __COM_REMOTE_CONFIG_H */
