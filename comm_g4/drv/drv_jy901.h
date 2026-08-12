#ifndef _JY901_H_
#define _JY901_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 帧头 */
#define JY901_FRAME_HEADER             0x55

/* 数据类型标识 */
#define JY901_ID_TIME                  0x50
#define JY901_ID_ACCEL                 0x51
#define JY901_ID_GYRO                  0x52
#define JY901_ID_ANGLE                 0x53
#define JY901_ID_MAG                   0x54
#define JY901_ID_PORT                  0x55
#define JY901_ID_BARO                  0x56
#define JY901_ID_GPS_LONLAT            0x57
#define JY901_ID_GPS_SPEED             0x58
#define JY901_ID_QUATERNION            0x59
#define JY901_ID_GPS_ACCURACY          0x5A

/* 指令帧头 */
#define JY901_CMD_HEADER               0xFF
#define JY901_CMD_AA                   0xAA

/* 默认参数 */
#define JY901_DEFAULT_BAUDRATE         9600
#define JY901_DEFAULT_ADDRESS          0x50

/* 量程定义：宏值直接等于换算公式使用的物理量程。 */
#define JY901_ACCEL_RANGE_2G           2
#define JY901_ACCEL_RANGE_4G           4
#define JY901_ACCEL_RANGE_8G           8
#define JY901_ACCEL_RANGE_16G          16

#define JY901_GYRO_RANGE_250           250
#define JY901_GYRO_RANGE_500           500
#define JY901_GYRO_RANGE_1000          1000
#define JY901_GYRO_RANGE_2000          2000

#define JY901_ALGORITHM_9AXIS          0
#define JY901_ALGORITHM_6AXIS          1

/* 端口模式 */
#define JY901_PORT_MODE_ADC            0x00
#define JY901_PORT_MODE_DIGITAL_IN     0x01
#define JY901_PORT_MODE_DIGITAL_OUT_H  0x02
#define JY901_PORT_MODE_DIGITAL_OUT_L  0x03
#define JY901_PORT_MODE_PWM            0x04
#define JY901_PORT_MODE_CLR            0x05  /* D1 only */

/* 波特率 */
#define JY901_BAUD_2400                0x00
#define JY901_BAUD_4800                0x01
#define JY901_BAUD_9600                0x02
#define JY901_BAUD_19200               0x03
#define JY901_BAUD_38400               0x04
#define JY901_BAUD_57600               0x05
#define JY901_BAUD_115200              0x06
#define JY901_BAUD_230400              0x07
#define JY901_BAUD_460800              0x08
#define JY901_BAUD_921600              0x09

/* 输出速率 */
#define JY901_RATE_0_1HZ               0x01
#define JY901_RATE_0_5HZ               0x02
#define JY901_RATE_1HZ                 0x03
#define JY901_RATE_2HZ                 0x04
#define JY901_RATE_5HZ                 0x05
#define JY901_RATE_10HZ                0x06
#define JY901_RATE_20HZ                0x07
#define JY901_RATE_50HZ                0x08
#define JY901_RATE_100HZ               0x09
#define JY901_RATE_200HZ               0x0B
#define JY901_RATE_SINGLE              0x0C

/* 回传数据包掩码：可用按位或组合后传给 jy901_send_output_packets()。 */
#define JY901_OUTPUT_TIME              (1U << 0)   /* 0x50 时间包 */
#define JY901_OUTPUT_ACCEL             (1U << 1)   /* 0x51 加速度包 */
#define JY901_OUTPUT_GYRO              (1U << 2)   /* 0x52 角速度包 */
#define JY901_OUTPUT_ANGLE             (1U << 3)   /* 0x53 角度包 */
#define JY901_OUTPUT_MAG               (1U << 4)   /* 0x54 磁场包 */
#define JY901_OUTPUT_PORT              (1U << 5)   /* 0x55 端口状态包 */
#define JY901_OUTPUT_BARO              (1U << 6)   /* 0x56 气压高度包 */
#define JY901_OUTPUT_GPS_LONLAT        (1U << 7)   /* 0x57 经纬度包 */
#define JY901_OUTPUT_GPS_SPEED         (1U << 8)   /* 0x58 地速包 */
#define JY901_OUTPUT_QUATERNION        (1U << 9)   /* 0x59 四元数包 */
#define JY901_OUTPUT_GPS_ACCURACY      (1U << 10)  /* 0x5A 卫星定位精度包 */

#define JY901_OUTPUT_NONE              0U
#define JY901_OUTPUT_IMU               (JY901_OUTPUT_ACCEL | JY901_OUTPUT_GYRO | JY901_OUTPUT_ANGLE)
#define JY901_OUTPUT_ATTITUDE          (JY901_OUTPUT_ACCEL | JY901_OUTPUT_GYRO | JY901_OUTPUT_ANGLE | JY901_OUTPUT_MAG)
#define JY901_OUTPUT_GPS_ALL           (JY901_OUTPUT_GPS_LONLAT | JY901_OUTPUT_GPS_SPEED | JY901_OUTPUT_GPS_ACCURACY)
#define JY901_OUTPUT_ALL               ((1U << 11) - 1U)

/* 原始传感器数据（16位有符号） */
typedef struct {
    int16_t ax;      /* X轴加速度 */
    int16_t ay;      /* Y轴加速度 */
    int16_t az;      /* Z轴加速度 */
    int16_t wx;      /* X轴角速度 */
    int16_t wy;      /* Y轴角速度 */
    int16_t wz;      /* Z轴角速度 */
    int16_t roll;    /* X轴角度 */
    int16_t pitch;   /* Y轴角度 */
    int16_t yaw;     /* Z轴角度 */
    int16_t hx;      /* X轴磁场 */
    int16_t hy;      /* Y轴磁场 */
    int16_t hz;      /* Z轴磁场 */
    int16_t temp;    /* 温度 (单位: 0.01°C) */
} jy901_raw_data_t;

/* 物理量数据 */
typedef struct {
    float ax;        /* X轴加速度 (g) */
    float ay;        /* Y轴加速度 (g) */
    float az;        /* Z轴加速度 (g) */
    float wx;        /* X轴角速度 (°/s) */
    float wy;        /* Y轴角速度 (°/s) */
    float wz;        /* Z轴角速度 (°/s) */
    float roll;      /* X轴角度 (°) */
    float pitch;     /* Y轴角度 (°) */
    float yaw;       /* Z轴角度 (°) */
    float hx;        /* X轴磁场 (uT) */
    float hy;        /* Y轴磁场 (uT) */
    float hz;        /* Z轴磁场 (uT) */
    float temp;      /* 温度 (°C) */
} jy901_float_data_t;

/* 时间数据 */
typedef struct {
    uint16_t year;   /* 2000+ */
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint16_t millisecond;
} jy901_time_t;

/* 端口状态 */
typedef struct {
    uint16_t d0;
    uint16_t d1;
    uint16_t d2;
    uint16_t d3;
} jy901_port_status_t;

/* 气压高度 */
typedef struct {
    int32_t pressure;  /* Pa */
    int32_t height;    /* cm */
} jy901_baro_t;

/* GPS数据 */
typedef struct {
    int32_t  longitude;  /* 经度 (格式: ddmm.mmmmmm * 100000) */
    int32_t  latitude;   /* 纬度 (格式: ddmm.mmmmmm * 100000) */
    int16_t  gps_height; /* GPS高度 (0.1m) */
    int16_t  gps_yaw;    /* GPS航向角 (0.01°) */
    uint32_t gps_speed;  /* 地速 (0.001 km/h) */
    uint8_t  sat_num;    /* 卫星数 */
    uint16_t pdop;       /* 位置精度 (0.01) */
    uint16_t hdop;       /* 水平精度 (0.01) */
    uint16_t vdop;       /* 垂直精度 (0.01) */
} jy901_gps_t;

/* 四元素 */
typedef struct {
    float q0;
    float q1;
    float q2;
    float q3;
} jy901_quaternion_t;

/* 传感器配置 */
typedef struct {
    uint8_t accel_range;     /* 加速度量程 */
    uint8_t gyro_range;      /* 角速度量程 */
    uint8_t algorithm;       /* 算法选择 */
    uint8_t output_rate;     /* 输出速率 */
    uint8_t baudrate;        /* 波特率 */
    uint8_t iic_address;     /* IIC地址 */
    uint8_t port_mode[4];    /* 端口模式 */
    uint16_t pwm_high[4];    /* PWM高电平宽度 (us) */
    uint16_t pwm_period[4];  /* PWM周期 (us) */
    int16_t accel_offset[3]; /* 加速度零偏 */
    int16_t gyro_offset[3];  /* 角速度零偏 */
    int16_t mag_offset[3];   /* 磁场零偏 */
    uint16_t output_mask_l;  /* 输出内容低字节 */
    uint16_t output_mask_h;  /* 输出内容高字节 */
} jy901_config_t;


typedef void (*jy901_data_callback_t)(uint8_t type, const uint8_t* data, uint16_t len);


/**
 * @brief 初始化JY901模块
 */
void drv_jy901_init(void);

/**
 * @brief 处理接收到的字节
 */
void drv_jy901_process(void);

/**
 * @brief 发送解锁指令（配置前必须调用）
 */
void jy901_send_unlock(void);

/**
 * @brief 保存当前配置
 */
void jy901_send_save_config(void);

/**
 * @brief 恢复出厂设置
 */
void jy901_send_restore_default(void);

/**
 * @brief 进入/退出校准模式
 * @param mode 0:退出 1:加速度校准 2:磁场校准 3:高度置零
 */
void jy901_send_calibration(uint8_t mode);

/**
 * @brief Z轴归零（仅6轴算法有效）
 */
void jy901_send_z_axis_reset(void);

/**
 * @brief 设置安装方向
 * @param vertical 0:水平 1:垂直
 */
void jy901_send_mount_direction(uint8_t vertical);

/**
 * @brief 休眠/唤醒切换
 */
void jy901_send_sleep_toggle(void);

/**
 * @brief 设置算法
 * @param algo 0:9轴 1:6轴
 */
void jy901_send_algorithm(uint8_t algo);

/**
 * @brief 陀螺仪自动校准开关
 * @param enable true:开启 false:关闭
 */
void jy901_send_gyro_autocal(bool enable);

/**
 * @brief 设置输出内容
 * @param mask_l 低字节掩码 (bit0=0x50, bit1=0x51, ...)
 * @param mask_h 高字节掩码 (bit0=0x58, bit1=0x59, bit2=0x5A)
 */
void jy901_send_output_mask(uint8_t mask_l, uint8_t mask_h);

/**
 * @brief 设置输出内容
 * @param packets JY901_OUTPUT_* 掩码，可按位或组合
 *
 * 示例：输出加速度、角速度、角度、磁场：
 * jy901_send_output_packets(JY901_OUTPUT_ATTITUDE);
 */
void jy901_send_output_packets(uint16_t packets);


/**
 * @brief 设置输出速率
 * @param rate JY901_RATE_* 宏定义
 */
void jy901_send_output_rate(uint8_t rate);

/**
 * @brief 设置波特率
 * @param baud JY901_BAUD_* 宏定义
 */
void jy901_send_baudrate(uint8_t baud);

/**
 * @brief 设置端口模式
 * @param port 端口号 0-3
 * @param mode JY901_PORT_MODE_* 宏定义
 */
void jy901_send_port_mode(uint8_t port, uint8_t mode);

/**
 * @brief 设置PWM参数
 * @param port 端口号 0-3
 * @param high_us 高电平宽度 (us)
 * @param period_us PWM周期 (us)
 */
void jy901_send_pwm_param(uint8_t port, uint16_t high_us, uint16_t period_us);

/**
 * @brief 设置IIC地址
 * @param addr 7位IIC地址
 */
void jy901_send_iic_address(uint8_t addr);

/**
 * @brief 设置加速度零偏
 * @param axis 0:X 1:Y 2:Z
 * @param offset 零偏值
 */
void jy901_send_accel_offset(uint8_t axis, int16_t offset);

/**
 * @brief 设置角速度零偏
 * @param axis 0:X 1:Y 2:Z
 * @param offset 零偏值
 */
void jy901_send_gyro_offset(uint8_t axis, int16_t offset);

/**
 * @brief 设置磁场零偏
 * @param axis 0:X 1:Y 2:Z
 * @param offset 零偏值
 */
void jy901_send_mag_offset(uint8_t axis, int16_t offset);

/**
 * @brief 解析加速度数据
 * @param raw 原始16位数据
 * @param range 量程 (2,4,8,16)
 * @return 物理值 (g)
 */
float jy901_parse_accel(int16_t raw, uint8_t range);

/**
 * @brief 解析角速度数据
 * @param raw 原始16位数据
 * @param range 量程 (250,500,1000,2000)
 * @return 物理值 (°/s)
 */
float jy901_parse_gyro(int16_t raw, uint16_t range);

/**
 * @brief 解析角度数据
 * @param raw 原始16位数据
 * @return 物理值 (°)
 */
float jy901_parse_angle(int16_t raw);

/**
 * @brief 解析温度数据
 * @param raw 原始16位数据
 * @return 物理值 (°C)
 */
float jy901_parse_temp(int16_t raw);

/**
 * @brief 获取最新解析后的数据
 */
const jy901_float_data_t* jy901_get_data(void);

/**
 * @brief 获取最新时间
 */
const jy901_time_t* jy901_get_time(void);

/**
 * @brief 获取最新端口状态
 */
const jy901_port_status_t* jy901_get_port_status(void);

/**
 * @brief 获取最新气压和高度数据
 * @return 内部只读缓存指针，气压单位 Pa，高度单位 cm
 */
const jy901_baro_t* jy901_get_baro(void);

/**
 * @brief 获取最新GPS数据
 * @return 内部只读缓存指针，经纬度/地速/精度由对应数据包分别更新
 */
const jy901_gps_t* jy901_get_gps(void);

/**
 * @brief 获取最新四元数数据
 * @return 内部只读缓存指针，四元数已按原始值/32768换算
 */
const jy901_quaternion_t* jy901_get_quaternion(void);

/**
 * @brief 检查数据是否已更新
 */
bool jy901_is_data_updated(void);

/**
 * @brief 清除更新标志
 */
void jy901_clear_update_flag(void);

#ifdef __cplusplus
}
#endif

#endif
