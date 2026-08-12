#include "drv_jy901.h"
#include <string.h>
#include <stdlib.h>

#include "bsp_usart2.h"



/* JY901 串口输出帧固定为 11 字节：帧头 + 包ID + 8字节数据 + 校验和。 */
#define JY901_FRAME_LEN         11
#define JY901_FRAME_DATA_LEN    8

#define RX_BUFFER_SIZE  JY901_FRAME_LEN

/* 数据更新标志*/
static bool _data_updated = false;

/* 存储最新数据 */
jy901_float_data_t _float_data = {0};
jy901_time_t _time_data = {0};
jy901_port_status_t _port_data = {0};
jy901_baro_t _baro_data = {0};
jy901_gps_t _gps_data = {0};
jy901_quaternion_t _quat_data = {0};

/* 当前配置范围 */
static uint8_t _accel_range = 16;
static uint16_t _gyro_range = 2000;


/* 计算校验和 */
static uint8_t _calc_checksum(const uint8_t* data, uint16_t len)
{
    uint8_t sum = 0;
    for (uint16_t i = 0; i < len; i++)
	{
        sum += data[i];
    }
    return sum;
}

/* 组合16位数据 (低字节在前) */
static int16_t _combine_int16(uint8_t low, uint8_t high)
{
    return (int16_t)(((uint16_t)high << 8) | (uint16_t)low);
}

/* 组合32位数据 */
static int32_t _combine_int32(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3)
{
    uint32_t value = ((uint32_t)b3 << 24) |
                     ((uint32_t)b2 << 16) |
                     ((uint32_t)b1 << 8)  |
                     (uint32_t)b0;
    return (int32_t)value;
}

float jy901_parse_angle(int16_t raw) 
{
    return (float)raw / 32768.0f * 180.0f;
}

float jy901_parse_temp(int16_t raw) 
{
    return (float)raw / 100.0f;
}
static bool _is_valid_output_rate(uint8_t rate)
{
    return !(((rate < JY901_RATE_0_1HZ) || (rate > JY901_RATE_100HZ)) &&
             (rate != JY901_RATE_200HZ) &&
             (rate != JY901_RATE_SINGLE));
}

/* 发送指令帧 */
static void _send_command(uint8_t reg, uint8_t data_l, uint8_t data_h)
{
	
    uint8_t cmd[5];
	
	jy901_send_unlock();
	
    cmd[0] = JY901_CMD_HEADER;
    cmd[1] = JY901_CMD_AA;
    cmd[2] = reg;
    cmd[3] = data_l;
    cmd[4] = data_h;
    bsp_UART2_DMASend(cmd, 5);
	
	jy901_send_unlock();
	
	jy901_send_save_config();
	
}

/* 发送单寄存器指令 (高字节为0) */
static void _send_command_single(uint8_t reg, uint8_t data) 
{
    _send_command(reg, data, 0x00);
}

/* 解析帧数据 */
static void _parse_frame(uint8_t id, const uint8_t* data, uint16_t len) 
{
    int16_t raw;
    
    if ((data == NULL)||(len < JY901_FRAME_DATA_LEN))
	{
        return;
    }
    
    switch (id) 
	{
        case JY901_ID_TIME:
            /* 0x50: 时间包 */
            _time_data.year = 2000 + data[0];
            _time_data.month = data[1];
            _time_data.day = data[2];
            _time_data.hour = data[3];
            _time_data.minute = data[4];
            _time_data.second = data[5];
            _time_data.millisecond = _combine_int16(data[6], data[7]);
            _data_updated = true;
            break;
        case JY901_ID_ACCEL:
            /* 0x51: 加速度包 */
            raw = _combine_int16(data[0], data[1]);
            _float_data.ax = jy901_parse_accel(raw, _accel_range);
            raw = _combine_int16(data[2], data[3]);
            _float_data.ay = jy901_parse_accel(raw, _accel_range);
            raw = _combine_int16(data[4], data[5]);
            _float_data.az = jy901_parse_accel(raw, _accel_range);
            _float_data.temp = jy901_parse_temp(_combine_int16(data[6], data[7]));
            _data_updated = true;
            break;
        case JY901_ID_GYRO:
            /* 0x52: 角速度包 */
            raw = _combine_int16(data[0], data[1]);
            _float_data.wx = jy901_parse_gyro(raw, _gyro_range);
            raw = _combine_int16(data[2], data[3]);
            _float_data.wy = jy901_parse_gyro(raw, _gyro_range);
            raw = _combine_int16(data[4], data[5]);
            _float_data.wz = jy901_parse_gyro(raw, _gyro_range);
            _data_updated = true;
            break;
        case JY901_ID_ANGLE:
            /* 0x53: 角度包 */
            raw = _combine_int16(data[0], data[1]);
            _float_data.roll = jy901_parse_angle(raw);
            raw = _combine_int16(data[2], data[3]);
            _float_data.pitch = jy901_parse_angle(raw);
            raw = _combine_int16(data[4], data[5]);
            _float_data.yaw = jy901_parse_angle(raw);
            _data_updated = true;
            break;
        case JY901_ID_MAG:
            /* 0x54: 磁场包 */
            /* 磁场值直接使用原始值，比例系数需要根据具体传感器确定 */
            _float_data.hx = (float)_combine_int16(data[0], data[1]);
            _float_data.hy = (float)_combine_int16(data[2], data[3]);
            _float_data.hz = (float)_combine_int16(data[4], data[5]);
            _data_updated = true;
            break;
        case JY901_ID_PORT:
            /* 0x55: 端口状态 */
            _port_data.d0 = _combine_int16(data[0], data[1]);
            _port_data.d1 = _combine_int16(data[2], data[3]);
            _port_data.d2 = _combine_int16(data[4], data[5]);
            _port_data.d3 = _combine_int16(data[6], data[7]);
            _data_updated = true;
            break;
        
        case JY901_ID_BARO:
            /* 0x56: 气压高度 */
            _baro_data.pressure = _combine_int32(data[0], data[1], data[2], data[3]);
            _baro_data.height   = _combine_int32(data[4], data[5], data[6], data[7]);
            _data_updated = true;
            break;
        
        case JY901_ID_GPS_LONLAT: 
            /* 0x57: 经纬度 */
            _gps_data.longitude = _combine_int32(data[0], data[1], data[2], data[3]);
            _gps_data.latitude  = _combine_int32(data[4], data[5], data[6], data[7]);
            _data_updated = true;
            break;
        case JY901_ID_GPS_SPEED: 
            /* 0x58: 地速 */
            _gps_data.gps_height = _combine_int16(data[0], data[1]);
            _gps_data.gps_yaw    = _combine_int16(data[2], data[3]);
            _gps_data.gps_speed  = (uint32_t)_combine_int32(data[4], data[5], data[6], data[7]);
            _data_updated = true;
            break;
        case JY901_ID_QUATERNION:
            /* 0x59: 四元素 */
            _quat_data.q0 = (float)_combine_int16(data[0], data[1]) / 32768.0f;
            _quat_data.q1 = (float)_combine_int16(data[2], data[3]) / 32768.0f;
            _quat_data.q2 = (float)_combine_int16(data[4], data[5]) / 32768.0f;
            _quat_data.q3 = (float)_combine_int16(data[6], data[7]) / 32768.0f;
            _data_updated = true;
            break;
        
        case JY901_ID_GPS_ACCURACY:
            /* 0x5A: 卫星定位精度 */
            _gps_data.sat_num = (uint8_t)_combine_int16(data[0], data[1]);
            _gps_data.pdop = (uint16_t)_combine_int16(data[2], data[3]);
            _gps_data.hdop = (uint16_t)_combine_int16(data[4], data[5]);
            _gps_data.vdop = (uint16_t)_combine_int16(data[6], data[7]);
            _data_updated = true;
            break;
        default:
            break;
    }
}

void drv_jy901_init(void)
{
    _data_updated = false;
    memset(&_float_data, 0, sizeof(_float_data));
    memset(&_time_data, 0, sizeof(_time_data));
    memset(&_port_data, 0, sizeof(_port_data));
    memset(&_baro_data, 0, sizeof(_baro_data));
    memset(&_gps_data, 0, sizeof(_gps_data));
    memset(&_quat_data, 0, sizeof(_quat_data));
}
/*出数据包*/
void drv_jy901_process(void)
{
	uint8_t buf[11]={0};
	
	if(JY901_FRAME_LEN==bsp_UART2_Read(buf,JY901_FRAME_LEN))
	{
		/* 验证校验和 */
		uint8_t sum = _calc_checksum(buf, JY901_FRAME_LEN - 1);
		if (sum == buf[JY901_FRAME_LEN - 1])
		{
			/* 有效帧 */
			uint8_t id = buf[1];
			_parse_frame(id, &buf[2], JY901_FRAME_DATA_LEN);
		}            
    }
} 

/*----------------- 指令发送 -----------------*/

void jy901_send_unlock(void) 
{
	uint8_t cmdbuf[5]={0xFF,0xAA,0x69,0x88,0xB5};
	bsp_UART2_DMASend(cmdbuf,5);
}

void jy901_send_save_config(void) 
{
	uint8_t cmdbuf[5]={0xFF,0xAA,0x00,0x00,0x00};
	bsp_UART2_DMASend(cmdbuf,5);
}

void jy901_send_restore_default(void) 
{
    _send_command_single(0x00, 0x01);
}

void jy901_send_calibration(uint8_t mode)
{
    _send_command_single(0x01, mode);
}

void jy901_send_z_axis_reset(void)
{
    /* 特殊指令: FF AA 01 04 00 */
    _send_command(0x01, 0x04, 0x00);
}

void jy901_send_mount_direction(uint8_t vertical)
{
    if (vertical > 1U) {
        return;
    }
    _send_command_single(0x23, vertical);
}

void jy901_send_sleep_toggle(void)
{
    _send_command(0x22, 0x01, 0x00);
}

void jy901_send_algorithm(uint8_t algo)
{
    if (algo > 1U) {
        return;
    }
    _send_command_single(0x24, algo);
}

void jy901_send_gyro_autocal(bool enable)
{
    _send_command_single(0x63, enable ? 0x00 : 0x01);
}

void jy901_send_output_mask(uint8_t mask_l, uint8_t mask_h) 
{
    _send_command(0x02, mask_l, mask_h);
}
static uint16_t output_mask=0;
void jy901_send_output_packets(uint16_t packets) 
{
    packets &= JY901_OUTPUT_ALL;
	output_mask|=packets;
    jy901_send_output_mask((uint8_t)(output_mask & 0xFFU),(uint8_t)(output_mask >> 8));
}


static void jy901_clear_output_packets(uint16_t packets)
{
  packets &= JY901_OUTPUT_ALL;

  output_mask &= (uint16_t)(~packets);
  jy901_send_output_mask((uint8_t)(output_mask & 0xFFU),(uint8_t)(output_mask >> 8));
}


void jy901_send_output_rate(uint8_t rate)
{
    /* 0x0A 为手册保留值，不允许发送。 */
    if (!_is_valid_output_rate(rate)) 
	{
        return;
    }
    _send_command_single(0x03, rate);
}

void jy901_send_baudrate(uint8_t baud)
{
    if (baud > JY901_BAUD_921600) {
        return;
    }
    _send_command_single(0x04, baud);
}

void jy901_send_port_mode(uint8_t port, uint8_t mode)
{
	uint8_t reg;
    /* D0~D3 端口范围为 0~3，CLR 相对姿态模式仅 D1 支持。 */
    if ((port > 3U) || (mode > JY901_PORT_MODE_CLR)) {
        return;
    }
    if ((mode == JY901_PORT_MODE_CLR) && (port != 1U)) {
        return;
    }
    reg = 0x0E + port;
    _send_command_single(reg, mode);
}

void jy901_send_pwm_param(uint8_t port, uint16_t high_us, uint16_t period_us)
{
    uint8_t reg_high;
    uint8_t reg_period;	
    if (port > 3U) {
        return;
    }
    /* 手册寄存器表：D0~D3 的 PWMH 为 0x12~0x15，PWMT 为 0x16~0x19。 */
    reg_high = 0x12 + port;
    reg_period = 0x16 + port;
    
    _send_command(reg_high, (uint8_t)(high_us & 0xFF), (uint8_t)(high_us >> 8));
    _send_command(reg_period, (uint8_t)(period_us & 0xFF), (uint8_t)(period_us >> 8));
}

void jy901_send_iic_address(uint8_t addr) 
{
    /* JY901 IIC 地址为 7 位地址，最大不能超过 0x7F。 */
    if (addr > 0x7FU) {
        return;
    }
    _send_command_single(0x1A, addr);
}

void jy901_send_accel_offset(uint8_t axis, int16_t offset) 
{
	uint8_t reg;
    if (axis > 2U) {
        return;
    }
    reg = 0x05 + axis;
    _send_command(reg, (uint8_t)(offset & 0xFF), (uint8_t)(offset >> 8));
}

void jy901_send_gyro_offset(uint8_t axis, int16_t offset)
{
	uint8_t reg;
    if (axis > 2U) {
        return;
    }
    reg = 0x08 + axis;
    _send_command(reg, (uint8_t)(offset & 0xFF), (uint8_t)(offset >> 8));
}

void jy901_send_mag_offset(uint8_t axis, int16_t offset)
{
	uint8_t reg;
    if (axis > 2U)
	{
        return;
    }
    reg = 0x0B + axis;
    _send_command(reg, (uint8_t)(offset & 0xFF), (uint8_t)(offset >> 8));
}

/*----------------- 解析函数 -----------------*/

float jy901_parse_accel(int16_t raw, uint8_t range) 
{
    float scale;
    switch (range) 
	{
        case 2:  scale = 2.0f;  break;
        case 4:  scale = 4.0f;  break;
        case 8:  scale = 8.0f;  break;
        case 16:
        default: scale = 16.0f; break;
    }
    return (float)raw / 32768.0f * scale;
}

float jy901_parse_gyro(int16_t raw, uint16_t range) 
{
    float scale;
    switch (range) 
	{
        case 250:  
			scale = 250.0f; 
			break;
        case 500:  
			scale = 500.0f;
			break;
        case 1000:
			scale = 1000.0f;
			break;
        case 2000:
        default:
           scale = 2000.0f; 
		   break;
    }
    return (float)raw / 32768.0f * scale;
}



/*----------------- 数据获取 -----------------*/

const jy901_float_data_t* jy901_get_data(void)
{
    return &_float_data;
}

const jy901_time_t* jy901_get_time(void)
{
    return &_time_data;
}

const jy901_port_status_t* jy901_get_port_status(void)
{
    return &_port_data;
}

const jy901_baro_t* jy901_get_baro(void)
{
    /* 返回内部缓存地址，调用方只读使用，不需要释放。 */
    return &_baro_data;
}

const jy901_gps_t* jy901_get_gps(void)
{
    /* GPS 经纬度、地速、精度数据分多个包更新，结构体保存最新组合结果。 */
    return &_gps_data;
}

const jy901_quaternion_t* jy901_get_quaternion(void)
{
    /* 四元数由 0x59 包更新，数值已按手册换算为 -1~1 浮点量。 */
    return &_quat_data;
}

bool jy901_is_data_updated(void)
{
    return _data_updated;
}

void jy901_clear_update_flag(void) 
{
    _data_updated = false;
}
