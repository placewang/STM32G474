#ifndef _APP_HOSTCOMM_H__
#define _APP_HOSTCOMM_H__

#ifdef __cplusplus
extern "C" {
#endif

#define FRAME_HEADER     0xAA
#define FRAME_FOOTER     0xBB
#define DATA_BLOCK_SIZE  8
#define PACK_LEN         68

#define FOOT_DEV_TYPE     0
#define WHEEL_DEV_TYPE    1

#define FOOT_PACK_LEN     68
#define WHEEL_PACK_LEN    88

typedef struct
{
	char    type;            /*设备模式0-足式  1-轮式*/
	char    RecFlag;         /*数据到位标记*/ 
	int     can1_fillcnt;    /*can1回传数据填充计数*/
	int     can2_fillcnt;    /*can2回传数据填充计数*/
}Host_Comm;


// 使用 __attribute__((packed)) 或 #pragma pack(1) 禁止字节对齐
#pragma pack(push, 1)
// 协议帧结构体（按实际字节顺序排列）
typedef struct 
{
    unsigned char   header;                
    unsigned short  num;          
    unsigned short  can2Id_01;
    unsigned char   can2data_01[DATA_BLOCK_SIZE];
    unsigned short  can2Id_02;
    unsigned char   can2data_02[DATA_BLOCK_SIZE];
    unsigned short  can2Id_03;
    unsigned char   can2data_03[DATA_BLOCK_SIZE];
	
    unsigned short  can1Id_01;   
    unsigned char   can1data_01[DATA_BLOCK_SIZE];
    unsigned short  can1Id_02;
    unsigned char   can1data_02[DATA_BLOCK_SIZE];
    unsigned short  can1Id_03;
    unsigned char   can1data_03[DATA_BLOCK_SIZE];

    unsigned int    checksum; 
    unsigned char   footer;                
} FootFrameStruct;
#pragma pack(pop)


#pragma pack(push, 1)
// 协议帧结构体（按实际字节顺序排列）
typedef struct 
{
    unsigned char   header;                
    unsigned short  num;          
	
    unsigned short  can2Id_01;
    unsigned char   can2data_01[DATA_BLOCK_SIZE];
    unsigned short  can2Id_02;
    unsigned char   can2data_02[DATA_BLOCK_SIZE];
    unsigned short  can2Id_03;
    unsigned char   can2data_03[DATA_BLOCK_SIZE];
    unsigned short  can2Id_04;
    unsigned char   can2data_04[DATA_BLOCK_SIZE];
	
    unsigned short  can1Id_01;   
    unsigned char   can1data_01[DATA_BLOCK_SIZE];
    unsigned short  can1Id_02;
    unsigned char   can1data_02[DATA_BLOCK_SIZE];
    unsigned short  can1Id_03;
    unsigned char   can1data_03[DATA_BLOCK_SIZE];
    unsigned short  can1Id_04;
    unsigned char   can1data_04[DATA_BLOCK_SIZE];
	
    unsigned int    checksum; 
    unsigned char   footer;                
} wheelFrameStruct;
#pragma pack(pop)

// 共用体：支持字节数组到结构体的直接转换
typedef union {
	FootFrameStruct  Foot;	
    wheelFrameStruct Wheel;	
    unsigned char     bytes[WHEEL_PACK_LEN];
} Host_DataPack;


extern Host_Comm  HostComm;
 
extern  Host_DataPack    rxDataPack,txDataPack;


void app_HostData_init(void);
void app_HostDataPacketDisassembly_loop(void);



#ifdef __cplusplus
}
#endif

#endif
