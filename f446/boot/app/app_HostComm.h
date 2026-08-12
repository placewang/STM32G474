#ifndef _APP_HOSTCOMM_H__
#define _APP_HOSTCOMM_H__

#ifdef __cplusplus
extern "C" {
#endif

#define __WaitPackTime  1000Ul  /*等待回报时间单位us*/

typedef enum
{
    MTR1_CAN1ID=0x01,
    MTR2_CAN1ID=0x02,  
    MTR3_CAN1ID=0x03,
    MTR1_CAN2ID=0x01,
    MTR2_CAN2ID=0x02,  
    MTR3_CAN2ID=0x03           
}MOTORID;

void appHostData_init(void);
void appHostData_PacketDisassembly_loop(void);
void appHostData_PackagingFillTxbuff_loop(void);

#ifdef __cplusplus
}
#endif

#endif
