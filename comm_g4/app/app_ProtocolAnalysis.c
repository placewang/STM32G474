#include <string.h>

#include "bsp_spi.h"
#include "app_HostComm.h"
#include "app_IapUpData.h"
#include "app_ProtocolAnalysis.h"

#define CMDTYPE_MCS  0x01
#define CMDTYPE_IAP  0x02
/*
接收数据包XOR校验和（处理任意长度，包括非4的倍数）
*/
uint32_t XORCheck(uint8_t *data, int len)
{
    uint32_t checksum = 0;
	int remaining =0;
    int doubleWordLen = len / 4;
    uint32_t *doubleWordData = (uint32_t *)data;

    // 逐个 32-bit 字 XOR
    for (int i = 0; i < doubleWordLen; i++)
	{
        checksum ^= doubleWordData[i];
    }
    // 处理剩余不足 4 字节的部分(逐字节 XOR)
    remaining = len % 4;
    for (int i = 0; i < remaining; i++)
	{
        checksum ^= data[doubleWordLen * 4 + i];
    }

    // qr_wl 协议特征：XOR 结果 + 0xAA
    return checksum + 0xAA;
}
static uint32_t get_32Byte(const uint8_t *ptr, size_t offset) {
    return ((uint32_t)ptr[offset])       |
           ((uint32_t)ptr[offset+1]<<8)  |
           ((uint32_t)ptr[offset+2]<<16) |
           ((uint32_t)ptr[offset+3]<<24);
}

static void HostRxDataCopy(const uint8_t *data,const uint8_t len)
{
	if(len==FOOT_PACK_LEN)
	{
		memcpy(rxDataPack.bytes,data,len);
		HostComm.type=FOOT_DEV_TYPE;
	}
	else if(len==WHEEL_PACK_LEN)
	{
	    memcpy(rxDataPack.bytes,data,len);
		HostComm.type=WHEEL_DEV_TYPE;
	}	 
}
static void setRxflag(const uint8_t f_len)
{
	if(f_len==FOOT_PACK_LEN)
	{
		HostComm.type=FOOT_DEV_TYPE;
		HostComm.RecFlag = 1;  
	}
	else if(f_len==WHEEL_PACK_LEN)
	{
		HostComm.type=WHEEL_DEV_TYPE;
		HostComm.RecFlag = 1;  
	}	 
}
void app_ProtocolAnalysisTask_loop(void)
{
    uint8_t  *pdata = NULL;	
	uint32_t  checksum=0;
	uint8_t  rx_len=0;
	uint8_t   cmdType=0;
    /* 错误重启处理 */
	bsp_SPI1_Process();   
    /* ---- 处理 PING 块接收 ---- */
    if (SPI1_RxTx.rx_ping_ready != 1U)
    {
		return;
	}  
	/* 临界区拷贝，防止中断覆盖 */
	__disable_irq();
	rx_len=SPI1_RxTx.rx_reality_cnt;
	SPI1_RxTx.rx_reality_cnt = 0U;
	SPI1_RxTx.rx_ping_ready  = 0U;
	/* 根据命令字节决定交给 Host 还是 IAP */
	if (Update_ACK > SPI1_RxTx.rx_shadow[1])
	{
	   cmdType=CMDTYPE_MCS;
	   HostRxDataCopy(SPI1_RxTx.rx_shadow,rx_len);
	   pdata=rxDataPack.bytes;
	}
	else if(Update_ACK <=SPI1_RxTx.rx_shadow[1]&&
			rx_len==PACK_UPDATA_LEN)
	{/* IAP 数据帧 */
		cmdType=CMDTYPE_IAP;
		memcpy(iapRxbuff.bytes, SPI1_RxTx.rx_shadow, PACK_UPDATA_LEN);
		pdata=iapRxbuff.bytes;
	}	
	else
	{
		__enable_irq();	
		return;
	}	
	__enable_irq();		
	/* 帧格式检查 */
	if (pdata[0] == FRAME_HEADER && pdata[rx_len-1] == FRAME_FOOTER)
	{	
		checksum = XORCheck(pdata, rx_len-5);
		if (checksum != get_32Byte(&pdata[rx_len-5],0))
		{
			return;
		}
		if(cmdType==CMDTYPE_MCS)
		{
			setRxflag(rx_len);
		}
		else if(cmdType==CMDTYPE_IAP)
		{
			iap_ps.RecFlag = 1;
		}	
    }			
}














