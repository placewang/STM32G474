#include <string.h>

#include "bsp_spi.h"
#include "app_HostComm.h"
#include "app_IapUpData.h"
#include "app_ProtocolAnalysis.h"

/*
接收数据包XOR校验和（处理任意长度，包括非4的倍数）
*/
uint32_t XORCheck(uint8_t *data, int len)
{
    uint32_t checksum = 0;

    int doubleWordLen = len / 4;
    uint32_t *doubleWordData = (uint32_t *)data;

    // 逐个 32-bit 字 XOR
    for (int i = 0; i < doubleWordLen; i++) {
        checksum ^= doubleWordData[i];
    }
    // 处理剩余不足 4 字节的部分（逐字节 XOR）
    int remaining = len % 4;
    for (int i = 0; i < remaining; i++) {
        checksum ^= data[doubleWordLen * 4 + i];
    }

    // qr_wl 协议特征：XOR 结果 + 0xAA
    return checksum + 0xAA;
}


void HostRxDataCopy(const uint8_t *data,const uint8_t len)
{
	if(len==FOOT_PACK_LEN)
	{
		memcpy(rxDataPack.bytes,data,len);
		HostComm.type=FOOT_DEV_TYPE;
		HostComm.RecFlag = 1;  
	}
	else if(len==WHEEL_PACK_LEN)
	{
	    memcpy(rxDataPack.bytes,data,len);
		HostComm.type=WHEEL_DEV_TYPE;
		HostComm.RecFlag = 1;  
	}	 
}

void app_ProtocolAnalysisTask_loop(void)
{
    uint8_t   block_data[WHEEL_PACK_LEN];
    uint8_t  *pdata = block_data;
	uint32_t  checksum=0;
	uint16_t  rx_len=0;
	
    /* 错误重启处理 */
	bsp_SPI1_Process();   
    /* ---- 处理 PING 块接收 ---- */
    if (SPI1_RxTx.rx_ping_ready == 1U)
    {
        /* 临界区拷贝，防止中断覆盖 */
        __disable_irq();
        memcpy(block_data, SPI1_RxTx.rx_shadow, SPI1_RxTx.rx_reality_cnt);
        rx_len=SPI1_RxTx.rx_reality_cnt;
		SPI1_RxTx.rx_reality_cnt = 0U;
		SPI1_RxTx.rx_ping_ready  = 0U;
        __enable_irq();

        /* 帧格式检查 */
        if (pdata[0] == FRAME_HEADER && pdata[rx_len-1] == FRAME_FOOTER)
        {
	        checksum = XORCheck(pdata, rx_len-5);
			if (checksum != *((uint32_t*)&pdata[rx_len-5]))
            {
				return;
			}
            /* 根据命令字节决定交给 Host 还是 IAP */
            if (Update_ACK > pdata[1])
            {
               HostRxDataCopy(pdata,rx_len);
            }
            else
            {/* IAP 数据帧 */
				memcpy(iapRxbuff.bytes, pdata, PACK_UPDATA_LEN);
				iap_ps.RecFlag = 1;
            }
        }
    }
}


















