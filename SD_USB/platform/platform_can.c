
#include "platform/platform_can_bridge.h"


static  platform_can_device *g_stm32_can1_bus=NULL;
static  platform_can_device *g_stm32_can2_bus=NULL;


int platform_can_bus_register(const platform_can_ops* bus, \
							  platform_can_device*    dev,
							  unsigned char canId)
{
	if (!bus || !dev)
		return -1;

	dev->ops=bus;
    if (canId == CAN1)
      g_stm32_can1_bus=dev;
    else if (canId == CAN2)
      g_stm32_can2_bus=dev;	
	return 0;
}

/* 可选：提供获取CAN设备实例的接口 */
platform_can_device* platform_can_get_device(unsigned char can_id)
{
    if (can_id == CAN1)
        return g_stm32_can1_bus;
    else if (can_id == CAN2)
        return g_stm32_can2_bus;
    return NULL;
}
/**/
int platform_can_send(platform_can_client *dev, 
					  const uint8_t *data,uint32_t len)
{
    if (!dev || !dev->bus || !dev->bus->ops->send|| !data)
        return -1;
    if(len > 8)
		len=8;
    dev->bus->ops->send(data, dev->sd_id, len);
    return 0;
}
/**/
int platform_can_receive(platform_can_client *dev, 
						 uint8_t *data,uint32_t* len)
{
    int  rv=0;
    if (!dev || !dev->bus || !dev->bus->ops->send || !data)
    { 
		return -1;
    }
	
	rv=dev->bus->ops->receive(data, &dev->rv_id, len);
	
    if(1==rv)
	{
		return 1;
	}
	else if(rv==-1)
	{
		return -1;
	}

    return 0;
}





