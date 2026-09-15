
#include "platform_uart_bridge.h"



static platform_uart_device *g_stm32_uart1_bus=NULL;



int platform_uart_bus_register(const platform_uart_ops* bus, \
							   platform_uart_device*    dev,
							   UART_NUM num)
{
	if (!bus || !dev)
		return -1;

	dev->ops=bus;
    if (num == UART_1)
      g_stm32_uart1_bus=dev;

	return 0;
}

/* 可选：提供获取CAN设备实例的接口 */
platform_uart_device* platform_uart_get_device(UART_NUM UART_Handle_num)
{
    if (UART_Handle_num == UART_1)
        return g_stm32_uart1_bus;

    return NULL;
}
/**/
int platform_uart_send(platform_uart_client *dev, 
					  const uint8_t *data,uint32_t len)
{
    if (!dev || !dev->bus || !dev->bus->ops->send|| !data)
        return -1;

    dev->bus->ops->send(data, len);
    return 0;
}
/**/
int platform_uart_receive(platform_uart_client *dev, 
						 uint8_t *data,uint32_t len)
{
    int  rv=0;
    if (!dev || !dev->bus || !dev->bus->ops->receive || !data)
    { 
		return -1;
    }
	
	rv=dev->bus->ops->receive(data, len);
	
    return rv;
}





