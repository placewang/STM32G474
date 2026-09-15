#ifndef PLATFORM_PLATFORM_UART_BRIDGE_H
#define PLATFORM_PLATFORM_UART_BRIDGE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>

typedef enum
{
	UART_1=1,
	UART_2,	
}UART_NUM;



typedef struct 
{

	int (*send)(const uint8_t * data, uint32_t len);
	int  (*receive)(uint8_t *data, uint32_t len);

}platform_uart_ops;


typedef struct  
{
	const platform_uart_ops *ops;
	uint32_t  ErrCode;	

}platform_uart_device;

typedef struct  
{
	platform_uart_device *bus;  

}platform_uart_client;





int platform_uart_bus_register(const platform_uart_ops* bus, \
							  platform_uart_device*    dev,
							  UART_NUM num);

platform_uart_device* platform_uart_get_device(UART_NUM UART_Handle_num);

int platform_uart_send(platform_uart_client *dev, 
					  const uint8_t *data,uint32_t len);
/**/
int platform_uart_receive(platform_uart_client *dev, 
						 uint8_t *data,uint32_t len);


extern  platform_uart_client  uart1_c;

#ifdef __cplusplus
}
#endif

#endif 
