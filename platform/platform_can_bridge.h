#ifndef PLATFORM_PLATFORM_CAN_BRIDGE_H
#define PLATFORM_PLATFORM_CAN_BRIDGE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>

#define CAN1 1
#define CAN2 2

/* ops 表抽象 (子类填写) */
typedef struct 
{
	void (*send)(const uint8_t * data,const uint32_t id, uint32_t len);
	int  (*receive)(uint8_t *data,uint32_t *id, uint32_t *len);
}platform_can_ops ;


typedef struct  
{
	const platform_can_ops *ops;
	uint32_t  ErrCode;	
}platform_can_device;

typedef struct  
{
	platform_can_device *bus;  
    uint32_t  sd_id;
	uint32_t  rv_id;

}platform_can_client;


void _stm32_CAN_Init(void);
void platform_hw_can_init(void);

int platform_can_bus_register(const platform_can_ops* bus, \
							  platform_can_device*    dev, \
							  unsigned char canId);
int platform_can_send(platform_can_client *dev, 
					  const uint8_t *data,uint32_t len);	
int platform_can_receive(platform_can_client *dev, 
						 uint8_t *data,uint32_t* len);				  
						  
platform_can_device* platform_can_get_device(unsigned char can_id);

#ifdef __cplusplus
}
#endif

#endif /* PLATFORM_PLATFORM_CAN_H */
