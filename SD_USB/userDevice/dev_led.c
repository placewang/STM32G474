#include <string.h>
#include <stdio.h>

#include "dev_led.h"
#include "../Terminal/shell_port.h"


#define LED_PORT  GPIOC
#define LED_PIN   GPIO_PIN_13 




void dev_led_toggle(void)
{
	HAL_GPIO_TogglePin(LED_PORT,LED_PIN);
}

void dev_led_on(void)
{
	HAL_GPIO_WritePin(LED_PORT,LED_PIN,0);
}


void dev_led_off(void)
{
	HAL_GPIO_WritePin(LED_PORT,LED_PIN,1);	
}

int dev_led_set(int argc, char *argv[])
{
	
    const char *cmd = argv[1];
    if (argc <2) 
	{
        logError("usage: led <on|off|toggle");
        return-1;
    }
    if (strcmp(cmd, "on") == 0) 
	{
		dev_led_on();
    }
    else if (strcmp(cmd, "off") == 0)
	{
		dev_led_off();
    }
    else if (strcmp(cmd, "toggle") == 0) 
	{
        dev_led_toggle();
    }
    else 
	{
        logWarning("unknown led command: %s\n", cmd);
    }
	return -1;
}

SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN)|SHELL_CMD_DISABLE_RETURN,
                 led,dev_led_set,led control);
				 
				 

