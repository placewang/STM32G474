

#ifndef  _UPGRADE_H
#define  _UPGRADE_H


typedef void (*pFunction)(void);

// FLASH 升级标志
#define CMD_UPGRADE_START		0xA050AA00	  /* 启动升级的标志*/
#define CMD_UPGRADE_RCV_FAIL	0xA050AA55	  /* 接收升级数据失败*/
#define CMD_UPGRADE_RCV_SUCCESS	0xAA5A5A5A	  /* 接收升级数据完成*/
#define CMD_UPGRADE_BURN	    0xA0505A5A	  /* 搬运数据标志*/
#define CMD_UPGRADE_SUCCESS		0x55AA55AA	  /* 升级完成标志*/
#define CMD_UPGRADE_FAIL		0xBBAA55AA	  /* 升级完成失败*/

// FLASH 地址定义
#define FLASH_APP_START_ADDRESS_APP       0x0800C000    // APP跳转A区地址
#define FLASH_APP_START_ADDRESS_BACKUP    0x08040000    // 备份区
#define BOOT_FLASH_UPGRADE_ADDRESS        0x08008000     //标志存放地址










void GoApp(unsigned int); 
void Upgrade_Process(void);
void Upgrade_getInformation(void);


#endif






























