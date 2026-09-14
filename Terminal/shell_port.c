#include "initcall.h"
#include "shell.h"
#include "../platform/platform_uart_bridge.h"





static SHELL_TypeDef shell_user;
#define SHELL_RX_BATCH_SIZE 64U
#define SHELL_TX_BUFFER_SIZE 64U
static uint8_t shell_tx_buffer[SHELL_TX_BUFFER_SIZE];
static uint32_t shell_tx_length;


static signed char shell_read(char *data)
{
	return platform_uart_receive(&uart1_c, (uint8_t *)data, 1U) == 1 ? 0 : -1;
}

static void shell_flush(void)
{
	if (shell_tx_length != 0U) {
		(void)platform_uart_send(&uart1_c, shell_tx_buffer, shell_tx_length);
		shell_tx_length = 0U;
	}
}

static void shell_write(const char data)
{
	shell_tx_buffer[shell_tx_length++] = (uint8_t)data;
	if (shell_tx_length == SHELL_TX_BUFFER_SIZE) {
		shell_flush();
	}
}


void shell_init(void)
{
	shell_user.read = shell_read;
	shell_user.write = shell_write;
	shellInit(&shell_user);
	shell_flush();
}

/*
 * 裸机模式下由主循环反复调用，驱动 Shell 读取并处理输入。
 */
void shell_process(void)
{
	char rx_buffer[SHELL_RX_BATCH_SIZE];
	int received = platform_uart_receive(&uart1_c,
		(uint8_t *)rx_buffer, SHELL_RX_BATCH_SIZE);

	for (int i = 0; i < received; ++i) {
		shellInput(&shell_user, rx_buffer[i]);
	}
	shell_flush();
}

MODULE_INIT(shell_init, INIT_LEVEL_LATE)



