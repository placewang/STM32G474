#include <stdarg.h>
#include <stdio.h>      
#include "initcall.h"
#include "shell_port.h"
#include "../platform/platform_uart_bridge.h"


#define SHELL_RX_BATCH_SIZE 64U
#define SHELL_TX_BUFFER_SIZE 64U

static Shell shell_user;
static char shell_buffer[512];
static uint8_t shell_tx_buffer[SHELL_TX_BUFFER_SIZE];
static uint32_t shell_tx_length;
static unsigned char shell_last_was_newline;                 /* 用于过滤上一次跨函数调用的重复 CRLF*/
static Log Loginfo;


static signed short shell_read(char *data, unsigned short len)
{
  return (signed short)platform_uart_receive(
	  &uart1_c,
	  (uint8_t *)data,
	  len
  );
}

static signed short shell_write(char *data, unsigned short len)
{
  unsigned short i;

  for (i = 0; i < len; i++) {
    /*
     * Shell 使用 CRLF（"\\r\\n"）换行。若上一段输出已经结束于 LF，
     * 当前又开始一个完整的 CRLF，则丢弃当前 CRLF，避免产生空白行。
     * 这里只过滤连续的完整 CRLF，不会删除普通的单独 CR 或 LF。
     */
    if (shell_last_was_newline &&
        data[i] == '\r' &&
        (i + 1U < len) &&
        data[i + 1U] == '\n') {
      i++;
      continue;
    }

    shell_tx_buffer[shell_tx_length++] = (uint8_t)data[i];
    shell_last_was_newline = (data[i] == '\n') ? 1U : 0U;

    /* 发送缓冲区满后立即通过 UART DMA 发送，避免溢出。 */
    if (shell_tx_length == SHELL_TX_BUFFER_SIZE) {
      if (platform_uart_send(&uart1_c, shell_tx_buffer, shell_tx_length) != 0) {
        shell_tx_length = 0U;
        return -1;
      }
      shell_tx_length = 0U;
    }
  }

  /* 本次回调结束时发送剩余数据，保持 Shell 输出及时可见。 */
  if (shell_tx_length != 0U) {
    if (platform_uart_send(&uart1_c, shell_tx_buffer, shell_tx_length) != 0) {
      shell_tx_length = 0U;
      return -1;
    }
    shell_tx_length = 0U;
  }

  return (signed short)len;
}





void shell_init(void)
{
	shell_user.read = shell_read;
	shell_user.write = shell_write;
	shellInit(&shell_user, shell_buffer, sizeof(shell_buffer));
	Loginfo.active=1;
	Loginfo.level=LOG_ALL;
	Loginfo.write=(void (*)(char *, short))shell_write;
	logRegister(&Loginfo, NULL);
	logError("void shell_init(void)");
	logPrintln("void shell_init(void)");
	logWarning("void shell_init(void)");
	logInfo("void shell_init(void)");

}

/*
 * 裸机模式下由主循环反复调用，驱动 Shell 读取并处理输入。
 */
void shell_process(void)
{
  shellTask(&shell_user);

}

MODULE_INIT(shell_init, INIT_LEVEL_LATE)



