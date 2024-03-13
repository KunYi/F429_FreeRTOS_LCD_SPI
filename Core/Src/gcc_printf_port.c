#include <stm32f4xx.h>

int _write (int fd, char *pBuffer, int size)
{
  for(int i = 0; i < size; i++) {
	  do {
      ;
    } while((USART3->SR & 0x40) == 0);
	  USART3->DR = (uint8_t) pBuffer[i];
  }
  return size;
}


int __io_putchar(int ch)
{
  while ((USART3->SR & 0x40) == 0) {}
  USART3->DR = (uint8_t) ch;
  return ch;
}
