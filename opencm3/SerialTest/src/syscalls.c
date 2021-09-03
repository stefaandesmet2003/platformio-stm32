#include <libopencm3/stm32/usart.h>

int _read( int file, char *ptr, int len ) {

    return 0;
}

int _write( int file, char *ptr, int len ) {
    int i;
    
    for (i=0;i<len;i++) {
        usart_send_blocking(USART1,ptr[i]);
    }
    
    return len;
}