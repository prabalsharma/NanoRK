#ifndef _SLIPSTREAM_H_
#define _SLIPSTREAM_H_


#define MAX_BUF    256

void error (char *);

int slipstream_open(char *addr, int port, int blocking_read);
int slipstream_send(char *buf, int size);
int slipstream_receive(char *buf);
//int slipstream_acked_send(char *buf, uint8_t len, uint8_t retries);

#endif
