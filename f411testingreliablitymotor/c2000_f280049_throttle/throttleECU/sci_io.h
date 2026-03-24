#ifndef SCI_IO_H
#define SCI_IO_H

#include <stdint.h>

void SciIo_init(void);
void SciIo_print(const char *s);
void SciIo_printLine(const char *s);
int SciIo_readLine(char *buf, uint32_t maxLen);

#endif
