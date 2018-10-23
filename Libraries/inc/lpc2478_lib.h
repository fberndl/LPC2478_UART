#ifndef __lpc2478_lib_H
#define __lpc2478_lib_H

void delayMs(unsigned short delayInMs);
void setPortPinDir(char port, char pin);
void setPortPin(char port, char pin);
void clrPortPin(char port, char pin);

#endif
