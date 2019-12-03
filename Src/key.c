#include "key.h"
#include "main.h"
extern uint8_t key0state,key1state,key0flag,key1flag;
uint8_t trg,cont;
void keyvalue(void)
{
	unsigned char tmp=(GPIOE->IDR)^0xff;
	trg=tmp&(tmp^cont);
	cont=tmp;	
	if((trg&0x0008)==0x0008)
	{
		key1flag=1;
	}
	if((trg&0x0010)==0x0010)
	{
		key0flag=1;
	}
}

