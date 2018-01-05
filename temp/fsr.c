//feedback shift register
//3 parts:
//rolling code
//command
//checksum

#include "TI_aes_128.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define polynomial 0xd008

#define initial_value 0x1




uint8_t state[16]  = {0x01, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f};  
const uint8_t key_orig[16]   = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f};

  
uint8_t command = 3;


int main(int argc, char* argv[])
{
	uint16_t decoded_lfsr;
    uint16_t lfsr = initial_value;
    uint32_t period = 0;
    char s[16+1];

    do {
		//for (uint8_t i=0;i<128;i++){
		uint8_t lsb = lfsr & 1;  /* Get lsb (i.e., the output bit). */
		lfsr >>= 1;               /* Shift register */
		if (lsb == 1)             /* Only apply toggle mask if output bit is 1. */
		lfsr ^= polynomial;        /* Apply toggle mask, value has 1 at bits corresponding
								/* to taps, 0 elsewhere. */
		printf("%d\n",lfsr);
		
		state[0] = (uint8_t)(lfsr >> 8); //store msb
		state[1] = (uint8_t)(lfsr & 0xFF); //store lsb
		state[2] = command;
		
		int loop;
		
		for (loop = 3; loop < 16; loop++)
			state[loop] = 0;	
		/*
		for(loop = 0; loop < 16; loop++)
			printf("%d ", state[loop]);
		printf("\n");
		*/
		uint8_t key[16];
		memcpy(key, key_orig, 16);
		
		aes_enc_dec(state,key,0);
		//memcpy(key, key_orig, 16);
		//aes_enc_dec(state,key,1);
		
		/*
		for(loop = 0; loop < 16; loop++)
			printf("%d ", state[loop]);
		printf("\n");
		*/
		decoded_lfsr = (state[0] << 8) + state[1];
		
		++period;

		int i;
		for (i = 0; i < 16; i++)
		{
			s[15 - i] = (decoded_lfsr & (1 << i)) ? '1' : '0';
		}
		
		s[16] = '\0';
		printf("\n%10d: %s", period, s);
		
    } while(decoded_lfsr != initial_value);
    //} while(period <= 2);

    return 0;
}

