//#include <stdio.h>
#include "TI_aes_128.h"
#include <stdint.h>
//#include <string.h>
#define polynomial 0xd008
#define initial_value 1

#define bits_to_zero_pad (16-3)

void copy_mem (uint8_t *dest, const uint8_t *source, uint8_t n);
uint16_t next_lfsr(uint16_t lfsr);
void generate_code(uint8_t command, uint8_t *generated_code);
uint8_t decode_code(uint8_t *command, uint8_t *generated_code);


const uint8_t key[16]   = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x5d, 0x0e, 0x0f};

//provide command to encrypt and get back a code
//generated_code must be uint8_t array with 16 elements
void generate_code(uint8_t command, uint8_t *generated_code){
	static uint16_t lfsr = initial_value;

	lfsr = next_lfsr(lfsr);
	//printf("lfsr %X\n",lfsr);
	generated_code[0] = (uint8_t)(lfsr >> 8); //store msb
	generated_code[1] = (uint8_t)(lfsr & 0xFF); //store lsb
	generated_code[2] = command;
	
	uint8_t loop;
	
	for (loop = (16-bits_to_zero_pad); loop < 16; loop++)	//zero pad
		generated_code[loop] = 0;	
	
	//uint8_t key[16] = {0};	//copy key into a scratch pad for the aes function
	//memcpy(key, key_orig, 16);
	//copy_mem(key, key_orig, 16);
	aes_enc_dec(generated_code,key,0);	//encrypt
	
}

//give a code and get back a command if its good
//return 1 if authenticated and valid code was found
uint8_t decode_code(uint8_t *command, uint8_t *generated_code){
	static uint16_t previous_lfsr = initial_value;
	
	//const uint8_t key_orig[16]   = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x5d, 0x0e, 0x0f};

	//uint8_t key[16] = {0};	//copy key into a scratch pad for the aes function
	//copy_mem(key, key_orig, 16);
	aes_enc_dec(generated_code,key,1);	//first decrypt

	uint16_t current_lfsr = (generated_code[0]<<8) + generated_code[1];

	if (next_lfsr(previous_lfsr) == current_lfsr){	//if our next lfsr code matches the recieved lfsr code
		if (generated_code[15] == 0){	//if 16th byte is a zero
			previous_lfsr = current_lfsr;	//store for the next time
			*command = generated_code[2];	//then we are good
			return(1);
		}			
	}
	previous_lfsr = current_lfsr;	//store for the next time
	
	command = 0;
	return(0);	
}

uint16_t next_lfsr(uint16_t lfsr){
	uint8_t lsb = lfsr & 1;  /* Get lsb (i.e., the output bit). */
	lfsr >>= 1;               /* Shift register */
	if (lsb == 1)             /* Only apply toggle mask if output bit is 1. */
		lfsr ^= polynomial;        /* Apply toggle mask, value has 1 at bits corresponding to taps, 0 elsewhere*/

	return(lfsr);
}

void copy_mem (uint8_t *dest, const uint8_t *source, uint8_t n){
	uint8_t i;
	for (i=0; i<n; i++)
		dest[i] = source[i];
}

