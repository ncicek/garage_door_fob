//contains the lfsr and encrypt/decrypt routines

#include "TI_aes_128.h"
#include "main.h"
#include <stdint.h>
#include "crypto.h"
#define polynomial 0xd008   //maximal lenght
#define initial_value 1

uint16_t previous_lfsrs[32];

//key is shared among reciever and all fobs
const unsigned char key[16]   = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x5d, 0x0e, 0x0f};

//provide command to encrypt and get back a code
//generated_code must be uint8_t array with 16 elements
void generate_code(uint8_t command, uint8_t *generated_code){
    #define bits_to_zero_pad (16-4) //modify as you more bytes of the generated_code
	static uint16_t lfsr = initial_value;

	lfsr = next_lfsr(lfsr);

	//construct the packet
	generated_code[0] = (uint8_t)(lfsr >> 8); //store msb
	generated_code[1] = (uint8_t)(lfsr & 0xFF); //store lsb
	generated_code[2] = command;
	generated_code[3] = get_chips_id();
	
	uint8_t loop;
	for (loop = (16-bits_to_zero_pad); loop < 16; loop++)	//zero pad
		generated_code[loop] = 0;	
	aes_enc_dec(generated_code,key,0);	//encrypt the constructed packet
	
}

//give a code and get back a command if its good
//return 1 if authenticated and valid code was found
uint8_t decode_code(uint8_t *command, uint8_t *generated_code){

	//static uint16_t previous_lfsr = initial_value;
	aes_enc_dec(generated_code,key,1);	//decrypt the packet

	uint8_t device_id = generated_code[3]; //pull out device id

	//sanity check first
	if (generated_code[15] != 0)
	    return (0);
	if (device_id >= 32)
	    return (0);

	uint16_t previous_lfsr = get_previous_lfsr(device_id); //call function to get the previous lfsr of this particular device



	uint16_t current_lfsr = (generated_code[0]<<8) + generated_code[1];

	if (next_lfsr(previous_lfsr) == current_lfsr){	//if our next lfsr code matches the recieved lfsr code
		if (generated_code[15] == 0){	//if 16th byte is a zero
			previous_lfsr = current_lfsr;	//store for the next time
			*command = generated_code[2];	//then we are good
			return(1);
		}			
	}
	
	save_current_lfsr(current_lfsr, device_id); //store for the next time
	//previous_lfsr = current_lfsr;	//store for the next time

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

//return a "unique" identifier of the chip
uint8_t get_chips_id(){
    return(1);

}


// pass it a device id and it will return that devices' previous_lfsr
//if device is unknown or something else, it will return garbage
uint16_t get_previous_lfsr(uint8_t device_id){
    return(previous_lfsrs[device_id]);
}

//write lfsr to ram and save ram to flash
void save_current_lfsr(uint16_t current_lfsr, uint8_t device_id){
    previous_lfsrs[device_id] = current_lfsr;
    write_to_flash(previous_lfsrs, 2*32);


}
