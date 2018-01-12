//1.4 mosi
//1.5 miso
//1.6 sclk

//1.0 led
//1.2 csn (spi chip select)
//1.7 ce (Chip Enable Activates RX or TX mode)
//2.0 irq coming out of nrf (active low)

//2.7 button

//2.1 toggle_door fet (only for receiver) active high

#include <msp430.h>
#include <stdint.h>
#include "nRF24L01.h"
#include "NRF.h"
#include "radio_functions.h"
#include "main.h"

#define TX_MODE
//#define RX_MODE

//global vars
uint8_t return_byte;
volatile uint8_t bounce_locked = 0;
volatile uint8_t button_pressed = 0;

int main(void)
{
    WDTCTL = WDTPW + WDTHOLD;				  // Stop watchdog timer
    PM5CTL0 &= ~LOCKLPM5;

	//BCSCTL1 |= DIVA_0;                        // ACLK/2
    //BCSCTL3 |= LFXT1S_2;                      // ACLK = VLO

	//gpio for led csn and ce pins
	P1OUT = 0x00;							  // P1 setup for LED & reset output
	P1DIR |= BIT0 + BIT2 + BIT7;

	//setup pin 2.0 as irq nrf
	P2OUT &= ~BIT0;
	P2DIR &= ~BIT0;
    P2IES |= BIT0;                            // Hi/lo edge
    P2IFG &= ~BIT0;                           // IFG cleared
	P2IE |=  BIT0; 

    //setup pin 2.7 as button irq
    P2DIR &= ~BIT7;	//set as input
    P2IES |= BIT7;                            // P1.3 Hi/lo edge
    P2OUT |= BIT7;
    P2REN |= BIT7;                          // Enable Pull Up on
    P2IFG &= ~BIT7;                           // P1.3 IFG cleared
	P2IE |=  BIT7; 

    #if defined(RX_MODE)
    #endif


	//usci config
    P1DIR |= BIT4 + BIT5 + BIT6;
    P1SEL0 = BIT4 + BIT5+ BIT6;   //set the spi pins to spi mode



	UCA0CTLW0 |= UCSWRST;                     // **Put state machine in reset**
    UCA0CTLW0 |= UCMST|UCSYNC|UCCKPH|UCMSB;   // 3-pin, 8-bit SPI master
                                              // Clock polarity high, MSB
    UCA0CTLW0 |= UCSSEL__SMCLK;               // SMCLK
    UCA0BR0 = 0x01;                           // /2,fBitClock = fBRCLK/(UCBRx+1).
    UCA0BR1 = 0;                              //
    UCA0MCTLW = 0;                            // No modulation
    UCA0CTLW0 &= ~UCSWRST;                    // **Initialize USCI state machine**
    UCA0IE |= UCRXIE;                         // Enable USCI_A0 RX interrupt


	NRF_set_csn(1);
	NRF_set_ce(0);

	uint8_t i = 0;
    while (NRF_check_chip() != 1){  //dont do anything before verifying comms
        i++;
        if (i > 5)
            __bis_SR_register(LPM4_bits + GIE);  //give up to avoid wasting battery

    }

	NRF_write(NRF_STATUS, 0xE);
	NRF_cmd(FLUSH_TX);
	NRF_cmd(FLUSH_RX);

	//set button interrupt so we will wake up after sleeping
    #if defined(TX_MODE)
	TX_Mode(); //configure
                               // P1.3 interrupt enabled
	//after config, wait for button interupt
    #elif defined(RX_MODE)
	RX_Mode();  //configure
	listen();
    #endif

	while (1){
	    __bis_SR_register(LPM4_bits + GIE);   //sleep, wait for button int
	    if (button_pressed){
            bounce_locked = 1;
            transmit();
            bounce_locked = 0;
	    }
	    button_pressed = 0;
	}
}


void NRF_set_csn(uint8_t bit){
    if (bit == 1)
        P1OUT |= BIT2;
    else
        P1OUT &= ~BIT2;
}

void NRF_set_ce(uint8_t bit){
    if (bit == 1)
        P1OUT |= BIT7;
    else
        P1OUT &= ~BIT7;
}

void led (uint8_t i){
    if (i == 1)
        P1OUT |= BIT0;
    else
        P1OUT &= ~BIT0;
}

//here we map what commands do what on the reciever side
void act_on_command(uint8_t command){
    if (command == TOGGLE_DOOR){
        P2OUT |= BIT1;
        __delay_cycles(10000);
        P2OUT &= ~BIT1;
    }
}

uint8_t get_rx_buffer(){
    return (return_byte);
}

//erase 64 bytes and write up to 64 bytes into fram
void write_to_flash(uint16_t *dbl_byte_array, uint8_t len)
{
    #define start_addr 0x1800
    if (len <= 64)  //dont do anything if asked to overwrite too much
    {
        SYSCFG0 &= ~DFWP; //disable data_ram write protection

        uint8_t i;
        uint16_t *Fram_ptr = (uint16_t *) start_addr; // Initialize Flash segment D pointer

        *Fram_ptr = 0;                    // Dummy write to erase Flash segment


        for (i = 0; i < len; i++)
        {
            *Fram_ptr++ = dbl_byte_array[i]; // copy value segment C to segment D
        }
        SYSCFG0 |= DFWP;    //reenable data_ram write protection

    }
}

// Button and NRF IRQs
#pragma vector=PORT2_VECTOR
__interrupt void Port_2(void)
{
	if (P2IFG & BIT0){	//nrf irq
		P2IFG &= ~BIT0; 
		__bic_SR_register_on_exit(LPM4_bits);
	}
	if (P2IFG & BIT7){	//butotn irq
		P2IFG &= ~BIT7;                       
		if (bounce_locked == 0)
			button_pressed = 1;
		__bic_SR_register_on_exit(LPM4_bits);
	}
}

#pragma vector=USCI_A0_VECTOR
__interrupt void USCIA0RX_ISR(void)
{
	//onlt rx interupt is enabled so we know this is an rx int
    return_byte = UCA0RXBUF; //HAVE to read the buffer to automatically clear the UCAxRXIFG flag
    __bic_SR_register_on_exit(LPM0_bits);
}
