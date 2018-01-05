//1.2 mosi
//1.1 miso
//1.4 sclk

//1.0 led
//1.6 csn (spi chip select)
//1.7 ce (Chip Enable Activates RX or TX mode)
//2.0 irq coming out of nrf (active low)
//2.1 toggle_door fet (only for receiver) active high
//1.3 button

#include <msp430.h>
#include <stdint.h>
#include "nRF24L01.h"
#include "NRF.h"
#include "radio_functions.h"

#define TOGGLE_DOOR 1
#define TX_MODE
//#define RX_MODE

//prototypes
void act_on_command(uint8_t command);
void led (uint8_t i);


//global vars
uint8_t return_byte;
uint8_t bounce_locked = 0;
uint8_t button_pressed = 0;

int main(void)
{
    WDTCTL = WDTPW + WDTHOLD;				  // Stop watchdog timer

	//BCSCTL1 |= DIVA_0;                        // ACLK/2
    //BCSCTL3 |= LFXT1S_2;                      // ACLK = VLO

	//gpio for led csn and ce pins
	P1OUT = 0x00;							  // P1 setup for LED & reset output
	P1DIR |= BIT0 + BIT6 + BIT7;

	//setup pin 2.0 as irq nrf
	P2OUT = 0x00;
	P2DIR &= ~BIT0;
	P2IE |=  BIT0;                            // interrupt enable
    P2IES |= BIT0;                            // Hi/lo edge
    P2IFG &= ~BIT0;                           // IFG cleared

    //setup pin 1.3 as button irq
    P1DIR &= ~BIT3;
    P1IES |= BIT3;                            // P1.3 Hi/lo edge
    P1OUT |= BIT3;
    P1REN |= BIT3;                          // Enable Pull Up on
    P1IFG &= ~BIT3;                           // P1.3 IFG cleared
	P1IE |=  BIT3; 
	
    #if defined(RX_MODE)
    #endif


	//usci config
    P1SEL = BIT1 + BIT2 + BIT4;	  //set the spi pins to spi mode
	P1SEL2 = BIT1 + BIT2 + BIT4;  //set the spi pins to spi mode


	//usci config
	UCA0CTL1 |= UCSWRST; //set reset before configuring
	UCA0CTL0 |= UCMSB + UCMST + UCSYNC + UCCKPH;	 // 3-pin, 8-bit SPI master
	UCA0CTL1 |= UCSSEL_2;					  // smclk
	UCA0BR0 |= 0x00;						  // /2
	UCA0BR1 = 0;							  //
	UCA0MCTL = 0;							  // No modulation
	UCA0CTL1 &= ~UCSWRST;					// **Initialize USCI state machine**
	IE2 |= UCA0RXIE;						  // Enable USCI0 RX interrupt

	//flash controller
	//DCOCTL = CALDCO_1MHZ;
	//FCTL2 = FWKEY + FSSEL0 + FN1;             // MCLK/3 for Flash Timing Generator
	
	NRF_set_csn(1);
	NRF_set_ce(0);
	while (NRF_check_chip() != 1);	//dont do anything before verifying comms
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
        P1OUT |= BIT6;
    else
        P1OUT &= ~BIT6;
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

// NRF IRQs
#pragma vector=PORT2_VECTOR
__interrupt void Port_2(void)
{
  P2IFG &= ~BIT0;                           // P2.0 IFG cleared
  __bic_SR_register_on_exit(LPM4_bits);
}

// Button IRQ
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
{
    P1IFG &= ~BIT3;                           // P1.3 IFG cleared
    if (bounce_locked == 0)
        button_pressed = 1;
    __bic_SR_register_on_exit(LPM4_bits);
}

#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCIA0RX_ISR(void)
{
	//onlt rx interupt is enabled so we know this is an rx int
    return_byte = UCA0RXBUF; //HAVE to read the buffer to automatically clear the UCAxRXIFG flag
    __bic_SR_register_on_exit(LPM0_bits);
}
