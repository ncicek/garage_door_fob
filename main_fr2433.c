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
#include "crypto.h"

#define TOGGLE_DOOR 1
#define TX_MODE
//#define RX_MODE


#define TX_PLOAD_WIDTH  16  // 16 unsigned chars TX payload
uint8_t tx_buf[TX_PLOAD_WIDTH] = {0};

//prototypes
void act_on_command(uint8_t command);
void led (uint8_t i);
void Common_NRF_Config(void);
void TX_Mode(void);
void RX_Mode(void);
void listen();
void transmit();
void blink_led();

//global vars
uint8_t bounce_locked = 0;
uint8_t button_pressed = 0;
uint8_t debug_count = 0;

int main(void)
{

    WDTCTL = WDTPW + WDTHOLD;				  // Stop watchdog timer

	//BCSCTL1 |= DIVA_0;                        // ACLK/2
    //BCSCTL3 |= LFXT1S_2;                      // ACLK = VLO

	//gpio for led csn and ce pins
	P1OUT = 0x00;							  // P1 setup for LED & reset output
	P1DIR |= BIT0 + BIT2 + BIT7;

	//setup pin 2.0 as irq nrf
	P2OUT &= ~BIT0;
	P2DIR &= ~BIT0;
	P2IE &=  ~BIT0;                            // interrupt disabled, we dont want int yet
    P2IES |= BIT0;                            // Hi/lo edge
    P2IFG &= ~BIT0;                           // IFG cleared

    //setup pin 2.7 as button irq
    P2DIR &= ~BIT7;	//set as input
    P2IES |= BIT7;                            // P1.3 Hi/lo edge
    P2OUT |= BIT7;
    P2REN |= BIT7;                          // Enable Pull Up on
    P2IFG &= ~BIT7;                           // P1.3 IFG cleared

    #if defined(RX_MODE)
    P2DIR |= BIT1;
    #endif

    P1SEL0 |= BIT4 + BIT5+ BIT6;	  //set the spi pins to spi mode
    //P1SEL1 = BIT1 + BIT2 + BIT4;  //set the spi pins to spi mode	//not for fram part

	//usci config
    UCA0CTLW0 |= UCSWRST; //set reset before configuring
    UCA0CTLW0 |= UCMSB + UCMST + UCSYNC + UCCKPH;	 // 3-pin, 8-bit SPI master
    UCA0CTLW0 |= UCSSEL__SMCLK; 					  // smclk
	UCA0BRW |= 0x010;						  // /2
	UCA0MCTLW = 0;
	UCA0CTLW0 &= ~UCSWRST; 					// **Initialize USCI state machine**
	UCA0IE |= UCRXIE; 						  // Enable USCI0 RX interrupt

	PM5CTL0 &= ~LOCKLPM5;
	//flash controller
	 //DCOCTL = CALDCO_1MHZ;
	  //FCTL2 = FWKEY + FSSEL0 + FN1;             // MCLK/3 for Flash Timing Generator

	

	NRF_set_csn(1);
	NRF_set_ce(0);
	while (NRF_check_chip() != 1);	//dont do anything before verifying comms
	NRF_write(NRF_STATUS, 0xE);
	NRF_cmd(FLUSH_TX);
	NRF_cmd(FLUSH_RX);

    #if defined(TX_MODE)
	TX_Mode(); //configure
    P2IE |=  BIT7;                            // P1.3 interrupt enabled
	//after config, wait for button interupt
    #elif defined(RX_MODE)
	RX_Mode();  //configure
	listen();
    #endif

	while (1){
	    __bis_SR_register(LPM4_bits + GIE);   //sleep, wait for button int
	    if (button_pressed){
            //P1IE &= ~BIT3;  //disable the button interrupt (a way of debouncing)
            bounce_locked = 1;
            debug_count++;
            transmit();
            bounce_locked = 0;
            //P1IFG &= ~BIT3;
            //P1IE |=  BIT3;  //after transmitting, reenable interrupt
	    }
	    button_pressed = 0;
	}
    //int k=0;

}

void led (uint8_t i){
    if (i == 1)
        P1OUT |= BIT0;
    else
        P1OUT &= ~BIT0;
}

void Common_NRF_Config(void){
    #define TX_ADR_WIDTH    5   // 5 unsigned chars TX(RX) address width
    uint8_t TX_ADDRESS[TX_ADR_WIDTH] = {0x34,0x43,0x10,0x10,0x01};
    NRF_set_ce(0);         //set ce low

    NRF_write_buf(TX_ADDR + W_REGISTER, TX_ADDRESS, TX_ADR_WIDTH);    // Writes TX_Address to nRF24L01
    NRF_write(SETUP_RETR, (ARD*10+ARC*10)); // 500us + 86us, 10 retransmit
    NRF_write_buf(RX_ADDR_P0 + W_REGISTER, TX_ADDRESS, TX_ADR_WIDTH); // RX_Addr0 same as TX_Adr for Auto.Ack
    NRF_write(EN_AA, ENAA_P0);      // Enable Auto.Ack:Pipe0
    NRF_write(EN_RXADDR, ERX_P0);  // Enable Pipe0
    NRF_write(RF_CH, 40);        // Select RF channel 40
    NRF_write(RF_SETUP, (RF_PWR_LOW + RF_PWR_HIGH + RF_DR_LOW));   // TX_PWR:0dBm, Datarate:250kbps (0x26)

}

void TX_Mode(void)	//configure into tx mode
{
    Common_NRF_Config();

	NRF_write(NRF_CONFIG, (CRCO + EN_CRC ));
	NRF_clear_status();
}

void RX_Mode(void){
    Common_NRF_Config();
    NRF_write(NRF_CONFIG, (CRCO + EN_CRC + PRIM_RX));
    NRF_clear_status();
}

void listen(){
    uint8_t recieved_code[16];
    uint8_t recieved_command;
    NRF_power_mode(1); //power up
    NRF_set_ce(1); //must be high to listen

    //now sleep and wait for irq indicating recived packet
    P2IE |=  BIT0; //set interrupt enable so we will wake up after irq
    while(1){
        __bis_SR_register(LPM4_bits + GIE);  //sleep wake up on irq
        uint8_t status = NRF_read_status();
        if (status & RX_DR) {
            NRF_read_buf(R_RX_PAYLOAD, recieved_code, 16);    //load the recieved payload into code
            NRF_cmd(FLUSH_RX);

            if (decode_code(&recieved_command, recieved_code) == 1){ //call decode_code on the recieved code to see if it is valid
                act_on_command(recieved_command);
            }
        }
        NRF_clear_status(); //clears all flags after each wakeup such as irq bit
    }
}

//here we map what commands do what on the reciever side
void act_on_command(uint8_t command){
    if (command == TOGGLE_DOOR){
        P2OUT |= BIT1;
        __delay_cycles(10000);
        P2OUT &= ~BIT1;
    }


}
void transmit(){
    uint8_t code[16];
    generate_code(101, code);

    NRF_cmd(FLUSH_TX);
    NRF_clear_status(); //clears all flags such as irq bit
    NRF_write_buf(W_TX_PAYLOAD, code, 16);  //write the code as payload into the nrf24
    NRF_power_mode(1); //power up
    NRF_set_ce(1);  //transmit
    //__delay_cycles(100000);
    P2IE |=  BIT0; //set interrupt so we will wake up after sleeping
    __bis_SR_register(LPM4_bits + GIE);  //sleep wake up on irq
    NRF_set_ce(0);  //stop transmitting
    NRF_power_mode(0); //power down radio

    uint8_t lost_packets = NRF_read(OBSERVE_TX)&0xf;
    uint8_t status = NRF_read_status();

    if (status&TX_DS){
        blink_led();
    }
    else{

    }
}

void blink_led(){ //sets led high and sets timer to turn itoff after a while
    led(1);
    __delay_cycles(100);
    led(0);
}


//pass an uint8_t array and how many bytes are in the array
/*
void write_SegA (uint8_t *data, uint8_t num_bytes)
{
  uint8_t *Flash_ptr;                          // Flash pointer
  uint8_t i;

  Flash_ptr = (uint8_t *) 0x010FF;              // Initialize Flash pointer
  FCTL1 = FWKEY + ERASE;                    // Set Erase bit
  FCTL3 = FWKEY;                            // Clear Lock bit
  *Flash_ptr = 0;                           // Dummy write to erase Flash segment

  FCTL1 = FWKEY + WRT;                      // Set WRT bit for write operation

  for (i=0; i<num_bytes; i++)
  {
    *Flash_ptr++ = data[i];                   // Write value to flash
  }

  FCTL1 = FWKEY;                            // Clear WRT bit
  FCTL3 = FWKEY + LOCK;                     // Set LOCK bit
}
*/



// IRQ NRF 2.0
// Button IRQ 2.7
#pragma vector=PORT2_VECTOR
__interrupt void Port_2(void)
{
    if (P2IFG & BIT0){
        P2IFG &= ~BIT0;
        __bic_SR_register_on_exit(LPM4_bits);
    }
    if (P2IFG & BIT7){
        P2IFG &= ~BIT7;
       if (bounce_locked == 0)
           button_pressed = 1;
       __bic_SR_register_on_exit(LPM4_bits);
    }

}
