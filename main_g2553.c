//1.2 mosi
//1.1 miso
//1.4 sclk

//2.3 led
//1.6 csn (spi chip select)
//1.7 ce (Chip Enable Activates RX or TX mode)
//2.0 irq coming out of nrf (active low)
//2.4 toggle_door fet (only for receiver) active high
//1.3 button

#include <msp430.h>
#include <stdint.h>
#include "nRF24L01.h"
#include "NRF.h"
#include "radio_functions.h"
#include "main.h"
#include "crypto.h"


//global vars / flag
volatile uint8_t button_pressed = 0;
volatile uint8_t debounce_timer = 0;
volatile uint8_t timer_armed = 0;
volatile uint8_t return_byte;

int main(void)
{
    WDTCTL = WDTPW + WDTHOLD;				  // Stop watchdog timer

    P1DIR = 0xFF;
    P2DIR = 0xFF;
    P3DIR = 0xFF;
    P1OUT = 0;
    P2OUT = 0;
    P3OUT = 0;


#if defined(TX_MODE)    //transmitter runs at 1mhz
    DCOCTL = CALDCO_1MHZ;
    BCSCTL1 = CALBC1_1MHZ;                    // Set DCO to 1MHz
#elif defined(RX_MODE)      //reciver runs at 8mhz
    DCOCTL = CALDCO_8MHZ;
    BCSCTL1 = CALBC1_8MHZ;                    // Set DCO to 8MHz
#endif

    BCSCTL3 |= LFXT1S_2;                 // MUST ENABLE VLO if you wanna use it!

#if defined(TX_MODE)
    __delay_cycles(10000); // wait for power to stabilize
#elif defined(RX_MODE)
    __delay_cycles(200000); // wait for power to stabilize
#endif

    //gpio for csn and ce pins
    P1DIR |= BIT0 + BIT6 + BIT7; //csn ce

#if defined(TX_MODE)
    P1DIR |= BIT0; //led
#elif defined(RX_MODE)
            P2DIR |= BIT3 + BIT4; //led relay
#endif

    //setup pin 2.0 as irq nrf
    P2DIR &= ~BIT0;
    P2IE |= BIT0;                            // interrupt enable
    P2IES |= BIT0;                            // Hi/lo edge
    P2IFG &= ~BIT0;                           // IFG cleared

    //setup pin 1.3 as button irq
    P1DIR &= ~BIT3;
    P1OUT |= BIT3;
    P1IES |= BIT3;                            // P1.3 Hi/lo edge
    P1REN |= BIT3;                          // Enable Pull Up on
    P1IFG &= ~BIT3;                           // P1.3 IFG cleared

    //usci config
    P1SEL = BIT1 + BIT2 + BIT4;	  //set the spi pins to spi mode
    P1SEL2 = BIT1 + BIT2 + BIT4;  //set the spi pins to spi mode

    //usci config
    UCA0CTL1 |= UCSWRST; //set reset before configuring
    UCA0CTL0 |= UCMSB + UCMST + UCSYNC + UCCKPH;	 // 3-pin, 8-bit SPI master
    UCA0CTL1 |= UCSSEL_2;					  // smclk
    UCA0BR0 |= 0x02;						  // /2
    UCA0BR1 = 0;							  //
    UCA0MCTL = 0;							  // No modulation
    UCA0CTL1 &= ~UCSWRST;					// **Initialize USCI state machine**
    IE2 |= UCA0RXIE;						  // Enable USCI0 RX interrupt

#if defined(RX_MODE)
    init_flash_controller();
#endif

    NRF_set_csn(1);
    NRF_set_ce(0);
    uint8_t i = 0;
    while (NRF_check_chip() != 1)
    {  //dont do anything before verifying comms
        i++;
        if (i > 5)
            __bis_SR_register(LPM4_bits + GIE); //give up to avoid wasting battery

    }
    blink_led();
    NRF_write(NRF_STATUS, 0xE);
    NRF_cmd(FLUSH_TX);
    NRF_cmd(FLUSH_RX);

    //set button interrupt so we will wake up after sleeping
#if defined(TX_MODE)
    TX_Mode(); //configure
    generate_seed();
#elif defined(RX_MODE)
    RX_Mode();  //configure
    load_lfsrs_into_ram();//load up the previous lfsr database into ram from flash mem
    listen();
#endif

#define wait_for_button 0
#define wait_for_debounce_timer 1
    uint8_t state = wait_for_button;
    while (1)
    {
      while (timer_armed == 1){
          __bis_SR_register(LPM3_bits + GIE); // sleep with smclk on for timer to expire
      }
        if (state == wait_for_button)
        {
            P1IE |= BIT3; //enable button int
            while (button_pressed == 0){
                __bis_SR_register(LPM4_bits + GIE);   //sleep, wait for button int
            }
            button_pressed = 0;
            P1IE &= ~BIT3;    //disable own interrupt
            state = wait_for_debounce_timer;
        }
        else if (state == wait_for_debounce_timer)
        {
            //start timer
            TA0CCTL0 = CCIE;                        // TA0 CCTL0
            TA0CCR0 = 55;                        // TA0 CCR0 value
            TA0CTL = TASSEL_1 + ID_3 + MC1 + TACLR + TAIE;
            while (debounce_timer == 0){
                __bis_SR_register(LPM3_bits + GIE);
            }
            debounce_timer = 0;
            TA0CTL = 0;                           // stop the timer
            if (~((P1IN & BIT3) > 0))    //if the button is still pressed
                transmit();
            state = wait_for_button;
        }
    }
}

inline void NRF_set_csn(uint8_t bit)
{
    if (bit == 1)
        P1OUT |= BIT6;
    else
        P1OUT &= ~BIT6;
}

inline void NRF_set_ce(uint8_t bit)
{
    if (bit == 1)
        P1OUT |= BIT7;
    else
        P1OUT &= ~BIT7;
}

inline void led(uint8_t i)
{
    if (i == 1)
    {
#if defined(TX_MODE)
        P1OUT |= BIT0;
#elif defined(RX_MODE)
        P2OUT |= BIT3;
#endif
    }
    else
    {
#if defined(TX_MODE)
        P1OUT &= ~BIT0;
#elif defined(RX_MODE)
        P2OUT &= ~BIT3;
#endif
    }
}

inline void relay(uint8_t i)
{
    if (i == 1)
    {
        P2OUT |= BIT4;
    }
    else
    {
        P2OUT &= ~BIT4; //relay off
    }
}

//here we map what commands do what on the reciever side
void act_on_command(uint8_t command, uint16_t time)
{
    if (command == TOGGLE_DOOR)
    {
        led(1);
        relay(1);
        arm_timer1(time);
        //timer isr will shut off the led and relay
    }
}

void arm_timer1(uint16_t vlo_ticks)
{
    if (timer_armed == 0)
    {
        timer_armed = 1;
        TA1CCTL0 = CCIE;                        // TA1 CCTL0
        TA1CCR0 = vlo_ticks;                        // TA1 CCR0 value
        TA1CTL = TASSEL_1 + MC1 + TACLR + TAIE; //tassel_1 == aclk, tassel_2 ==smclk
    }
}

uint8_t get_rx_buffer()
{
    return (return_byte);
}

void blink_led(){ //sets led high and sets timer to turn itoff after a while
    led(1);
    //led(0);
#if defined(TX_MODE)
    arm_timer1(250);
#elif defined(RX_MODE)
    arm_timer1(50);
#endif

    //_bis_SR_register(LPM3_bits + GIE);
}

#define start_addr 0x1080 //segment B
//erase 64 bytes and write up to 64 bytes into flash mem
void write_to_flash(uint16_t *dbl_byte_array, uint8_t len)
{
    if (len <= 64)
    {  //dont do anything if asked to overwrite too much
        __disable_interrupt(); //ti says disable interupts before writing to flash

        uint8_t i;
        uint16_t *Flash_ptr = (uint16_t *) start_addr; // Initialize Flash segment D pointer

        FCTL1 = FWKEY + ERASE;                    // Set Erase bit
        FCTL3 = FWKEY;                            // Clear Lock bit
        *Flash_ptr = 0;                    // Dummy write to erase Flash segment
        FCTL1 = FWKEY + WRT;                  // Set WRT bit for write operation

        for (i = 0; i < len; i++)
        {
            *Flash_ptr++ = dbl_byte_array[i]; // copy value segment C to segment D
        }

        FCTL1 = FWKEY;                            // Clear WRT bit
        FCTL3 = FWKEY + LOCK;                     // Set LOCK bit

        __enable_interrupt();
    }
}

void read_from_flash(uint16_t *dbl_byte_array, uint8_t len)
{
    if (len <= 64)
    {
        uint8_t i;
        uint16_t *Flash_ptr = (uint16_t *) start_addr; // Initialize Flash segment D pointer

        for (i = 0; i < len; i++)
        {
            dbl_byte_array[i] = *Flash_ptr++; // copy value segment C to segment D
        }

    }
}

void init_flash_controller()
{
    FCTL2 = FWKEY + FSSEL0 + 0x17;         // MCLK/24 for Flash Timing Generator
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
    P1IFG &= ~BIT3;                           // clear the int flag
    button_pressed = 1;  //flag
    __bic_SR_register_on_exit(LPM4_bits);
}

//action timer (led/relay)
#pragma vector=TIMER1_A0_VECTOR //CCR0 vector
__interrupt void timer1_a0(void)
{
    TA1CTL = 0;                           // stop the timer SUSPECT

#if defined(RX_MODE)
    relay(0);
#endif

    led(0);
    timer_armed = 0; //reset
    __bic_SR_register_on_exit(LPM3_bits);
}

#pragma vector=TIMER0_A0_VECTOR //CCR0 vector
__interrupt void timer0_a0(void)
{   debounce_timer = 1; //flag
    __bic_SR_register_on_exit(LPM3_bits);
}

#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCIA0RX_ISR(void)
{
    //onlt rx interupt is enabled so we know this is an rx int
    return_byte = UCA0RXBUF; //HAVE to read the buffer to automatically clear the UCAxRXIFG flag
    __bic_SR_register_on_exit(LPM0_bits);
}
