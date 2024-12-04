/*********************************
HardwareSync for CANDOR
Sean McGinnis - 1/31/2024

This program uses the LP-MSP430FR2476 as a frequency generator for the CANDOR system.


OUTPUT:
        Sync high - P5.1, P5.2
        Pulse out - P5.0

        Unused - 3.5, 3.2 (pin 6 on hand and face)

this update ensures that the same signal comes out of both hand and face pins by #,
so that if they are accidentally switched it shouldn't matter, as long as the
person who wires each knows the pin-out format


Operation:
    Whenever the button is pressed (high signal to P2.4) all of the signals start, outputting signals to
    the above mentioned pins. The button can be pressed again to turn the system off.
    The 120Hz, 60Hz, and 30Hz all rise at the same time after the button is pressed
    and stay in sync with each other. Additionally, both sync high pins are held high while the system is
    active and are low when the system is turned off. The pulse out is held high momentarily when
    the system is turned on and then turns off as soon as the signals are output.
    The onboard LED1 should be on while the system is active.
**********************************/

#include ".\driverlib\MSP430FR2xx_4xx\driverlib.h"


#define CS_MCLK_DESIRED_FREQUENCY_IN_KHZ   12000  // Target frequency for MCLK in kHz
#define CS_MCLK_FLLREF_RATIO               366    // MCLK/FLLRef Ratio

void CS_init(void);

volatile bool toggle = false;   //determines if the system is running
int counter = 0b1111;           //value used to determine state of output pins



int main(void) {

    // Stop watchdog timer
    WDT_A_hold(WDT_A_BASE);

    //set up unused ports
    P4DIR = 0xFF;
    P6DIR = 0xFF;
    PADIR = 0xFF;
    PBDIR = 0xFF;
    PCDIR = 0xFF;
    P4OUT = 0x00;
    P6OUT = 0x00;
    PAOUT = 0x00;
    PBOUT = 0x00;
    PCOUT = 0x00;

    //Set up ports for outputs

    P1DIR |= 0b11001111;       //Hz outputs (all on for now)
    P2DIR |= 0b01100011;
    P3DIR |= 0b00100100;

    P5DIR |= 0b00000111;    //Sync high - P5.1, P5.2     Pulse out - P5.0

    P2DIR &= 0b11101111;    //P2.4 = button in
    P2REN |= 0b00010000;    //P2.4 pull down resistor enable




    //set initial state, all outputs low
    P1OUT = 0b00000000;
    P2OUT = 0b00000000;
    P3OUT = 0b00000000;
    P5OUT = 0b00000000;

    //enable button interrupt
    GPIO_enableInterrupt(GPIO_PORT_P2, GPIO_PIN4);
    GPIO_selectInterruptEdge(GPIO_PORT_P2, GPIO_PIN4, GPIO_LOW_TO_HIGH_TRANSITION);

    // Disable the GPIO power-on default high-impedance mode
    // to activate previously configured port settings
    PMM_unlockLPM5();

    //setup timerA0, used to generate 120Hz
    TA0CCTL0 |= CCIE;         // TACCR0 interrupt enabled

    //MAKE 40000 COUNTS TO GET 74HZ!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    TA0CCR0 = 50000;
    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


    TA0CTL |= TASSEL__SMCLK | MC__UP;     // SMCLK, up mode

    //THIS IS JUST FOR 74 HZ. COMMENT OUT TO RETURN TO 120HZ FRIEND!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    //TA0CTL &= ~0x0080;
    //TA0CTL |=  0x0040; //sets clk divider /2, to slow down for 74 hz
    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    //setup timerA1, used for button debounce
    TA1CCTL0 |= CCIE;   // TACCR0 interrupt enabled
    TA1CCR0 = 0x2000;
    TA1CTL |= TASSEL__ACLK | MC__CONTINUOUS;// ACLK, continuous mode

    CS_init();  //setup SMCLK and ACLK

    __bis_SR_register(LPM0_bits | GIE);           // Enter LPM0 w/ interrupts
    __no_operation();                             // For debug


}

// Timer A0 interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer_A (void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_A0_VECTOR))) Timer_A (void)
#else
#error Compiler not supported!
#endif
{
    //states for output frequencies
    static bool on120 = 0;
    static bool on60 = 0;
    static bool on30 = 0;
    static bool shift30 = 0;


        static bool p10, p16 = 0; //pin 1
        static bool p11, p17 = 0; //pin 2
        static bool p12, p21 = 0; //pin 3
        static bool p13, p25 = 0; //pin 4
        static bool p20, p26 = 0; //pin 5
        static bool p32, p35 = 0; //pin 6

    //intermediate to hold pin states
    static char p1pins = 0b00000000;
    static char p2pins = 0b00000000;
    static char p3pins = 0b00000000;

    if(toggle)  //check if system is on
    {

        //each frequency is one of the first three bits of a down counter,
        //toggling each at half the rate of the bit to the right, 120Hz for the LSB
        on120   = counter & 0x0001;
        on60    = counter & 0x0002;
        on30    = counter & 0x0004;
        shift30 = ~on30 & 0x01;

        //change frequency values here
        p16 = on30; //pin 1
        p10 = on30;

        p17 = on60; //pin 2
        p11 = on60;

        p21 = on30; //pin 3
        p12 = on30;

        p25 = shift30; //pin 4
        p13 = shift30;

        p26 = on60; //on60;
        p20 = on60; //on60;

        p35 = on30; //pin 6
        p32 = on30;


        //determine pin states before setting them to reduce delay
        p1pins = p17 << 7 | p16 << 6 | p13 << 3 | p12 << 2 | p11 << 1 | p10;
        p2pins = p26 << 6 | p25 << 5 | p21 << 1 | p20;
        p3pins = p35 << 5 | p32 << 2;

        //set outputs
        P1OUT = p1pins;
        P2OUT = p2pins;
        P3OUT = p3pins;
        P5OUT = toggle << 2 | toggle << 1;

        counter -= 0b0001;  //down count

        if(counter < 0b0100)//reset count if below 4
        {
            counter = 0b1011;
        }

    }
}


// Timer A1 interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = TIMER1_A0_VECTOR
__interrupt void Timer_A1 (void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER1_A0_VECTOR))) Timer_A1 (void)
#else
#error Compiler not supported!
#endif
{
    //used for button debounce
    TA1CTL &= MC__STOP; //stop timer once it has reached its max value
}


//interrupt for button input P2.4
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=PORT2_VECTOR
__interrupt void PORT2_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(PORT2_VECTOR))) PORT_2_ISR (void)
#else
#error Compiler not supported!
#endif
{
    P2IFG &= ~BIT4;// Clear P2.4 IFG
    toggle ^= true; //turn on system

    //check if button debounce timer has reached max value
    //if not, send to else
    if(toggle && (TA1R >= TA1CCR0)) //turn on system
    {
        counter = 0b1111;   // set counter for all rising edge start
        P5OUT |= 0b00000001;    //set pulse out high

        TA0CTL |= TASSEL__SMCLK | MC__UP;   //turn on frequency generator TA0
        TA1CTL |= TASSEL__ACLK | MC__CONTINUOUS;    //start button debounce
        TA1R = 0;   //reset debounce counts
        TA0R = 0;   //reset frequency generator counts
    }
    else if(TA1R == TA1CCR0)    //turn off system
    {
        //turn off all outputs
        P1OUT = 0b00000000;
        P2OUT = 0b00000000;
        P3OUT = 0b00000000;
        P5OUT = 0b00000000;

        TA0CTL &= MC__STOP; //turn off frequency generator TA0

        TA1CTL |= TASSEL__ACLK | MC__CONTINUOUS;    //start button debounce
        TA1R = 0;   //reset debounce counts
    }
    else    //continue button debounce, untoggle toggle
    {
        TA1CTL |= TASSEL__ACLK | MC__CONTINUOUS;
        toggle ^= true;
    }

}


void CS_init(void)
{
    // Configure one FRAM waitstate as required by the device datasheet for MCLK
    // operation beyond 8MHz _before_ configuring the clock system.
    FRAMCtl_configureWaitStateControl(FRAMCTL_ACCESS_TIME_CYCLES_1);

    //Set DCO FLL reference = REFO
    CS_initClockSignal(
        CS_FLLREF,
        CS_REFOCLK_SELECT,
        CS_CLOCK_DIVIDER_1
        );

    //Set ACLK = REFO
    CS_initClockSignal(
        CS_ACLK,
        CS_REFOCLK_SELECT,
        CS_CLOCK_DIVIDER_16
        );

    CS_initFLLParam param = {0};

//    Set Ratio/Desired MCLK Frequency, initialize DCO, save trim values
    CS_initFLLCalculateTrim(
        CS_MCLK_DESIRED_FREQUENCY_IN_KHZ,
        CS_MCLK_FLLREF_RATIO,
        &param
        );

    //Clear all OSC fault flag
    CS_clearAllOscFlagsWithTimeout(1000);
}
