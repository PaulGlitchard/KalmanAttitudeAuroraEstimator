#include <msp430.h> 


/**
 * main.c
 */
int main(void)
{
 WDTCTL = WDTPW | WDTHOLD; // stop watchdog timer
 PM5CTL0 &= ~LOCKLPM5;       // Disable the GPIO power-on default high-impedance mode

     P2DIR |= BIT6;              // Set P2.6 as output
     P2OUT &= ~BIT6;             // Initially turn off the LED

     while(1)
     {
         P2OUT ^= BIT6;          // Toggle the LED
         __delay_cycles(1000000);  // Delay for half a second (adjust as needed)
     }
 return 0;
}
