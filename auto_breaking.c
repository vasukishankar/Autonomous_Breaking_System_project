/* Smart breaking system for electric automobiles
 * MSP432 based project to detect the obstacle and reduce or stop
 * the automobile based on its distance to the obstacle.
 * DC motor control using PWM is also implemented in parallel.
 * Potentiometer is used to vary the speed of the motor using
 * ADC. Piezoelectric sensors are used to generate ultrasonic waves
 * which are reflected from the obstacle.
 */

#include "ti/devices/msp432p4xx/inc/msp.h"
#include <stdint.h>


int distance  = 0;
int miliseconds = 0;
long sensor = 0;
int p;


static void delay(uint32_t count)
{
    volatile uint32_t i;

    for (i = 0 ; i < count ; i++);
}

void clock_setup()
{
        CS->KEY = CS_KEY_VAL;                   // Unlock CS module for register access
        CS->CTL0 = 0;                           // Reset tuning parameters
        CS->CTL0 = CS_CTL0_DCORSEL_3;           // Set DCO to 12MHz (nominal, center of 8-16MHz range)
        CS->CTL1 = CS_CTL1_SELA_2 |             // Select ACLK = REFO
                CS_CTL1_SELS_3 |                // SMCLK = DCO
                CS_CTL1_SELM_3;                 // MCLK = DCO
        CS->KEY = 0;                            // Lock CS module from unintended accesses
}

void port_pin_setup()
{
    // LED SETUP
     P2->OUT &= ~BIT0;                       // Setup P2.0 led --> RED
     P2->DIR |= BIT0;                        // Setup P2.0 led --> RED

     // PIN for ADC setup
     P5->SEL1 |= BIT4;                       // Configure P5.4 for ADC
     P5->SEL0 |= BIT4;

     // PWM output
     P6->SEL0 |= 0x40;
     P6->DIR |= 0x40;

     // Configure GPIO
     P1->DIR |= BIT0;                        // Set P1.0 as output
     P1->OUT |= BIT0;                        // P1.0 high
     P1->OUT &= ~BIT0;

     // Configure UART pins
     P1->SEL0 |= BIT2 | BIT3;                // set 2-UART pin as secondary function

     P2->DIR &= ~BIT7;                       // P2.7 for echo pin
     P2->REN |= BIT7;                        // P2.7 pull down enabled
     P2->OUT &= ~BIT7;                       // P2.7 initial output Low

     P2->SEL0 = 0;                           // select port 2 for adc
     P2->SEL1 = 0;

     P2->IFG = 0;
     P2->IE |= BIT7;                         // enable interrupts on port.
     P2->IES &= ~BIT7;
}

void timerA2_for_pwm()
{
    TIMER_A2->CTL = 0x0212;
    TIMER_A2->CCTL[3] = 0x00f1;
    TIMER_A2->CCR[0] = 0x1000;
}

void ADC_config()
{
    ADC14->CTL0 = ADC14_CTL0_SHT0_2 | ADC14_CTL0_SHP | ADC14_CTL0_ON;
    ADC14->CTL1 = ADC14_CTL1_RES_2;                 // Use sampling timer, 12-bit conversion results

    ADC14->MCTL[0] |= ADC14_MCTLN_INCH_1;           // A1 ADC input select; Vref=AVCC
    ADC14->IER0 |= ADC14_IER0_IE0;
}

void uart_setup()
{

    EUSCI_A0->CTLW0 |= EUSCI_A_CTLW0_SWRST;      // Put eUSCI in reset
    EUSCI_A0->CTLW0 = EUSCI_A_CTLW0_SWRST |      // Remain eUSCI in reset
                EUSCI_B_CTLW0_SSEL__SMCLK;       // Configure eUSCI clock source for SMCLK
    // Baud Rate calculation
    // 12000000/(16*9600) = 78.125
    // Fractional portion = 0.125
    // User's Guide Table 21-4: UCBRSx = 0x10
    // UCBRFx = int ( (78.125-78)*16) = 2
    EUSCI_A0->BRW = 78;                            // 12000000/16/9600
    EUSCI_A0->MCTLW = (2 << EUSCI_A_MCTLW_BRF_OFS) |
                EUSCI_A_MCTLW_OS16;

    EUSCI_A0->CTLW0 &= ~EUSCI_A_CTLW0_SWRST;      // Initialize eUSCI
    EUSCI_A0->IFG &= ~EUSCI_A_IFG_RXIFG;          // Clear eUSCI RX interrupt flag
    EUSCI_A0->IE &= ~EUSCI_A_IE_RXIE;             // Disable USCI_A0 RX interrupt
}


int main(void)
{

    WDT_A->CTL = WDT_A_CTL_PW |             // Stop watchdog timer
            WDT_A_CTL_HOLD;

    clock_setup();                          //setup uart registers.

    port_pin_setup();                       //setup port pins for ADC, PWM and LED.

    timerA2_for_pwm();

    ADC_config();

    uart_setup();

                                                    // Timer A0 configuration for object detection.

    TIMER_A0->CCTL[0]= TIMER_A_CCTLN_CCIE;          // CCR0 interrupt enabled
    TIMER_A0->CCR[0] = 1000-1;                      // 1ms at 1mhz
    TIMER_A0->CTL = TIMER_A_CTL_TASSEL_2 | TIMER_A_CTL_MC__UP | TIMER_A_CTL_CLR;                  // SMCLK, upmode


    __enable_irq();                                 // Enables interrupts to the system

    NVIC->ISER[1] = 1 << ((PORT2_IRQn) & 31);       // assign NVIC for port interrupt
    NVIC->ISER[0] = 1 << ((ADC14_IRQn) & 31);       // assign NVIC for ADC interrupt
    NVIC->ISER[0] = 1 << ((TA0_0_IRQn) & 31);       // assign NVIC for Timer interrupt

    while(1){

        P2->DIR |= BIT6;          // trigger pin as output

        P2->OUT |= BIT6;          // generate pulse
        delay(100);               // Delay of 10us
        P2->OUT &= ~BIT6;         // stop pulse

        P2->IFG = 0;              // clear P2 interrupt
        P2->IES &= ~BIT7;         // rising edge on ECHO pin

        delay(300000);            // delay for 30ms (after this time echo times out if there is no object detected)

        ADC14->CTL0 |= ADC14_CTL0_ENC | ADC14_CTL0_SC;   //Start ADC operation.

        distance = sensor/58;     // converting ECHO length into cm


        if (distance < 1000 && distance > 750)
        {                         // if obstacle is within 100cm
                TIMER_A2->CCR[3] = TIMER_A2->CCR[3] * 3/4;
                printf("Distance to obstacle is %d cm \n",distance/10);
                delay(1000000);
        }

        else if(distance <= 750 && distance > 500){
                                 //if obstacle is within 75cm to 50cm.
                TIMER_A2->CCR[3] = TIMER_A2->CCR[3] * 1/4;
                printf("Distance to obstacle is %d cm \n",distance/10);
                delay(1000000);
        }

        else if(distance <= 500 && distance > 250)
        {                        //if obstacle is within 50cm to 25cm.
                TIMER_A2->CCR[3] = TIMER_A2->CCR[3] * 1/8;
                printf("Distance to obstacle is %d cm \n",distance/10);
                delay(1000000);
        }

                                 // if distance is within 25cm - stop the car.
        else if(distance <= 250 && distance != 0)
        {
            P1->OUT |= BIT0;     //turning LED on if distance is less than 25cm.
            ADC14->CTL0 &= ~(ADC14_CTL0_ENC | ADC14_CTL0_SC);  //Stop ADC.
            TIMER_A2->CCR[3] = 0;          // Stop motor. Set Duty cycle to 0.
            printf("BREAK!! %d cm is too close\n",distance/10);
            delay(10000000);               // Halt vehicle for a while.
        }
        else
        {
            P1->OUT &= ~BIT0;
            printf("BREAK!! %d cm is too close\n",distance/10);
        }
    }
}


// Timer A0 interrupt service routine
void PORT2_IRQHandler(void)
{

    if(P2->IFG & BIT7)                          //If interrupt is pending
    {
        if(!(P2->IES & BIT7))                   // On rising edge
        {

            TIMER_A0->CTL |= TIMER_A_CTL_CLR;   // clears timer A
            miliseconds = 0;
            P2->IES |=  BIT7;
        }                                       //falling edge
        else
        {
            sensor = (long)miliseconds*1000 + (long)TIMER_A0->R;    //calculating ECHO length
                                                                    //  P1->OUT ^= BIT0;
            P2->IES &=  ~BIT7;                                      //falling edge

        }
        P2->IFG &= ~BIT7;                                           //clear flag
    }

}

void TA0_0_IRQHandler(void)
{
    //Interrupt gets triggered for every clock cycle in SMCLK Mode counting number of pulses
    miliseconds++;          //global counter
    TIMER_A0->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG;

}

void ADC14_IRQHandler(void) {

      TIMER_A2->CCR[3] = ADC14->MEM[0];         //set duty cycle value here.
      P2->OUT |= BIT0;
}
