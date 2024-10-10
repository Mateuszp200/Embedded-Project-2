#define F_CPU 20000000
#define USART3_BAUD_RATE(BAUD_RATE)((float)(F_CPU * 64 / (16 * (float) BAUD_RATE)) + 0.5)
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/cpufunc.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdbool.h>
 
void CLOCK_init(void);
void EVSYS_Init(void);
void TCA0_SS_init(void);
void Initialise_TCB0_ICP_PW(void);
void Initialise_TCB3_ICP_PW(void);
void ADC_Init(void);
static void USART3_init(void);
void sendmsg(char * s);

 

/* Serial comms related globals & TCBO And ADC Flag */
unsigned char qcntr = 0, sndcntr = 0; /*indexes into the queue*/
unsigned char queue[50]; /*character queue*/
volatile int ADC_FLAG = 0; // flag for new ADC VALUE
volatile uint16_t adc_reading, adc_reading_mv;
volatile int TCB0_Flag = 0; //TCBO capture flag
uint16_t clocksP, clocksT, Pulse_Width, Time_Period;
#define THREE_5V 716 // voltage for ADC to display LED BIT 6


void CLOCK_init(void) {
  /* Do not use low frequency clock, disable CLK_PER Prescaler */
  ccp_write_io((void * ) & CLKCTRL.MCLKCTRLB, (0 << CLKCTRL_PEN_bp));
  /* If set from the fuses during programming, the CPU will now run at 20MHz (default is /6) */
}

 
/* used for to intialise needed ports     */
void InitialiseLED_PORT_bits() {
  PORTC.DIR = PIN6_bm;
  PORTA.DIR = PIN1_bm | PIN0_bm;
  PORTF.DIR = PIN5_bm | PIN4_bm;
}

void Initialise_TCB1_ICP_PW(void){
              TCB1.CTRLA = TCB_CLKSEL_CLKDIV2_gc | TCB_ENABLE_bm;
              TCB1.CTRLB = 0b01000001;
              TCB1.EVCTRL = ~TCB_EDGE_bm | TCB_CAPTEI_bm;
}

void EVSYS_Init(void) {
  EVSYS.CHANNEL4 = EVSYS_GENERATOR_PORT0_PIN3_gc; // EVSYS Channel 4 generator is PORTE Bit 3
  EVSYS.USERTCB0 = EVSYS_CHANNEL_CHANNEL4_gc; /* TCB0 is the Channel 4 User */
  EVSYS.CHANNEL0 = EVSYS_GENERATOR_TCB3_CAPT_gc; //Channel 0 generator is TCB3 CAP
  EVSYS.USERADC0 = EVSYS_CHANNEL_CHANNEL0_gc;
}

void TCA0_SS_init(void) {
	TCA0.SINGLE.CTRLA = (TCA_SINGLE_CLKSEL_DIV16_gc | TCA_SINGLE_ENABLE_bm);
	TCA0.SINGLE.CTRLB = (TCA_SINGLE_CMP1EN_bm | TCA_SINGLE_WGMODE_SINGLESLOPE_gc);
	TCA0.SINGLE.PER = 24999; // Set period for a 50 Hz PWM
	TCA0.SINGLE.CMP1 = 1250; // Initial duty cycle, static setup
	//1250 to 2500
}

void Initialise_TCB0_ICP_PW() {
  /* Enable TCB0 and set CLK_PER divider to 2: Timer clock = 10MHz now */
  TCB0.CTRLA = TCB_CLKSEL_CLKDIV2_gc | TCB_ENABLE_bm; /* May do enable later */
/* Configure TCB0 Frequency Pulse-Width Measurement mode */
  TCB0.CTRLB = TCB_CNTMODE_FRQPW_gc;/* Enable Capture or Timeout interrupt */
  TCB0.INTCTRL = TCB_CAPT_bm;/* Enable Event Input and Event Edge*/
  TCB0.EVCTRL = ~TCB_EDGE_bm | TCB_CAPTEI_bm; /* falling Edge selected because 555 Timer is asymmetric, low pulse shorter */
}

void Initialise_TCB3_ICP_PW(void) {
  TCB3.CTRLA = TCB_CLKSEL_CLKDIV2_gc | TCB_ENABLE_bm; //CLK_PER/2 and Enable TCB0
  TCB3.CTRLB = TCB_CNTMODE_INT_gc; //Periodic Measurement mode 
  TCB3.INTCTRL = !(TCB_CAPT_bm);
  TCB3_CCMP = 5 * (10 ^ (-3)); // 5 mili-seconds
}

void ADC_Init(void) {
  ADC0.CTRLA = 0b00000001; /* 10-bit resolution selected, Free Running Mode off, ADC0 enabled */
  ADC0.CTRLB = 0b00000000; /* Simple No Accumulation operation selected, this line could be omitted */
  ADC0.CTRLC = 0b00010101; /* SAMPCAP=0; REFSEL: VDD; PRESC set to DIV64 */
  ADC0.CTRLD = 0b00000000; /* INITDLY 0*/
  ADC0.MUXPOS = 0b00000001; /* Select AIN1  */
  ADC0.INTCTRL = 0b00000001; /* Enable an interrupt when conversion complete (RESRDY) */
  ADC0.EVCTRL = 0b00000001;
}

static void USART3_init(void) {
  PORTB.DIR &= ~PIN5_bm; /* this is the RX input */
  PORTB.DIR |= PIN4_bm; /* this is the TX output */
  USART3.BAUD = (uint16_t) USART3_BAUD_RATE(115200);
  USART3.CTRLB |= (USART_TXEN_bm | USART_RXEN_bm);
  PORTMUX.USARTROUTEA |= PORTMUX_USART3_ALT1_gc;
  USART3.CTRLA = USART_TXCIE_bm;
}

int main(void) {
  char ch;
  char str_buffer[60];
  static int CONT_TIMER_MODE = 0; // continuous timer mode
  static int CONT_ADC_MODE = 0; // continuous ADC
 
  CLOCK_init();
  EVSYS_Init();
  TCA0_SS_init();
  Initialise_TCB0_ICP_PW();
  Initialise_TCB1_ICP_PW();
  Initialise_TCB3_ICP_PW();
  ADC_Init();
  USART3_init();
  InitialiseLED_PORT_bits();

  sei(); /* Enable Global Interrupts */

  while (1) {
    if (USART3.STATUS & USART_RXCIF_bm) {
      /* If a character has been received, read it - this structure allows other code to run */
      ch = USART3.RXDATAL;
      switch (ch) {
      case '0':
      sprintf(str_buffer, "Angle = 0 degrees\n");
      sendmsg(str_buffer);
      TCA0.SINGLE.CMP1 = 1250;
      break;
      case '1':
	  {
		  unsigned int position;
		  sprintf(str_buffer, "0.5s per step of 16.67 degrees\n");
		  sendmsg(str_buffer);

		  // Moving forward from 1250 to 2500
		  for (position = 1250; position <= 2500; position += 125) {
			  TCA0.SINGLE.CMP1 = position; // Set the servo to the next position.
			  _delay_ms(1000); // Wait for 1 second before next increment
		  }

		  // Moving backward from 2500 to 1250
		  for (position = 2500; position >= 1250; position -= 125) {
			  TCA0.SINGLE.CMP1 = position; // Set the servo to the previous position.
			  _delay_ms(1000); // Wait for 1 second before next decrement
		  }
	  }
      break;
      case '2':
      {
	      unsigned int position;
	      sprintf(str_buffer, "0.5s per step of 16.67 degrees\n");
	      sendmsg(str_buffer);

	      // Moving forward from 1250 to 2500
	      for (position = 1250; position <= 2500; position += 125) {
		      TCA0.SINGLE.CMP1 = position; // Set the servo to the next position.
		      _delay_ms(500); // Wait for 0.5 second before next increment
	      }

	      // Moving backward from 2500 to 1250
	      for (position = 2500; position >= 1250; position -= 125) {
		      TCA0.SINGLE.CMP1 = position; // Set the servo to the previous position.
		      _delay_ms(500); // Wait for 0.5 second before next decrement
	      }
      }
      break;
      case '3':
      {
	      unsigned int position;
	      sprintf(str_buffer, "0.4s per step of 16.67 degrees\n");
	      sendmsg(str_buffer);

	      // Moving forward from 1250 to 2500
	      for (position = 1250; position <= 2500; position += 125) {
		      TCA0.SINGLE.CMP1 = position; // Set the servo to the next position.
		      _delay_ms(400); // Wait for 0.4 second before next increment
	      }

	      // Moving backward from 2500 to 1250
	      for (position = 2500; position >= 1250; position -= 125) {
		      TCA0.SINGLE.CMP1 = position; // Set the servo to the previous position.
		      _delay_ms(400); // Wait for 0.4 second before next decrement
	      }
      }
      break;
      case '4':
      {
	      unsigned int position;
	      sprintf(str_buffer, "0.3s per step of 16.67 degrees\n");
	      sendmsg(str_buffer);

	      // Moving forward from 1250 to 2500
	      for (position = 1250; position <= 2500; position += 125) {
		      TCA0.SINGLE.CMP1 = position; // Set the servo to the next position.
		      _delay_ms(300); // Wait for 0.3 second before next increment
	      }

	      // Moving backward from 2500 to 1250
	      for (position = 2500; position >= 1250; position -= 125) {
		      TCA0.SINGLE.CMP1 = position; // Set the servo to the previous position.
		      _delay_ms(300); // Wait for 0.3 second before next decrement
	      }
      }
      break;
      case '5':
      {
	      unsigned int position;
	      sprintf(str_buffer, "0.25s per step of 16.67 degrees\n");
	      sendmsg(str_buffer);

	      // Moving forward from 1250 to 2500
	      for (position = 1250; position <= 2500; position += 125) {
		      TCA0.SINGLE.CMP1 = position; // Set the servo to the next position.
		      _delay_ms(250); // Wait for 0.25 second before next increment
	      }

	      // Moving backward from 2500 to 1250
	      for (position = 2500; position >= 1250; position -= 125) {
		      TCA0.SINGLE.CMP1 = position; // Set the servo to the previous position.
		      _delay_ms(250); // Wait for 0.25 second before next decrement
	      }
      }
      break;
      case '6':
      {
	      unsigned int position;
	      sprintf(str_buffer, "0.2s per step of 16.67 degrees\n");
	      sendmsg(str_buffer);

	      // Moving forward from 1250 to 2500
	      for (position = 1250; position <= 2500; position += 125) {
		      TCA0.SINGLE.CMP1 = position; // Set the servo to the next position.
		      _delay_ms(200); // Wait for 0.2 second before next increment
	      }

	      // Moving backward from 2500 to 1250
	      for (position = 2500; position >= 1250; position -= 125) {
		      TCA0.SINGLE.CMP1 = position; // Set the servo to the previous position.
		      _delay_ms(200); // Wait for 0.2 second before next decrement
	      }
      }
      break;
      case '7':
      {
	      unsigned int position;
	      sprintf(str_buffer, "0.15s per step of 16.67 degrees\n");
	      sendmsg(str_buffer);

	      // Moving forward from 1250 to 2500
	      for (position = 1250; position <= 2500; position += 125) {
		      TCA0.SINGLE.CMP1 = position; // Set the servo to the next position.
		      _delay_ms(150); // Wait for 0.15 second before next increment
	      }

	      // Moving backward from 2500 to 1250
	      for (position = 2500; position >= 1250; position -= 125) {
		      TCA0.SINGLE.CMP1 = position; // Set the servo to the previous position.
		      _delay_ms(150); // Wait for 0.15 second before next decrement
	      }
      }
      break;
      case '8':
      {
	      unsigned int position;
	      sprintf(str_buffer, "0.1s per step of 16.67 degrees\n");
	      sendmsg(str_buffer);

	      // Moving forward from 1250 to 2500
	      for (position = 1250; position <= 2500; position += 125) {
		      TCA0.SINGLE.CMP1 = position; // Set the servo to the next position.
		      _delay_ms(100); // Wait for 0.1 second before next increment
	      }

	      // Moving backward from 2500 to 1250
	      for (position = 2500; position >= 1250; position -= 125) {
		      TCA0.SINGLE.CMP1 = position; // Set the servo to the previous position.
		      _delay_ms(100); // Wait for 0.1 second before next decrement
	      }
      }
      break;
      case '9':
      {
	      unsigned int position;
	      sprintf(str_buffer, "0.05s per step of 16.67 degrees\n");
	      sendmsg(str_buffer);

	      // Moving forward from 1250 to 2500
	      for (position = 1250; position <= 2500; position += 125) {
		      TCA0.SINGLE.CMP1 = position; // Set the servo to the next position.
		      _delay_ms(50); // Wait for 0.05 second before next increment
	      }

	      // Moving backward from 2500 to 1250
	      for (position = 2500; position >= 1250; position -= 125) {
		      TCA0.SINGLE.CMP1 = position; // Set the servo to the previous position.
		      _delay_ms(50); // Wait for 0.05 second before next decrement
	      }
      }
      break;
      case 't':
      case 'T':
      sprintf(str_buffer, "Period = %dus\n", Time_Period);
      sendmsg(str_buffer);
      break;
      case 'l':
      case 'L':
      sprintf(str_buffer, "TimeLow = %dus\n", Time_Period - Pulse_Width);
      sendmsg(str_buffer);
      break;
      case 'h':
      case 'H':
      sprintf(str_buffer, "TimeHigh = %dus\n", Pulse_Width);
      sendmsg(str_buffer);
      break;
      case 'c':
      case 'C':
      while (1) {
	      sprintf(str_buffer, "Timer input = %dus\n", Time_Period);
	      sendmsg(str_buffer);
	      if (USART3.STATUS & USART_RXCIF_bm) {
		      ch = USART3.RXDATAL;
		      if (ch == 'e' || ch == 'E') break;
	      }
	      _delay_ms(500);
      }
      break;
      case 'a':
      case 'A':
      sprintf(str_buffer, "ADC = %d\n", adc_reading);
      sendmsg(str_buffer);
      break;
      case 'v':
      case 'V':
      sprintf(str_buffer, "Voltage = %dmV\n", (adc_reading_mv));
      sendmsg(str_buffer);
      break;
      case 'm':
      case 'M':
      while (1) {
	      sprintf(str_buffer, "Voltage = %dmV\n", adc_reading_mv);
	      sendmsg(str_buffer);
	      if (USART3.STATUS & USART_RXCIF_bm) {
		      ch = USART3.RXDATAL;
		      if (ch == 'n' || ch == 'N') break;
	      }
	      _delay_ms(500);
      }
      break;
      }
    }
    /* Even if a character has not been received, code inserted here can still run */
    if (CONT_TIMER_MODE) {
      if (TCB0_Flag) {
        sprintf(str_buffer, "Time Period = %d\n", Time_Period);
        sendmsg(str_buffer);
        TCB0_Flag = 0;
      }
    } else {
      if (CONT_ADC_MODE) {
        if (ADC_FLAG) {
             sprintf(str_buffer, "Voltage = %dmV\n", (adc_reading_mv));
             sendmsg(str_buffer);
          ADC_FLAG = 0;
        }
      }
    }
  }
}

/*this function loads the queue and */
/*starts the sending process*/
void sendmsg(char * s) {
  qcntr = 0; /*preset indices*/
  sndcntr = 1; /*set to one because first character already sent*/
  while ( * s)
    queue[qcntr++] = * s++; /*put characters into queue*/
  USART3.TXDATAL = queue[0]; /*send first character to start process*/
}

ISR(ADC0_RESRDY_vect) {
  adc_reading = ADC0.RES; // adc reading
  adc_reading_mv = ((ADC0.RES * 5) / 1.023); // reading mv for main()
  ADC_FLAG = 1;
 
  if (adc_reading > THREE_5V) {
    PORTF.OUTSET = PIN4_bm; // turn on LED bit 6
  } else {
    PORTF.OUTCLR = PIN4_bm; // turn off LED bit 6
  }
  TCB3.INTFLAGS = 0b00000001;
}

ISR(TCB0_INT_vect) {
  static uint32_t clocks, Pulse_Width_1;
  TCB0_Flag = 1;
  TCB0.INTFLAGS = TCB_CAPT_bm; // Clear the interrupt flag /
  clocksT = TCB0.CNT; // In PW - Freq mode. CNT stops on the trailing edge until CCMP is read. Read order is important /
  clocksP = TCB0.CCMP;
  clocks = (uint32_t) TCB0.CCMP;

  Pulse_Width = clocksP / 10; // Convert to microseconds /
  Time_Period = clocksT / 10; // To microseconds */
  Pulse_Width_1 = clocks / 10; // used for comaparison

  if (Pulse_Width_1 > 150) {
    PORTA.OUTSET = PIN0_bm; // turn on LED 2
  } else {
    PORTA.OUTCLR = PIN0_bm; // turn off LED 2
  }
  if (Pulse_Width_1 > 320) {
    PORTF.OUTSET = PIN5_bm;  //turn on LED 3
  } else {
    PORTF.OUTCLR = PIN5_bm;  // turn off LED 3
  }
  PORTF.OUTCLR = PIN6_bm; // turn off LED 4 

}

ISR(USART3_TXC_vect) {
  /*send next character and increment index*/
  USART3.STATUS |= USART_TXCIF_bm;
  if (qcntr != sndcntr)
    USART3.TXDATAL = queue[sndcntr++];
  /* Stop sending when the queue is empty. TXC interrupts only happen when a character
     has been transmitted. Stopping sending stops the interrupts */
}

//ISR TCB3?
