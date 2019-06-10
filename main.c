/*
 * irocha_custom_lab.c
 *
 * Created: 6/2/2019 2:59:51 PM
 * Author : irock
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>

/* This is AVR code for driving the RGB LED strips from Pololu.
   It allows complete control over the color of an arbitrary number of LEDs.
   This implementation disables interrupts while it does bit-banging with inline assembly.
 */

/* This line specifies the frequency your AVR is running at.
   This code supports 20 MHz, 16 MHz and 8MHz */
#define F_CPU 8000000

// These lines specify what pin the LED strip is on.
// You will either need to attach the LED strip's data line to PC0 or change these
// lines to specify a different pin.
#define LED_STRIP_PORT PORTD
#define LED_STRIP_DDR  DDRD
#define LED_STRIP_PIN  0

#include <util/delay.h>

/** The rgb_color struct represents the color for an 8-bit RGB LED.
    Examples:
      Black:      (rgb_color){ 0, 0, 0 }
      Pure red:   (rgb_color){ 255, 0, 0 }
      Pure green: (rgb_color){ 0, 255, 0 }
      Pure blue:  (rgb_color){ 0, 0, 255 }
      White:      (rgb_color){ 255, 255, 255} */
typedef struct rgb_color
{
  unsigned char red, green, blue;
} rgb_color;

/** led_strip_write sends a series of colors to the LED strip, updating the LEDs.
 The colors parameter should point to an array of rgb_color structs that hold the colors to send.
 The count parameter is the number of colors to send.
 This function takes about 1.1 ms to update 30 LEDs.
 Interrupts must be disabled during that time, so any interrupt-based library
 can be negatively affected by this function.
 Timing details at 20 MHz (the numbers slightly different at 16 MHz and 8MHz):
  0 pulse  = 400 ns
  1 pulse  = 850 ns
  "period" = 1300 ns
 */
void __attribute__((noinline)) led_strip_write(rgb_color * colors, unsigned int count) 
{
  // Set the pin to be an output driving low.
  LED_STRIP_PORT &= ~(1<<LED_STRIP_PIN);
  LED_STRIP_DDR |= (1<<LED_STRIP_PIN);

  cli();   // Disable interrupts temporarily because we don't want our pulse timing to be messed up.
  while(count--)
  {
    // Send a color to the LED strip.
    // The assembly below also increments the 'colors' pointer,
    // it will be pointing to the next color at the end of this loop.
    asm volatile(
        "ld __tmp_reg__, %a0+\n"
        "ld __tmp_reg__, %a0\n"
        "rcall send_led_strip_byte%=\n"  // Send red component.
        "ld __tmp_reg__, -%a0\n"
        "rcall send_led_strip_byte%=\n"  // Send green component.
        "ld __tmp_reg__, %a0+\n"
        "ld __tmp_reg__, %a0+\n"
        "ld __tmp_reg__, %a0+\n"
        "rcall send_led_strip_byte%=\n"  // Send blue component.
        "rjmp led_strip_asm_end%=\n"     // Jump past the assembly subroutines.

        // send_led_strip_byte subroutine:  Sends a byte to the LED strip.
        "send_led_strip_byte%=:\n"
        "rcall send_led_strip_bit%=\n"  // Send most-significant bit (bit 7).
        "rcall send_led_strip_bit%=\n"
        "rcall send_led_strip_bit%=\n"
        "rcall send_led_strip_bit%=\n"
        "rcall send_led_strip_bit%=\n"
        "rcall send_led_strip_bit%=\n"
        "rcall send_led_strip_bit%=\n"
        "rcall send_led_strip_bit%=\n"  // Send least-significant bit (bit 0).
        "ret\n"

        // send_led_strip_bit subroutine:  Sends single bit to the LED strip by driving the data line
        // high for some time.  The amount of time the line is high depends on whether the bit is 0 or 1,
        // but this function always takes the same time (2 us).
        "send_led_strip_bit%=:\n"
#if F_CPU == 8000000
        "rol __tmp_reg__\n"                      // Rotate left through carry.
#endif
        "sbi %2, %3\n"                           // Drive the line high.

#if F_CPU != 8000000
        "rol __tmp_reg__\n"                      // Rotate left through carry.
#endif

#if F_CPU == 16000000
        "nop\n" "nop\n"
#elif F_CPU == 20000000
        "nop\n" "nop\n" "nop\n" "nop\n"
#elif F_CPU != 8000000
#error "Unsupported F_CPU"
#endif

        "brcs .+2\n" "cbi %2, %3\n"              // If the bit to send is 0, drive the line low now.

#if F_CPU == 8000000
        "nop\n" "nop\n"
#elif F_CPU == 16000000
        "nop\n" "nop\n" "nop\n" "nop\n" "nop\n"
#elif F_CPU == 20000000
        "nop\n" "nop\n" "nop\n" "nop\n" "nop\n"
        "nop\n" "nop\n"
#endif

        "brcc .+2\n" "cbi %2, %3\n"              // If the bit to send is 1, drive the line low now.

        "ret\n"
        "led_strip_asm_end%=: "
        : "=b" (colors)
        : "0" (colors),         // %a0 points to the next color to display
          "I" (_SFR_IO_ADDR(LED_STRIP_PORT)),   // %2 is the port register (e.g. PORTC)
          "I" (LED_STRIP_PIN)     // %3 is the pin number (0-8)
    );

    // Uncomment the line below to temporarily enable interrupts between each color.
    //sei(); asm volatile("nop\n"); cli();
  }
  sei();          // Re-enable interrupts now that we are done.
  _delay_us(80);  // Send the reset signal.
}

#define LED_COUNT 60
rgb_color colors[LED_COUNT];

volatile unsigned char TimerFlag = 0; // TimerISR() sets this to 1. C programmer should clear to 0.

// Internal variables for mapping AVR's ISR to our cleaner TimerISR model.
unsigned long _avr_timer_M = 1; // Start count from here, down to 0. Default 1 ms.
unsigned long _avr_timer_cntcurr = 0; // Current internal count of 1ms ticks

void TimerOn() {
	// AVR timer/counter controller register TCCR1
	TCCR1B = 0x0B;// bit3 = 0: CTC mode (clear timer on compare)
	// bit2bit1bit0=011: pre-scaler /64
	// 00001011: 0x0B
	// SO, 8 MHz clock or 8,000,000 /64 = 125,000 ticks/s
	// Thus, TCNT1 register will count at 125,000 ticks/s

	// AVR output compare register OCR1A.
	OCR1A = 125;	// Timer interrupt will be generated when TCNT1==OCR1A
	// We want a 1 ms tick. 0.001 s * 125,000 ticks/s = 125
	// So when TCNT1 register equals 125,
	// 1 ms has passed. Thus, we compare to 125.
	// AVR timer interrupt mask register
	TIMSK1 = 0x02; // bit1: OCIE1A -- enables compare match interrupt

	//Initialize avr counter
	TCNT1=0;

	_avr_timer_cntcurr = _avr_timer_M;
	// TimerISR will be called every _avr_timer_cntcurr milliseconds

	//Enable global interrupts
	SREG |= 0x80; // 0x80: 1000000
}

void TimerOff() {
	TCCR1B = 0x00; // bit3bit1bit0=000: timer off
}

void TimerISR() {
	TimerFlag = 1;
}

// In our approach, the C programmer does not touch this ISR, but rather TimerISR()
ISR(TIMER1_COMPA_vect) {
	// CPU automatically calls when TCNT1 == OCR1 (every 1 ms per TimerOn settings)
	_avr_timer_cntcurr--; // Count down to 0 rather than up to TOP
	if (_avr_timer_cntcurr == 0) { // results in a more efficient compare
		TimerISR(); // Call the ISR that the user uses
		_avr_timer_cntcurr = _avr_timer_M;
	}
}

// Set TimerISR() to tick every M ms
void TimerSet(unsigned long M) {
	_avr_timer_M = M;
	_avr_timer_cntcurr = _avr_timer_M;
}

void ADC_init() {
	ADCSRA |= (1 << ADEN) | (1 << ADSC) | (1 << ADATE);
	// ADEN: setting this bit enables analog-to-digital conversion.
	// ADSC: setting this bit starts the first conversion.
	// ADATE: setting this bit enables auto-triggering. Since we are
	//        in Free Running Mode, a new conversion will trigger whenever
	//        the previous conversion completes.
}

unsigned short ADC_num = 0;
unsigned char tmpB = 0x00;
unsigned char tmpC = 0xFF;
unsigned char tmpD = 0x00;
unsigned char B0 = 0x00;
unsigned char B1 = 0x00;
unsigned short max = 400;
unsigned char tuneFlag = 0x00;
unsigned char LEDFlag = 0x00;

enum States{start1, hold, add, sub, wait} state1;

void ChangeMax () {
	switch (state1) {
		case start1:
			max = 400;
			tuneFlag = 0x00;
			state1 = hold;
			break;
		case hold:
			if ((B0) && (!B1) && (max < 470)) {
				state1 = add;
			}
			else if ((!B0) && (B1) && (max > 400)) {
				state1 = sub;
			}
			else {
				state1 = hold;
			}
			break;
		case add:
			max = max + 10;
			tuneFlag++;
			state1 = wait;
			break;
		case sub:
			max = max - 10;
			tuneFlag--;
			state1 = wait;
			break;
		case wait:
			if ((!B0) && (!B1)) {
				state1 = hold;
			}
			else {
				state1 = wait;
			}
			break;
		default:
			state1 = hold;
			break;
	}
}

enum tuneStates{start2, off, on} state2;

void TuneSM() {
	switch(state2) {
		case start2:
			state2 = off;
			LEDFlag = 0x00;
			break;
		case off:
			tmpC = 0xFF;
			LEDFlag = 0x00;
			if(ADC_num >= max) {
				state2 = on;
			}
			else if(ADC_num < max) {
				state2 = off;
			}
			break;
		case on:
			LEDFlag = 0x01;
			if(tuneFlag == 0x00) {
				tmpC = 0x7F;
			}
			else if(tuneFlag == 0x01) {
				tmpC = 0xBF;
			}
			else if(tuneFlag == 0x02) {
				tmpC = 0xDF;
			}
			else if(tuneFlag == 0x03) {
				tmpC = 0xEF;
			}
			else if(tuneFlag == 0x04) {
				tmpC = 0xF7;
			}
			else if(tuneFlag == 0x05) {
				tmpC = 0xFB;
			}
			else if(tuneFlag == 0x06) {
				tmpC = 0xFD;
			}
			else if(tuneFlag == 0x07) {
				tmpC = 0xFE;
			}
		
			if(ADC_num >= max) {
				state2 = on;
			}
			else if(ADC_num < max) {
				state2 = off;
			}
			break;
		default:
			state2 = start2;
			break;
	}
}

int main(void)
{
	DDRA = 0x00;
	PORTA = 0xFF;
	DDRB = 0x00;
	PORTB = 0xFF;
	DDRC = 0xFF;
	PORTC = 0xFF;
	DDRD = 0xFF;
	PORTD = 0x00;
	unsigned int time = 0;
	unsigned int i = 0;
	unsigned char x = 0x00;
	unsigned char cnt = 0x00;
	ADC_init();
	TimerSet(100);
	TimerOn();
	while (1)
	{
		B0 = 0x00;
		B1 = 0x00;
		tmpB = 0x00;
		
		tmpB = ~PINB & 0x01;
		if(tmpB == 0x01) {
			B0 = 0x01;
		}
		
		tmpB = ~PINB & 0x02;
		if(tmpB == 0x02) {
			B1 = 0x01;
		}
		
		ChangeMax();
		
		if(TimerFlag) {
			ADC_num = ADC;
			TimerFlag = 0;
		}
		
		TuneSM();
		PORTC = tmpC;
		
		if(LEDFlag == 0x01) {
			cnt = 0;
			while(cnt < 50) {
				while(!TimerFlag){
					B0 = 0x00;
					B1 = 0x00;
					tmpB = 0x00;
					
					tmpB = ~PINB & 0x01;
					if(tmpB == 0x01) {
						B0 = 0x01;
					}
					
					tmpB = ~PINB & 0x02;
					if(tmpB == 0x02) {
						B1 = 0x01;
					}
					
					ChangeMax();
					
					for(i = 0; i < LED_COUNT; i++)
					{
						x = (time >> 2) - 8*i;
						colors[i] = (rgb_color){ x, 255 - x, x };
					}

					led_strip_write(colors, LED_COUNT);

					_delay_ms(20);
					time += 10;
				}
				if(TimerFlag) {
					ADC_num = ADC;
					TimerFlag = 0;
				}
				
				TuneSM();
				PORTC = tmpC;
				cnt++;
			}
			cnt = 0;
		}
		else if(LEDFlag == 0x00) {
			for(i = 0; i < LED_COUNT; i++)
			{
				colors[i] = (rgb_color){ 0, 0, 0 };
			}

			led_strip_write(colors, LED_COUNT);
			
			_delay_ms(20);
		}
	}
}