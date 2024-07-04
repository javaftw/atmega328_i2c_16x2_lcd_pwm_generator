/******************************************************************************/
/*                             PROJECT PINOUT                                 */
/******************************************************************************/
/*                             ATMega328P
 *                                ______
 *            RESET/PCINT14/PC6 =|01* 28|= PC5/PCINT13/SCL/ADC5
 *               RX/PCINT16/PD0 =|02  27|= PC4/PCINT12/SDA/ADC4
 *               TX/PCINT17/PD1 =|03  26|= PC3/PCINT11/ADC3
 *             INT0/PCINT18/PD2 =|04  25|= PC2/PCINT10/ADC2
 *                  PCINT19/PD3 =|05  24|= PC1/PCINT9/ADC1
 *                  PCINT20/PD4 =|06  23|= PC0/PCINT8/ADC0
 *                          Vcc =|07  22|= GND
 *                          GND =|08  21|= Aref
 *             XTAL1/PCINT6/PB6 =|09  20|= AVcc
 *             XTAL2/PCINT7/PB7 =|10  19|= PB5/PCINT5/SCK
 *             OC0B/PCINT21/PD5 =|11  18|= PB4/PCINT4/MISO
 *        OC0A/AIN0/PCINT22/PD6 =|12  17|= PB3/PCINT3/MOSI/OC2A/OC2
 *             AIN1/PCINT23/PD7 =|13  16|= PB2/PCINT2/SS/OC1B
 *                   PCINT0/PB0 =|14  15|= PB1/PCINT1/OC1A
 *                                ------
 * 
 *                                ______
 *                              =|01* 28|= SCL --> LCD
 *                              =|02  27|= SDA --> LCD
 *                              =|03  26|= 
 *          Rotary A (PD2/INT0) =|04  25|= 
 *                              =|05  24|= 
 *                              =|06  23|= BTN Mode (PC0/PCINT8)
 *                          Vcc =|07  22|= GND
 *                          GND =|08  21|= Aref
 *                              =|09  20|= 
 *                              =|10  19|= Rotary C (PB5/PCINT5)
 *                              =|11  18|= 
 *                              =|12  17|= 
 *                              =|13  16|= 
 *       Rotary BTN (PCINT0/PB0)=|14  15|= 
 *                                ------
 * 
 * 
 */

/* *** MASTER *** */


/******************************************************************************/
/*                                   DEFS                                     */
/******************************************************************************/

#define F_CPU   8000000
#define MEMSIZE   2048
#define TRUE    1
#define FALSE   0
#define ON      1
#define OFF     0
#define HIGH    1
#define LOW     0
#define NOP     asm("nop")

#define ADC_REF _BV(REFS0)

#define LED                 0x80
#define PIN_ROTARY_L        0x10
#define PIN_ROTARY_R        0x20
#define PIN_ROTARY_BTN      0x01
#define PIN_MODE_BTN        0x01

//---MACROS
#define CMD_EQ(A)       strcmp(A, rxbuf) == 0
#define CMD_EQN(A,N)    strncmp(A, rxbuf, N) == 0
#define ITOA(A)         itoa(A, txbuf, 10)
#define LTOA(A)         ltoa(A, txbuf, 10)
#define FTOA(A)         ftoa(A, txbuf, 10)
#define ITOA2(A)         itoa(A, txbuf, 2)
#define ITOA16(A)         itoa(A, txbuf, 16)

#define _setpin(P,p)          (P |= p)
#define _clearpin(P,p)        (P &= ~(p))
#define _togglepin(P,p)       (P ^= p)
#define _testpin(P,p)         ((P & p) == p)

/******************************************************************************/
/*                                 INCLUDES                                   */
/******************************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/twi.h>
#include <stdlib.h>
#include <string.h>
#include "i2c.h"
#include "lcd16x2i2c.h"

/******************************************************************************/
/*                                VARIABLES                                   */

char rxbuf[8];
char txbuf[8];
volatile uint8_t multiplier;
uint16_t timer_counter;
volatile uint16_t adcval_freq, adcval_dc;
uint32_t pwm_freq;
uint32_t pwm_dc;
float pwm_dc_percent;
//bool buttonRotaryPressed = false;
bool buttonModePressed = false;

/******************************************************************************
                                                           STRUCTS AND ENUMS */
enum Events {
    EVT_NONE,
    EVT_LEFT,
    EVT_RIGHT,
    EVT_ROTARY_BUTTON
};
volatile Events evt;

enum Modes {
    MODE_FREQ,
    MODE_DC
};
volatile Modes mode;


/******************************************************************************/
/*                            FUNCTION DECLARATIONS                           */
/******************************************************************************/

int main(void);
static void init(void);
void mainloop(void);
//----ADC
uint16_t read_adc(uint8_t);
uint16_t compute_average(uint16_t [], uint8_t);
//----PWM
void initPWM(void);
void updateFreq(void);
void updateDC(void);
//----timer
ISR(TIMER0_OVF_vect);
ISR(PCINT0_vect);
ISR(PCINT1_vect);
ISR(INT0_vect);
//----util
void delay_n_us(uint16_t);
void delay_n_ms(uint16_t);

/******************************************************************************
 *                                FUNCTIONS                                   *
 ******************************************************************************/

int main(void) {
    multiplier = 1;
    evt = EVT_NONE;
    mode = MODE_FREQ;
    pwm_dc = 200;
    pwm_freq = 400;
    pwm_dc_percent = pwm_dc / pwm_freq;
    init();
    init_i2c();
    initLCD();
    initPWM();
    mainloop();
    return 0;
}

/*******************************************************************************
 *                                                                        INIT*/

static void init(void) {

    //=======================================================================I/O
    //--PORTB Direction Register (0=input, 1=output)
    DDRB |= 0b00000000;
    //--PORTB Pull-ups (0=deactivated, 1=activated)
    PORTB |= 0b00000100;

    //--PORTC Direction Register (0=input, 1=output)
    DDRC |= 0b00000000;
    //--PORTC Pull-ups (0=deactivated, 1=activated)
    PORTC |= 0b11111111;

    //--PORTD Direction Register (0=input, 1=output)
    DDRD |= 0b11000000;
    //--PORTD Pull-ups (0=deactivated, 1=activated)
    PORTD |= 0b00000000;

    //=======================================================================ADC
    //ADMUX = ADC_REF | 0x00; // | _BV(MUX0);//source is AVCC, select only channel 1
    //enable ADC, select ADC clock to 8000000Hz/64 = 125000Hz (ADPS = 110)
    //ADCSRA |= _BV(ADEN) | _BV(ADPS1) | _BV(ADPS2);


    //====================================================================TIMER0
    TCNT0 = 0x00;
    TCCR0B |= _BV(CS00); // | _BV(CS02);//prescaler
    TIMSK0 |= _BV(TOV0); //timer 0 overflow interrupt enable


    //================================================================INTERRUPTS
    EICRA |= /*_BV(ISC11) | _BV(ISC10) |*/ _BV(ISC01) | _BV(ISC00);
    EIMSK |= /*_BV(INT1)|*/_BV(INT0);
    PCMSK0 |= /*| _BV(PCINT2) |*/ _BV(PCINT0);
    PCMSK1 |= /*| _BV(PCINT2) |*/ _BV(PCINT8);
    PCICR |= /*_BV(PCIE2) |*/ _BV(PCIE1) | _BV(PCIE0);
    sei();
}

/******************************************************************************
 *                                                                    MAINLOOP*/


void mainloop(void) {
    lcd_send_command(0x0c); //0x0C dispaly on cursor off
    _delay_ms(25);
    lcd_home();
    _delay_ms(10);
    lcd_clear();
    lcd_cursor_at(1, 1);
    lcd_send_byte(0, Rs);
    lcd_cursor_at(1, 2);
    lcd_send_byte(1, Rs);
    lcd_cursor_at(2, 2);
    lcd_send_byte(4, Rs);

    lcd_cursor_at(13, 2);
    lcd_write_string("x", 0);
    lcd_write_string(ITOA(multiplier), 0);
    updateDC();
    updateFreq();

    _delay_ms(1200);
    lcd_home();
    while (TRUE) {
        if (buttonModePressed == true) {
            switch (mode) {
                case MODE_DC:
                    lcd_cursor_at(2, 2);
                    lcd_write_string(" ", 0);
                    lcd_cursor_at(2, 1);
                    lcd_send_byte(4, Rs);
                    break;

                case MODE_FREQ:
                    lcd_cursor_at(2, 1);
                    lcd_write_string(" ", 0);
                    lcd_cursor_at(2, 2);
                    lcd_send_byte(4, Rs);
                    break;
            }
            _delay_ms(10);
            buttonModePressed = false;
        }

        switch (evt) {
            case EVT_NONE:
                NOP;
                break;

            case EVT_ROTARY_BUTTON:
                switch (multiplier) {
                    case 1:
                        multiplier = 10;
                        break;
                    case 10:
                        multiplier = 100;
                        break;
                    case 100:
                        multiplier = 250;
                        break;
                    case 250:
                        multiplier = 1;
                        break;

                }
                lcd_cursor_at(14, 2);
                lcd_write_string(ITOA(multiplier), 0);
                lcd_write_string("  ", 0);
                evt = EVT_NONE;
                break;

            case EVT_RIGHT:
                switch (mode) {
                    case MODE_DC:
                        lcd_cursor_at(16, 1);
                        lcd_send_byte(5, Rs);
                        updateDC();
                        break;

                    case MODE_FREQ:
                        lcd_cursor_at(16, 1);
                        lcd_send_byte(5, Rs);
                        updateFreq();
                        break;
                }
                evt = EVT_NONE;
                break;

            case EVT_LEFT:
                switch (mode) {
                    case MODE_DC:
                        lcd_cursor_at(16, 1);
                        lcd_send_byte(6, Rs);
                        updateDC();
                        break;

                    case MODE_FREQ:
                        lcd_cursor_at(16, 1);
                        lcd_send_byte(6, Rs);
                        updateFreq();
                        break;
                }
                evt = EVT_NONE;
                break;

        }

    }
}

void updateFreq() {
    ICR1 = pwm_freq;
    double adjusted_pwm;
    adjusted_pwm = pwm_dc_percent * pwm_freq;
    uint16_t adj_pwm;
    adj_pwm = (uint16_t) adjusted_pwm;
    OCR1A = adj_pwm;
    //-------------------FREQ
    uint32_t freq;
    freq = (8000000 / (2 * 1 * pwm_freq))-(pwm_freq / 100); //F_clk/2*N*TOP where N=prescale 1,8,64,256 or 1024
    lcd_home();
    lcd_cursor_at(3, 2);
    lcd_write_string(LTOA(freq), 0);
    lcd_send_byte(2, Rs);
    lcd_write_string(" ", 0);
    uint32_t us;
    us = 1000000 / freq;
    lcd_cursor_at(11, 1);
    lcd_write_string(ITOA(us), 0);
    lcd_send_byte(3, Rs);
    lcd_write_string(" ", 0);
}

void updateDC() {
    //------------------------DC
    if (pwm_dc == 0) {
        pwm_dc = pwm_freq / 2;
    }
    OCR1A = pwm_dc;
    if ((pwm_dc > 0) & (pwm_freq > 0)) {
        lcd_home();
        lcd_cursor_at(3, 1);
        pwm_dc_percent = (double) pwm_dc / (double) pwm_freq;
        double d = pwm_dc_percent * 100.0;
        lcd_write_string(dtostrf(d, 5, 1, txbuf), 0);
        lcd_write_string("% ", 0);
    }
}

/******************************************************************************
 *                                                                        ADC*/

uint16_t read_adc(uint8_t chan) {
    ADMUX = ADC_REF | chan; //channel 1
    _delay_ms(1);
    ADCSRA |= _BV(ADSC); //enable adc and start conversion
    while (ADCSRA & _BV(ADSC)); //wait for conversion to complete
    return ADCW;
}

uint16_t compute_average(uint16_t data[], uint8_t len) {
    uint16_t ret = 0;
    for (uint8_t i = 0; i < len; i++) {
        ret += data[i];
    }
    return ret / len;
}

/******************************************************************************
 *                                                                        PWM */

//PWM_fequency = clock_speed / [Prescaller_value * (1 + TOP_Value) ]

void initPWM() {
    //--PD6 to output
    DDRB |= (1 << DDB1);

    //===================================================================TIMER1 PWM
    //(In Phase Correct and Phase and Frequency Correct PWM Mode)
    //Clear OC1A/OC1B on Compare Match when up-counting. Set OC1A/OC1B on Compare Match when down-counting.     
    TCCR1A |= _BV(COM1A1);
    //PWM, Phase and Frequency Correct TOP=ICR1 OCR1x_UPDATE=BOTTOM TOV1_Flag=BOTTOM
    TCCR1B |= _BV(WGM13);
    //--Clock select for TCCR1B (001=1, 010=8, 011=64, 100=256, 101=1024)
    TCCR1B |= /*_BV(CS11) | _BV(CS10)|*/ _BV(CS00);
    //
    OCR1A = 500; //duty cycle
    //
    ICR1 = 1000; //frequency

}

/******************************************************************************
 *                                                                 INTERRUPTS*/
ISR(TIMER0_OVF_vect) {
    timer_counter++;
    if (timer_counter == 31250) {
        timer_counter = 0;
        //evt = EVT_READING_ADC;
    }
}

ISR(PCINT0_vect) {

    if (_testpin(PINB, PIN_MODE_BTN) == 1 && _testpin(PINB, PIN_ROTARY_BTN) == 1) {
        evt = EVT_ROTARY_BUTTON;
        /*if (!buttonRotaryPressed) {
            evt = EVT_BUTTON;
            buttonRotaryPressed = true;
        } else {
            evt = EVT_NONE;
            buttonRotaryPressed = false;
        }*/
    }
}

ISR(PCINT1_vect) {
    if (_testpin(PINC, PIN_MODE_BTN) == 0 && !buttonModePressed) {
        buttonModePressed = true;
        switch (mode) {
            case MODE_DC:
                mode = MODE_FREQ;
                break;

            case MODE_FREQ:
                mode = MODE_DC;
                break;
        }
    }
}

ISR(INT0_vect) {
    //----check the state of the SCL pin
    /*
     * 
     * LEFT:
     *      __
     * A  _|  |__
     *       __
     * C  __|  |_
     * 
     * RIGHT:
     *      __
     * A __|  |_
     *     __
     * C _|  |__
     */
    if (evt == EVT_NONE && _testpin(PINB, PIN_ROTARY_R) == 1) {
        evt = EVT_RIGHT;
        switch (mode) {
            case MODE_DC:
                if (pwm_dc + multiplier >= pwm_freq) {
                    pwm_dc = pwm_freq - 1;
                } else {
                    pwm_dc += multiplier;
                }
                break;

            case MODE_FREQ:
                if (pwm_freq + multiplier < 0xFFFE) {
                    pwm_freq += multiplier;
                } else {
                    pwm_freq = 0xFFFE;
                }
                break;
        }
    } else if (evt == EVT_NONE) {// && _testpin(PINB, PIN_ROTARY_R) == 0) {
        evt = EVT_LEFT;
        switch (mode) {
            case MODE_DC:
                if (pwm_dc <= multiplier) {
                    pwm_dc = 1;
                } else {
                    pwm_dc -= multiplier;
                }
                break;

            case MODE_FREQ:
                if (pwm_freq > multiplier) {
                    pwm_freq -= multiplier;
                } else {
                    pwm_freq = 1;
                }
                break;
        }
    } else {
        evt = EVT_NONE;
    }
}

/*****************************************************************************
 *                                                                        UTIL*/

void delay_n_us(uint16_t n) {
    while (n--) {
        _delay_us(1);
    }
}

void delay_n_ms(uint16_t n) {
    while (n--) {
        _delay_ms(1);
    }
}