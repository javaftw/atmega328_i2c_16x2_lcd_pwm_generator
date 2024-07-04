/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   lcd16x2i2c.h
 * Author: henniek
 *
 * Created on 16 April 2021, 09:49
 */

#ifndef LCD16X2I2C_H
#define LCD16X2I2C_H

//#define SLAVE_ADDRESS 0b00111100 //0x3c
#define SLAVE_ADDRESS 0x3F
//#define SLAVE_ADDRESS 0x27


// commands
#define LCD_CLEARDISPLAY            0x01
#define LCD_RETURNHOME              0x02
#define LCD_ENTRYMODESET            0x04
#define LCD_DISPLAYCONTROL          0x08
#define LCD_CURSORSHIFT             0x10
#define LCD_FUNCTIONSET             0x20
#define LCD_SETCGRAMADDR            0x40
#define LCD_SETDDRAMADDR            0x80

// flags for display entry mode
#define LCD_ENTRYRIGHT              0x00
#define LCD_ENTRYLEFT               0x02
#define LCD_ENTRYSHIFTINCREMENT     0x01
#define LCD_ENTRYSHIFTDECREMENT     0x00

// flags for display on/off control
#define LCD_DISPLAYON               0x04
#define LCD_DISPLAYOFF              0x00
#define LCD_CURSORON                0x02
#define LCD_CURSOROFF               0x00
#define LCD_BLINKON                 0x01
#define LCD_BLINKOFF                ~(LCD_BLINKON)

// flags for display/cursor shift
#define LCD_DISPLAYMOVE             0x08
#define LCD_CURSORMOVE              0x00
#define LCD_MOVERIGHT               0x04
#define LCD_MOVELEFT                0x00

// flags for function set
#define LCD_8BITMODE                0x10
#define LCD_4BITMODE                0x00
#define LCD_2LINE                   0x08
#define LCD_1LINE                   0x00
#define LCD_5x10DOTS                0x04
#define LCD_5x8DOTS                 0x00

// flags for backlight control
#define LCD_BACKLIGHT_ON            0b00001000
#define LCD_BACKLIGHT_OFF           0b00000000
#define En                          0b00000100  // Enable bit
#define Rw                          0b00000010  // Read/Write bit
#define Rs                          0b00000001  // Register select bit


//------------------------------------------bitmaps
//https://maxpromer.github.io/LCD-Character-Creator/

const uint8_t PWM_duty_cycle_map[] = {
    0b01010,
    0b01110,
    0b01010,
    0b00000,
    0b01110,
    0b01010,
    0b11011,
    0b00000
};

const uint8_t PWM_period_map[] = {
    0b10001,
    0b11111,
    0b10001,
    0b00000,
    0b11100,
    0b10101,
    0b00111,
    0b00000
};

const uint8_t symbol_us[] = {
    0b00000,
    0b00000,
    0b00011,
    0b00100,
    0b00010,
    0b11001,
    0b11110,
    0b01000
};

const uint8_t symbol_Hz[] = {
    0b10100,
    0b10100,
    0b11100,
    0b10100,
    0b10111,
    0b00001,
    0b00010,
    0b00011
};

const uint8_t symbol_arrow_right[] = {
    0b00000,
    0b01000,
    0b01100,
    0b01110,
    0b01100,
    0b01000,
    0b00000,
    0b00000
};

const uint8_t symbol_arrow_up[] = {
    0b00000,
    0b00100,
    0b01110,
    0b10101,
    0b00100,
    0b00100,
    0b00100,
    0b00000
};

const uint8_t symbol_arrow_down[] = {
    0b00000,
    0b00100,
    0b00100,
    0b00100,
    0b10101,
    0b01110,
    0b00100,
    0b00000
};


//LCD
void initLCD(void);
void initCustomsCharset(void);
void lcd_send_byte(uint8_t, uint8_t);
void lcd_send_nibble(uint8_t);
void lcd_send_nibble_pulse_enable(uint8_t);
void lcd_write_char(uint8_t);
void lcd_write_string(const char *, uint8_t);
void lcd_send_command(uint8_t);
void lcd_clear(void);
void lcd_home(void);
void lcd_cursor_at(uint8_t, uint8_t);
void lcd_update_multiline(uint8_t *);
void lcd_scroll_top_line(uint8_t *, uint8_t);
void lcd_create_char(uint8_t, const uint8_t *);

//--------------------------------------------------

void initLCD() {
    _delay_ms(500);
    lcd_send_nibble(0x30);
    _delay_ms(5);
    lcd_send_nibble(0x30);
    _delay_ms(5);
    lcd_send_nibble(0x30);
    _delay_ms(15);
    lcd_send_command(0x20);
    _delay_ms(5);
    lcd_send_command(0x2c);
    _delay_ms(2);
    lcd_send_command(0x0c);
    _delay_ms(2);
    lcd_send_command(0x06);
    _delay_ms(2);
    lcd_send_command(0x01);
    _delay_ms(5); /**/

    initCustomsCharset();
}

void initCustomsCharset() {
    lcd_create_char(0, PWM_duty_cycle_map);
    lcd_create_char(1, PWM_period_map);
    lcd_create_char(2, symbol_Hz);
    lcd_create_char(3, symbol_us);
    lcd_create_char(4, symbol_arrow_right);
    lcd_create_char(5, symbol_arrow_up);
    lcd_create_char(6, symbol_arrow_down);
}

/******************************************************************************
 *                                                                LCD   */

void lcd_send_byte(uint8_t data, uint8_t mode) {
    uint8_t HI = 0xF0 & data;
    uint8_t LO = 0xF0 & (data << 4);
    lcd_send_nibble_pulse_enable(HI | mode);
    lcd_send_nibble_pulse_enable(LO | mode);
}

void lcd_send_nibble(uint8_t data) {
    i2c_send_command_W(SLAVE_ADDRESS, data | LCD_BACKLIGHT_ON/**/);
}

void lcd_send_nibble_pulse_enable(uint8_t data) {
    i2c_send_command_W(SLAVE_ADDRESS, data | LCD_BACKLIGHT_ON/**/ | En);
    _delay_ms(2);
    i2c_send_command_W(SLAVE_ADDRESS, data | LCD_BACKLIGHT_ON/**/ | ~En);
    _delay_ms(2);
}

void lcd_write_char(uint8_t data) {
    lcd_send_byte(data, Rs);
}

void lcd_write_string(const char * str, uint8_t len) {
    if (len == 0) {
        for (uint8_t idx = 0; str[idx] != '\0'; idx++) {
            lcd_write_char(str[idx]);
        }
    } else {
        for (uint8_t idx = 0; idx < len && str[idx] != '\0'; idx++) {
            lcd_write_char(str[idx]);
        }
    }
}

void lcd_send_command(uint8_t data) {
    lcd_send_byte(data, 0x00);
}

//-----------------------------LCD higher level

void lcd_clear(void) {
    lcd_send_command(0x01);
    _delay_ms(2);
}

void lcd_home(void) {
    lcd_send_command(0x02);
    _delay_ms(2);
}

void lcd_cursor_at(uint8_t col, uint8_t row) {
    int row_offsets[] = {0x00, 0x40};
    lcd_send_command(LCD_SETDDRAMADDR | (col - 1 + row_offsets[row - 1]));
}

void lcd_update_multiline(uint8_t * str) {
    lcd_clear();
    lcd_home();
    /*if (line_1_len == 0) {
        line_1_len = uart_received_buffer_size;
        copyarr(str, lcd_line_1);
    } else {
        line_2_len = uart_received_buffer_size;
        copyarr(str, lcd_line_2);
    }
    lcd_write_string(lcd_line_1);
    if (line_2_len > 0) {
        lcd_cursor_at(1, 2);
        lcd_write_string(lcd_line_2);
        copyarr(lcd_line_2, lcd_line_1);
        line_1_len = line_2_len;
        line_2_len = 0;
    }*/
}

void lcd_scroll_top_line(uint8_t *str, uint8_t speed) {
    /*uint8_t displaybuf[8];*/

}

void lcd_create_char(uint8_t location, const uint8_t * _map) {
    location &= 0x07;
    lcd_send_command(LCD_SETCGRAMADDR | (location << 3));
    for (uint8_t i = 0; i < 8; i++) {
        lcd_send_byte(_map[i], Rs);
    }
}


#endif /* LCD16X2I2C_H */

