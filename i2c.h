/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   i2c.h
 * Author: henniek
 *
 * Created on 02 April 2021, 12:36
 */

#ifndef I2C_H
#define I2C_H

#ifdef __cplusplus
extern "C" {
#endif

#define I2C_READ    1
#define I2C_WRITE   0

    uint8_t RX_buffer[16];

    /* *** i2c method declarations *** */
    void init_i2c(void);
    uint8_t i2c_start(void);
    void i2c_stop(void);
    uint8_t i2c_send_address_rw(uint8_t, uint8_t);
    uint8_t i2c_send_byte(uint8_t);
    uint8_t i2c_read_byte(bool);
    uint8_t i2c_send_address_W(uint8_t);
    uint8_t i2c_read_data_from_slave(uint8_t, uint8_t);
    uint8_t i2c_send_command_W(uint8_t, uint8_t);
    uint8_t i2c_send_command_R(uint8_t);
    uint8_t i2c_send_word_W(uint8_t, uint8_t, uint8_t);
    void clearRxBuffer(void);

    /*****************************************************************************
     *                                                                     I2C */


    void init_i2c() {

        //=======================================================================I2C
        //Page 260
        //SCL period = F_CPU/(16 + 2*(TWBitRate) * (PrescalerValue)) 
        //TWBR - Bit rate register
        //TWCR - Control register
        //TWSR - Status register
        //TWDR - Data register
        TWBR = 0x66; //0x20; //bit rate
        TWSR |= /*_BV(TWPS1) | _BV(TWPS0)*/ /**/0x01; // Prescaler 00=1, 01=4, 10=16, 11=64
        //   TWCR |= _BV(TWEN); // Enable TWI
    }

    /* See 'Using TWI' on page 270 of the datasheet */

    uint8_t i2c_start(void) {
        //generate I2C start condition
        //START=SDA transitions from HI to LO while SCL is HI
        //    _
        //SDA  \__
        //    __
        //SCL   \_
        //--clear the control register
        TWCR = 0x00;
        //--set the control register
        //--enable interrupt, status and 
        TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);
        //--Wait for TWINT flag to be set,
        //--indicating that the START condition has been transmitted
        while (!(TWCR & _BV(TWINT)));
        //--check value of status register (with prescaler mask)
        //--return error code if something went wrong
        if ((TWSR & 0xF8) != TW_START) return 1;
        //--return normally
        return 0;
    }

    uint8_t i2c_send_address_rw(uint8_t address, uint8_t read_or_write) {
        //--load the slave address into the data register
        //--the address is shifted one bit to the left
        //--the register value is ORed with a (masked) R/W bit
        TWDR = (address << 1) | (0x01 & read_or_write);
        //--start transmission of address
        TWCR = _BV(TWINT) | _BV(TWEN);
        //--Wait for TWINT flag to be set,
        //--indicating that the address+R/W byte has been transmitted
        //--and ACK/NACK received
        while (!(TWCR & _BV(TWINT)));
        //--check value of status register (with prescaler mask)
        //--return error code if something went wrong
        uint8_t twsr = TWSR & 0xF8;
        if (twsr == TW_MT_SLA_ACK || twsr == TW_MR_SLA_ACK) {
            return 0;
        } else {
            return 1;
        }
    }

    uint8_t i2c_send_byte(uint8_t byte) {
        //--load the byte into the data register
        TWDR = byte;
        //--clear TWINT to start transmission
        TWCR = _BV(TWINT) | _BV(TWEN);
        //--Wait for TWINT flag to be set,
        //--indicating that the data byte has been transmitted
        //--and ACK/NACK received
        while (!(TWCR & _BV(TWINT)));
        //--check value of status register (with prescaler mask)
        //--return error code if something went wrong
        if ((TWSR & 0xF8) != TW_MT_DATA_ACK) return 1;
        return 0;
    }

    uint8_t i2c_read_byte(bool isLastByte) {
        //--clear TWINT to start transmission
        if (isLastByte) {
            TWCR = _BV(TWINT) | _BV(TWEN);
        } else {
            TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWEA);
        }
        //--Wait for TWINT flag to be set,
        //--indicating that the data byte has been transmitted
        //--and ACK/NACK received
        while (!(TWCR & _BV(TWINT)));
        //--check value of status register (with prescaler mask)
        //--return error code if something went wrong
        /*if ((TWSR & 0xF8) != isLastByte ? TW_MR_DATA_NACK : TW_MR_DATA_ACK) return 0xff;*/
        /*if ((TWSR & 0xF8) !=  TW_MR_DATA_ACK) return 0xff;*/
        uint8_t twsr = TWSR & 0xF8;
        if (twsr == TW_MR_DATA_ACK || twsr == TW_MR_DATA_NACK) {
            uint8_t data = TWDR;
            return data;
        } else {
            RX_buffer[0] = 1;
            //----THIS BREAKS RULE #1!
            //----NEVER RETURN AN ERROR VALUE WHICH IS
            //----INDISTINGUISHABLE FROM LEGITIMATE DATA
            //----EVEN IF THE VALUE IS EXTREMELY UNLIKELY
            //----TO LEGITIMATELY OCCUR
            return 0;
        }
        //--the data should now be in the register

    }

    void i2c_stop(void) {
        //generate I2C stop condition
        //STOP=SDA transitions from LO to HI while SCL is HI
        //       _
        //SDA __/
        //      __
        //SCL _/
        //--Transmit STOP condition
        TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWSTO);
    }

    uint8_t i2c_send_address_W(uint8_t addr) {
        //--the error condition is set by default
        //--will be cleared when everything worked
        uint8_t ret = 1;
        //--continue only if start transmitted
        if (i2c_start() != 0) return ret;
        //--if the address is responded to...
        ret = i2c_send_address_rw(addr, I2C_WRITE);
        //--transmit stop condition
        i2c_stop();
        //--ret will be 0 if all went well
        return ret;
    }

    uint8_t i2c_read_data_from_slave(uint8_t addr, uint8_t numBytesToRead) {
        //--
        uint8_t numBytesRead = 0;
        //--the first element of the return array is the error indicator
        //--if it is 0, then there was no error and the array contains
        //--the data received from the slave
        //--if the first element != 0 then there was an error, and the
        //--rest of the array should be considered garbage
        clearRxBuffer();
        //--set error code element to zero
        RX_buffer[0] = 0;
        //--transmit start
        if (i2c_start() != 0) RX_buffer[0] += 1;
        //--send the slave address with R bit 
        if (i2c_send_address_rw(addr, I2C_READ) != 0) RX_buffer[0] += 1;
        //--populate the array with responses from slave
        if (RX_buffer[0] == 0) {
            for (uint8_t idx = 0; idx < numBytesToRead; idx++) {
                RX_buffer[idx + 1] = i2c_read_byte(idx == numBytesToRead - 1 ? true : false);
                numBytesRead++;
            }
        }
        i2c_stop();
        return numBytesRead;
    }

    void clearRxBuffer() {
        RX_buffer[0] = 1;
        for (uint8_t i = 1; i < 16; i++) {
            RX_buffer[i] = 0;
        }
    }

    uint8_t i2c_send_command_W(uint8_t addr, uint8_t cmd) {
        //--the error condition is set by default
        //--will be cleared when everything worked
        uint8_t ret = 1;
        //--continue only if start transmitted
        if (i2c_start() != 0) return ret;
        //--if the address is responded to...
        if (i2c_send_address_rw(addr, I2C_WRITE) == 0) {
            //--...and the command byte is received and acknowledged,
            //-- then the return value is set to 0 
            ret = i2c_send_byte(cmd);
        }
        //--transmit stop condition
        i2c_stop();
        //--ret will be 0 if all went well
        return ret;
    }

    uint8_t i2c_send_word_W(uint8_t addr, uint8_t cmd, uint8_t data) {
        //--the error condition is set by default
        //--will be cleared when everything worked
        uint8_t ret = 1;
        //--continue only if start transmitted
        if (i2c_start() != 0) return ret;
        //--if the address is responded to...
        if (i2c_send_address_rw(addr, I2C_WRITE) == 0) {
            //--...and the command byte is received and acknowledged,
            //-- then the return value is set to 0 
            if (i2c_send_byte(cmd) == 0) {
                if (i2c_send_byte(data) == 0) ret = 0;
            }
        }
        //--transmit stop condition
        i2c_stop();
        //--ret will be 0 if all went well
        return ret;
    }

    uint8_t i2c_send_command_R(uint8_t addr) {
        uint8_t ret = 0;
        if (i2c_start() != 0) return 1;
        if (i2c_send_address_rw(addr, I2C_READ) != 0) ret = 2;
        ret = i2c_read_byte(true);
        i2c_stop();
        return ret;
    }


#ifdef __cplusplus
}
#endif

#endif /* I2C_H */

