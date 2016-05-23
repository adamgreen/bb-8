//ported from arduino library: https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
//written by szymon gaertig (email: szymon@gaertig.com.pl)
//
//Changelog: 
//2013-01-08 - first beta release

#ifndef I2Cdev_h
#define I2Cdev_h

#include "mbed.h"

#define DEFAULT_I2C_SDA p9
#define DEFAULT_I2C_SCL p10

class I2Cdev {
    private:
        I2C i2c;
    public:
        I2Cdev(PinName i2cSda = DEFAULT_I2C_SDA, PinName i2cScl = DEFAULT_I2C_SCL, int frequency = 100000);
        
        int8_t readBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data, uint16_t timeout=I2Cdev::readTimeout());
        int8_t readBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t *data, uint16_t timeout=I2Cdev::readTimeout());
        int8_t readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data, uint16_t timeout=I2Cdev::readTimeout());
        int8_t readBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t *data, uint16_t timeout=I2Cdev::readTimeout());
        int8_t readByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint16_t timeout=I2Cdev::readTimeout());
        int8_t readWord(uint8_t devAddr, uint8_t regAddr, uint16_t *data, uint16_t timeout=I2Cdev::readTimeout());
        int8_t readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data, uint16_t timeout=I2Cdev::readTimeout());
        int8_t readWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t *data, uint16_t timeout=I2Cdev::readTimeout());

        bool writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data);
        bool writeBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t data);
        bool writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
        bool writeBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t data);
        bool writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data);
        bool writeWord(uint8_t devAddr, uint8_t regAddr, uint16_t data);
        bool writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data);
        bool writeWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t *data);

        static uint16_t readTimeout(void);
};

#endif