#ifndef I2C_H_
#define I2C_H_

#include <stdint.h>

#define GPIO_PD0_I2C3SCL 0x00030003
#define GPIO_PD1_I2C3SDA 0x00030403

void I2C_3_config();
void I2C_sendSingleByte(uint8_t slave_addr, char data);
void I2C_sendMultipleBytes(uint8_t slave_addr, uint8_t numOfBytes, char by[]);

#endif 
