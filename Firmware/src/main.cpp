/**
 * @file main.cpp
 * @brief TRINA Baxter Embedded Control Demo
 * @author Dan Oates (WPI Class of 2020)
 */
#include <Arduino.h>
#include <AS5048B.h>

/**
 * Hall Encoder
 */
TwoWire* const wire = &Wire;
const uint8_t pin_sda = 18;
const uint8_t pin_scl = 19;
const uint8_t i2c_addr = 0x48;
AS5048B encoder(wire, i2c_addr);

/**
 * Serial Communication
 */
const uint32_t baud = 115200;
uint8_t buffer[4];
Struct str(buffer, Struct::lsb_first);

/**
 * @brief Arduino setup function
 * 
 * Runs once on powerup or reset.
 * Initializes serial and I2C
 */
void setup()
{
	Serial.begin(baud);
	wire->begin();
	wire->setSCL(pin_scl);
	wire->setSDA(pin_sda);
}

/**
 * @brief Arduino loop function
 * 
 * Runs repeatedly after setup
 * Sends joint angle to ROS when prompted
 */
void loop()
{
	if (Serial.available())
	{
		Serial.read();
		str.reset() << encoder.get_angle();
		Serial.write((char*)buffer, 4);
	}
}