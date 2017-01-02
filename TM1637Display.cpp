
//  Author: avishorp@gmail.com
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

extern "C" {
  #include <stdlib.h>
  #include <string.h>
  #include <inttypes.h>
}

#include "TM1637Display.h"


#ifdef __ARM_ARCH_6__
// Raspberry Pi has ARMv6
#include <wiringPi.h>

#elif defined(__XTENSA__) 
// ESP8266 RTOS SDK
#include "esp_common.h"
#include "esp_misc.h"
#include "gpio.h"
#include "freertos/task.h"

// Arduino-like wrapper functions
#define HIGH 0x1
#define LOW  0x0

#define INPUT 0x0
#define OUTPUT 0x1

static void pinMode(uint8_t pin, uint8_t mode)
{
    if (pin < 16) // only for GPIO0 to GPIO15, GPIO16 is special
    {
        if (mode == OUTPUT)
        {
            GPIO_AS_OUTPUT(1 << pin);
        }
        else // INPUT
        {
            GPIO_AS_INPUT(1 << pin);
        }
    }
}

static void digitalWrite(uint8_t pin, uint8_t val)
{
    GPIO_OUTPUT_SET(pin, val ? 1 : 0);
}

static int digitalRead(uint8_t pin)
{
    return GPIO_INPUT_GET(pin);
}

static void delayMicroseconds(unsigned int us)
{
    if (us < 1000)
    {
        os_delay_us(us);
    }
    else
    {
        vTaskDelay((us / 1000)/portTICK_RATE_MS);
    }

}

#else
#include <Arduino.h>
#endif

#define TM1637_I2C_COMM1    0x40
#define TM1637_I2C_COMM2    0xC0
#define TM1637_I2C_COMM3    0x80

//
//      A
//     ---
//  F |   | B
//     -G-
//  E |   | C
//     ---
//      D
const uint8_t digitToSegment[] = {
 // XGFEDCBA
  0b00111111,    // 0
  0b00000110,    // 1
  0b01011011,    // 2
  0b01001111,    // 3
  0b01100110,    // 4
  0b01101101,    // 5
  0b01111101,    // 6
  0b00000111,    // 7
  0b01111111,    // 8
  0b01101111,    // 9
  0b01110111,    // A
  0b01111100,    // b
  0b00111001,    // C
  0b01011110,    // d
  0b01111001,    // E
  0b01110001     // F
  };


TM1637Display::TM1637Display(uint8_t pinClk, uint8_t pinDIO)
{
	// Copy the pin numbers
	m_pinClk = pinClk;
	m_pinDIO = pinDIO;
	
	// Set the pin direction and default value.
	// Both pins are set as inputs, allowing the pull-up resistors to pull them up
    pinMode(m_pinClk, INPUT);
    pinMode(m_pinDIO,INPUT);
	digitalWrite(m_pinClk, LOW);
	digitalWrite(m_pinDIO, LOW);
}

void TM1637Display::setBrightness(uint8_t brightness)
{
	m_brightness = brightness;
}

void TM1637Display::setSegments(const uint8_t segments[], uint8_t length, uint8_t pos)
{
    // Write COMM1
	start();
	writeByte(TM1637_I2C_COMM1);
	stop();
	
	// Write COMM2 + first digit address
	start();
	writeByte(TM1637_I2C_COMM2 + (pos & 0x03));
	
	// Write the data bytes
	for (uint8_t k=0; k < length; k++) 
	  writeByte(segments[k]);
	  
	stop();

	// Write COMM3 + brightness
	start();
	writeByte(TM1637_I2C_COMM3 + (m_brightness & 0x0f));
	stop();
}
 
void TM1637Display::showNumberDec(int num, bool leading_zero, uint8_t length, uint8_t pos)
{
	uint8_t digits[4];
	const static int divisors[] = { 1, 10, 100, 1000 };
	bool leading = true;
	
	for(int8_t k = 0; k < 4; k++) {
	    int divisor = divisors[4 - 1 - k];
		int d = num / divisor;
		
		if (d == 0) {
		  if (leading_zero || !leading || (k == 3))
		    digits[k] = encodeDigit(d);
	      else
		    digits[k] = 0;
		}
		else {
			digits[k] = encodeDigit(d);
			num -= d * divisor;
			leading = false;
		}
	}
	
	setSegments(digits + (4 - length), length, pos);
}

void TM1637Display::bitDelay()
{
	delayMicroseconds(50);
}
   
void TM1637Display::start()
{
  pinMode(m_pinDIO, OUTPUT);
  bitDelay();
}
   
void TM1637Display::stop()
{
	pinMode(m_pinDIO, OUTPUT);
	bitDelay();
	pinMode(m_pinClk, INPUT);
	bitDelay();
	pinMode(m_pinDIO, INPUT);
	bitDelay();
}
  
bool TM1637Display::writeByte(uint8_t b)
{
  uint8_t data = b;

  // 8 Data Bits
  for(uint8_t i = 0; i < 8; i++) {
    // CLK low
    pinMode(m_pinClk, OUTPUT);
    bitDelay();
    
	// Set data bit
    if (data & 0x01)
      pinMode(m_pinDIO, INPUT);
    else
      pinMode(m_pinDIO, OUTPUT);
    
    bitDelay();
	
	// CLK high
    pinMode(m_pinClk, INPUT);
    bitDelay();
    data = data >> 1;
  }
  
  // Wait for acknowledge
  // CLK to zero
  pinMode(m_pinClk, OUTPUT);
  pinMode(m_pinDIO, INPUT);
  bitDelay();
  
  // CLK to high
  pinMode(m_pinClk, INPUT);
  bitDelay();
  uint8_t ack = digitalRead(m_pinDIO);
  if (ack == 0)
    pinMode(m_pinDIO, OUTPUT);
	
	
  bitDelay();
  pinMode(m_pinClk, OUTPUT);
  bitDelay();
  
  return ack;
}

uint8_t TM1637Display::encodeDigit(uint8_t digit)
{
	return digitToSegment[digit & 0x0f];
}


/* C wrapper functions */
TM1637_Handle_t TM1637_Init(uint8_t pinClk, uint8_t pinDIO)
{
    return (TM1637_Handle_t)new TM1637Display(pinClk, pinDIO);
}

void TM1637_Destroy(TM1637_Handle_t handle)
{
    delete (TM1637Display*)handle;
}

void TM1637_setBrightness(TM1637_Handle_t handle, uint8_t brightness)
{
    ((TM1637Display*)handle)->setBrightness(brightness);
}

void TM1637_setSegments(TM1637_Handle_t handle, const uint8_t segments[], uint8_t length, uint8_t pos)
{
    ((TM1637Display*)handle)->setSegments(segments, length, pos);
}

void TM1637_showNumberDec(TM1637_Handle_t handle, int num, bool leading_zero, uint8_t length, uint8_t pos)
{
    ((TM1637Display*)handle)->showNumberDec(num, leading_zero, length, pos);
}

uint8_t TM1637_encodeDigit(TM1637_Handle_t handle, uint8_t digit)
{
    ((TM1637Display*)handle)->encodeDigit(digit);
}
