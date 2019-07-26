/*
* Example source code for an Arduino to show how
* to use SPI to communicate with an Allegro A1335
*
* Written by K. Robert Bate, Allegro MicroSystems, LLC.
*
* A1335SPIExample is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/
#include <SPI.h>
#define kNOERROR 0
#define kPRIMARYREADERROR 1
#define kEXTENDEDREADTIMEOUTERROR 2
#define kPRIMARYWRITEERROR 3
#define kEXTENDEDWRITETIMEOUTERROR 4
#define kCRCERROR 5
#define kUNABLETOCHANGEPROCESSORSTATE 6

const uint16_t LEDPin = 13;
const uint32_t WRITE = 0x40;
const uint32_t READ = 0x00;
const uint32_t COMMAND_MASK = 0xC0;
const uint32_t ADDRESS_MASK = 0x3F;
unsigned long nextTime;
bool ledOn = false;
bool includeCRC = true;
void setup()
{
uint16_t unused;
uint32_t flags;
uint16_t angle;
uint32_t flagsAndZeroOffset;
// Initialize SPI
SPI.begin();
pinMode(SS, OUTPUT);

pinMode(LEDPin, OUTPUT);
// Initialize serial
Serial.begin(9600);
// If the Arduino has built in USB, keep the next line
// in to wait for the Serial to initialize
while (!Serial);

nextTime = millis();
digitalWrite(LEDPin, LOW);
digitalWrite(SS, HIGH);
// Make sure all of the SPI pins are
// ready by doing a read
PrimaryRead(SS, 0x0, unused);
// Unlock the device
ExtendedWrite(SS, 0xFFFE, 0x27811F77);
// Make sure the device is unlocked
ExtendedRead(SS, 0x22, flags);
if (!((flags & 0x0022) == 0x0020))
{
Serial.println("Device is not Unlocked");
}
// Zero the angle
// Extended location 0x06 contains flags in the MSW and the Zero Angle values in the LSW
// so get both and zero out ZeroAngle
ExtendedRead(SS, 0x06, flagsAndZeroOffset);
flagsAndZeroOffset = (flagsAndZeroOffset & 0xFFFF0000);
ExtendedWrite(SS, 0x06, flagsAndZeroOffset);
// Get the current angle. It is now without the ZeroAngle correction
PrimaryRead(SS, 0x20, angle);
// Copy the read angle into location 0x5C preserving the flags
flagsAndZeroOffset = (flagsAndZeroOffset & 0xFFFF0000) | ((angle << 4) & 0x0000FFFF);
ExtendedWrite(SS, 0x06, flagsAndZeroOffset);
}
void loop()
{
uint16_t angle;
uint16_t temperature;
uint16_t fieldStrength;
// Every second, read the angle, temperature and field strength
if (nextTime < millis())
{
if (PrimaryRead(SS, 0x20, angle) == kNOERROR)
{
if (CalculateParity(angle))
{
Serial.print("Angle = ");
Serial.print((float)(angle & 0x0FFF) * 360.0 / 4096.0);
Serial.println(" Degrees");
}
else
{
Serial.println("Parity error on Angle read");
}
}
else
{
Serial.println("Unable to read Angle");
}
if (PrimaryRead(SS, 0x28, temperature) == kNOERROR)
{

Serial.print("Temperature = ");
Serial.print(((float)(temperature & 0x0FFF) / 8.0) - 273.15);
Serial.println(" C");
}
else
{
Serial.println("Unable to read Temperature");
}
if (PrimaryRead(SS, 0x2A, fieldStrength) == kNOERROR)
{
Serial.print("Field Strength = ");
Serial.print(fieldStrength & 0x0FFF);
Serial.println("Gauss");
}
else
{
Serial.println("Unable to read Field Strength");
}
nextTime = millis() + 500L;
// Blink the LED every half second
if (ledOn)
{
digitalWrite(LEDPin, LOW);
ledOn = false;
}
else
{
digitalWrite(LEDPin, HIGH);
ledOn = true;
}
}
}
/*
* PrimaryRead
*
* Read from the primary serial registers
*/
uint16_t PrimaryRead(uint16_t cs, uint16_t address, uint16_t& value)
{
uint16_t command;
if (includeCRC)
{
uint8_t crcValue;
uint8_t crcCommand;
SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
// On the Teensy, SPI0_CTAR0 is used to describe the SPI transaction for transfer (byte)
// while SPI0_CTAR1 is used to describe the SPI transaction for transfer16.
// To do a 20 bit transfer, change the length of the transaction to 4 bits for transfer
// and do a transfer16 followed by a transfer.

// Combine the register address and the command into one byte
command = ((address & ADDRESS_MASK) | READ) << 8;
crcCommand = CalculateCRC(command);

// take the chip select low to select the device
digitalWrite(cs, LOW);
// send the device the register you want to read
SPI.transfer16(command);
SPI.transfer(crcCommand);
digitalWrite(cs, HIGH);
digitalWrite(cs, LOW);
// send the command again to read the contents
value = SPI.transfer16(command);
crcValue = SPI.transfer(crcCommand);
// take the chip select high to de-select
digitalWrite(cs, HIGH);
// Restore the 8 bit description

SPI.endTransaction();
// Check the CRC value
if (CalculateCRC(value) != crcValue)
{
return kCRCERROR;
}
}
else
{
SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
// Combine the register address and the command into one byte
command = ((address & ADDRESS_MASK) | READ) << 8;
// take the chip select low to select the device
digitalWrite(cs, LOW);
// send the device the register you want to read
SPI.transfer16(command);
digitalWrite(cs, HIGH);
digitalWrite(cs, LOW);
// send the command again to read the contents
value = SPI.transfer16(command);
// take the chip select high to de-select
digitalWrite(cs, HIGH);
SPI.endTransaction();
}
return kNOERROR;
}
/*
* PrimaryRead
*
* Read from the primary serial registers
*/

void PrimaryRead(uint16_t cs, uint16_t* addresses, uint16_t* values, uint16_t* errors, uint16_t numberOfItems)
{
uint16_t command;
if (includeCRC)
{
uint8_t crcValue;
uint8_t crcCommand;
SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
// On the Teensy, SPI0_CTAR0 is used to describe the SPI transaction
// for transfer (byte) while SPI0_CTAR1 is used to describe the SPI
// transaction for transfer16.
// To do a 20 bit transfer, change the length of the transaction to
// 4 bits for transfer and do a transfer16 followed by a transfer.


// to send 4 bits
// Combine the register address and the command into one byte
command = ((addresses[0] & ADDRESS_MASK) | READ) << 8;
crcCommand = CalculateCRC(command);
// take the chip select low to select the device
digitalWrite(cs, LOW);
// send the device the register you want to read
SPI.transfer16(command);
SPI.transfer(crcCommand);
digitalWrite(cs, HIGH);
for (int index = 0; index < (numberOfItems - 1); ++index)
{
// Combine the register address and the command into one byte
command = ((addresses[index + 1] & ADDRESS_MASK) | READ) << 8;
crcCommand = CalculateCRC(command);
digitalWrite(cs, LOW);
// send the command again to read the contents
values[index] = SPI.transfer16(command);
crcValue = SPI.transfer(crcCommand);
// take the chip select high to de-select
digitalWrite(cs, HIGH);
// Check the CRC of the read value
if (CalculateCRC(values[index]) != crcValue)
{
errors[index] = kCRCERROR;
}
else
{
errors[index] = kNOERROR;
}
}
digitalWrite(cs, LOW);

// send the command again to read the contents
values[numberOfItems - 1] = SPI.transfer16(command);
crcValue = SPI.transfer(crcCommand);
// take the chip select high to de-select
digitalWrite(cs, HIGH);
// Check the CRC value
if (CalculateCRC(values[numberOfItems - 1]) != crcValue)
{
errors[numberOfItems - 1] = kCRCERROR;
}
else
{
errors[numberOfItems - 1] = kNOERROR;
}
// Restore the 8 bit description

SPI.endTransaction();
}
else
{
SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
// Combine the register address and the command into one byte
command = ((addresses[0] & ADDRESS_MASK) | READ) << 8;
// take the chip select low to select the device
digitalWrite(cs, LOW);
// send the device the register you want to read
SPI.transfer16(command);
digitalWrite(cs, HIGH);
for (int index = 0; index < (numberOfItems - 1); ++index)
{
command = ((addresses[index + 1] & ADDRESS_MASK) | READ) << 8;
digitalWrite(cs, LOW);
// send the command again to read the contents
values[index] = SPI.transfer16(command);
// take the chip select high to de-select
digitalWrite(cs, HIGH);
errors[index] = kNOERROR;
}
digitalWrite(cs, LOW);
// send the command again to read the contents
values[numberOfItems - 1] = SPI.transfer16(command);
errors[numberOfItems - 1] = kNOERROR;
// take the chip select high to de-select
digitalWrite(cs, HIGH);
SPI.endTransaction();

}
}
/*
* PrimaryWrite
*
* Write to the primary serial registers
*/
uint16_t PrimaryWrite(uint16_t cs, uint16_t address, uint16_t value)
{
uint16_t command;
if (includeCRC)
{
uint8_t crcCommand;
SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
// On the Teensy, SPI0_CTAR0 is used to describe the SPI transaction for transfer
// while SPI0_CTAR1 is used to describe the SPI transaction for transfer16.
// To do a 20 bit transfer, change the length of the transaction to 4 bits for transfer
// and do a transfer16 followed by a transfer.


// Combine the register address and the command into one byte
command = (((address & ADDRESS_MASK) | WRITE) << 8) | ((value >> 8) & 0x0FF);
crcCommand = CalculateCRC(command);
// take the chip select low to select the device:
digitalWrite(cs, LOW);
SPI.transfer16(command); // Send most significant byte of register data
SPI.transfer(crcCommand); // Send the crc
// take the chip select high to de-select:
digitalWrite(cs, HIGH);
command = ((((address + 1) & ADDRESS_MASK) | WRITE) << 8 ) | (value & 0x0FF);
crcCommand = CalculateCRC(command);
// take the chip select low to select the device:
digitalWrite(cs, LOW);
SPI.transfer16(command); // Send least significant byte of register data
SPI.transfer(crcCommand); // Send the crc
// take the chip select high to de-select:
digitalWrite(cs, HIGH);
// Restore the 8 bit description

SPI.endTransaction();
}
else
{
SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
// Combine the register address and the command into one byte
command = (((address & ADDRESS_MASK) | WRITE) << 8) | ((value >> 8) & 0x0FF);
// take the chip select low to select the device:
digitalWrite(cs, LOW);
SPI.transfer16(command); // Send most significant byte of register data
// take the chip select high to de-select:
digitalWrite(cs, HIGH);
command = ((((address + 1) & ADDRESS_MASK) | WRITE) << 8) | (value & 0x0FF);
// take the chip select low to select the device:
digitalWrite(cs, LOW);
SPI.transfer16(command); // Send least significant byte of register data
// take the chip select high to de-select:
digitalWrite(cs, HIGH);
SPI.endTransaction();
}
return kNOERROR;
}
/*
* PrimaryWrite
*
* Write to the primary serial registers
*/
void PrimaryWrite(uint16_t cs, uint16_t* addresses, uint16_t* values, uint16_t* errors, uint16_t numberOfItems)
{
uint16_t command;
SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
if (includeCRC)
{
uint8_t crcCommand;
// On the Teensy, SPI0_CTAR0 is used to describe the SPI transaction
// for transfer (byte) while SPI0_CTAR1 is used to describe the SPI
// transaction for transfer16.
// To do a 20 bit transfer, change the length of the transaction to
// 4 bits for transfer and do a transfer16 followed by a transfer.

for (int index = 0; index < numberOfItems; ++index)
{
// Combine the register address and the command into one byte
command = (((addresses[index] & ADDRESS_MASK) | WRITE) << 8) | ((values[index] >> 8) & 0x0FF);
crcCommand = CalculateCRC(command);
// take the chip select low to select the device
digitalWrite(cs, LOW);
SPI.transfer16(command); // Send most significant byte of register data
SPI.transfer(crcCommand); // Send the crc
// take the chip select high to de-select
digitalWrite(cs, HIGH);
command = ((((addresses[index] + 1) & ADDRESS_MASK) | WRITE) << 8 ) | (values[index] & 0x0FF);
crcCommand = CalculateCRC(command);
// take the chip select low to select the device
digitalWrite(cs, LOW);
SPI.transfer16(command); // Send least significant byte of register data
SPI.transfer(crcCommand); // Send the crc
// take the chip select high to de-select
digitalWrite(cs, HIGH);
errors[index] = kNOERROR;
}

}
else
{
for (int index = 0; index < numberOfItems; ++index)
{
// Combine the register address and the command into one byte
command = (((addresses[index] & ADDRESS_MASK) | WRITE) << 8) | ((values[index] >> 8) & 0x0FF);
// take the chip select low to select the device
digitalWrite(cs, LOW);
SPI.transfer16(command); // Send most significant byte of register data
// take the chip select high to de-select
digitalWrite(cs, HIGH);
command = ((((addresses[index] + 1) & ADDRESS_MASK) | WRITE) << 8 ) | (values[index] & 0x0FF);
// take the chip select low to select the device
digitalWrite(cs, LOW);
SPI.transfer16(command); // Send least significant byte of register data
// take the chip select high to de-select:
digitalWrite(cs, HIGH);
errors[index] = kNOERROR;
}
}
SPI.endTransaction();
}
/*
* ExtendedRead
*
* Read from the SRAM, EEPROM, AUX or Extended Registers
*/
uint16_t ExtendedRead(uint16_t cs, uint16_t address, uint32_t& value)
{
uint16_t results;
uint16_t readFlags;
uint32_t timeout;
uint32_t currentTime;
// Write the address to the Extended Read Address register
results = PrimaryWrite(cs, 0x0A, address & 0xFFFF);
if (results != kNOERROR)
{
return results;
}
// Initiate the extended read
results = PrimaryWrite(cs, 0x0C, 0x8000);
if (results != kNOERROR)
{
return results;
}
timeout = millis() + 100L;
do // Wait for the read to be complete
{
results = PrimaryRead(cs, 0x0C, readFlags);
if (results != kNOERROR)
{
return results;
}
// Make sure the read is not taking too long
currentTime = millis();
if (timeout < currentTime)
{
return kEXTENDEDREADTIMEOUTERROR;
}
} while ((readFlags & 0x0001) != 0x0001);
// Read the 2 data registers
uint16_t addresses[] = {0x0E, 0x10};
uint16_t values[2];
uint16_t errors[2];
PrimaryRead(cs, addresses, values, errors, 2);
// Check all of the results of the reads
for (int index = 0; index < 2; ++index)
{
if (errors[index] != kNOERROR)
{
return errors[index];
}
}
// Combine them
value = ((uint32_t)values[0] << 16) + (uint32_t)values[1];
return results;
}
/*
* ExtendedWrite
*
* Write to the SRAM, EEPROM, AUX or Extended Access Commands
Allegro MicroSystems, LLC 18
115 Northeast Cutoff
Worcester, Massachusetts 01615-0036 U.S.A.
1.508.853.5000; www.allegromicro.com
*/
uint16_t ExtendedWrite(uint16_t cs, uint16_t address, uint32_t value)
{
uint16_t results;
uint16_t writeFlags;
uint32_t timeout;
uint16_t addresses[] = {0x02, 0x04, 0x06, 0x08};
uint16_t values[] = {address, (uint16_t)(value >> 16), (uint16_t)value, 0x8000};
uint16_t errors[4];
// Write into the extended address register
PrimaryWrite(cs, addresses, values, errors, 4);
// Check all of the results of the writes
for (int index = 0; index < 4; ++index)
{
if (errors[index] != kNOERROR)
{
return errors[index];
}
}
// If writing to the EEPROM, generate the Program
// Pulses on Vcc
if ((address >= 0x300) && (address < 0x320))
{
// Send the Program Pulses
// Need hardware which this example does not have.
}
timeout = millis() + 100;
// Wait for the write to complete
do
{
results = PrimaryRead(cs, 0x08, writeFlags);
if (results != kNOERROR)
{
return results;
}
if (timeout < millis())
{
return kEXTENDEDWRITETIMEOUTERROR;
}
} while ((writeFlags & 0x0001) != 0x0001);
return results;
}
/*
* SetProcessorStateToIdle
*
* Change the processor state to idle
* This is needed to write EEPROM, change ORATE or perform certain self tests
*/
uint16_t SetProcessorStateToIdle(uint8_t cs)
{
uint16_t results;
uint16_t value;
// Write the enter idle state command into the control register
results = PrimaryWrite(cs, 0x1F, 0x8046);
if (results == kNOERROR)
{
delay(1);
// Read the status register to see if the processor is in the idle state
results = PrimaryRead(cs, 0x22, value);
if (results == kNOERROR)
{
if ((value & 0x00FF) != 0x0010)
{
return kUNABLETOCHANGEPROCESSORSTATE;
}
}
}
return results;
}
/*
* SetProcessorStateToRun
*
* Change the processor state to run
* This is needed to process angles
*/
uint16_t SetProcessorStateToRun(uint8_t cs)
{
uint16_t results;
uint16_t value;
// Write the enter idle state command into the control register
results = PrimaryWrite(cs, 0x1F, 0xC046);
if (results == kNOERROR)
{
delay(1);
// Read the status register to see if the processor is in the idle state
results = PrimaryRead(cs, 0x22, value);
if (results == kNOERROR)
{
if ((value & 0x00FF) != 0x0011)
{
return kUNABLETOCHANGEPROCESSORSTATE;
}
}
}
return results;
}
/*
* CalculateParity
*
* From the 16 bit input, calculate the parity
*/
bool CalculateParity(uint16_t input)
{
uint16_t count = 0;
// Count up the number of 1s in the input
for (int index = 0; index < 16; ++index)
{
if ((input & 1) == 1)
{
++count;
}
input >>= 1;
}
// return true if there is an odd number of 1s
return (count & 1) != 0;
}
/*
* CalculateCRC
*
* Take the 16bit input and generate a 4bit CRC
* Polynomial = x^4 + x^1 + 1
* LFSR preset to all 1’s
*/
uint8_t CalculateCRC(uint16_t input)
{
bool CRC0 = true;
bool CRC1 = true;
bool CRC2 = true;
bool CRC3 = true;
int i;
bool DoInvert;
uint16_t mask = 0x8000;
for (i = 0; i < 16; ++i)
{
DoInvert = ((input & mask) != 0) ^ CRC3; // XOR required?
CRC3 = CRC2;
CRC2 = CRC1;
CRC1 = CRC0 ^ DoInvert;
CRC0 = DoInvert;
mask >>= 1;
}
return (CRC3 ? 8U : 0U) + (CRC2 ? 4U : 0U) + (CRC1 ? 2U : 0U) + (CRC0 ? 1U : 0U);
}
