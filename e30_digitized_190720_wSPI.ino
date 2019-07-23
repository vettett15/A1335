

#include <Wire.h> //standard library
#include <Adafruit_MCP4725.h> //library for the DAC
#include <SPI.h> //standard library
#include <avr/wdt.h>
#include "Definitions.h" //library developed for the A1335 sensor

float sensorVoltage = 0; //setting sensorVoltage variable which will represent Vin
float outputVoltage = 0; //setting outputVoltage variable which will represent Vout
const int TEMPERATURE = 0x28; //from A1335 Datasheet
const byte READ = 0b11111100;

Adafruit_MCP4725 dac;

//Code below is to read Vin, which will be used later as the "reference voltage" for the DAC

long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  //#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
     ADMUX = _BV(MUX5) | _BV(MUX0) ;
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  
 
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring
 
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both
 
  long result = (high<<8) | low;
 
  result = 1105300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}

//End code to read Vin

void setup(void) {

  Serial.begin(9600);

 
  SPI.begin();

  pinMode (SS,OUTPUT);//Sets select slave pin as an output, I think this may redundant since I already call out SPI.h



//Initiate MCP4725 DAC
  dac.begin(0x62);  // For Adafruit MCP4725A1 the address is 0x62 (default
    TWBR = ((F_CPU /400000l) - 16) / 2; // Change the i2c clock to 400KHz

}


void loop(void) {


//Block is to set sensorVoltage equal to the readVcc() and display on the serial monitor    
  Serial.println("The Sensor Voltage Input is:  ");
  float sensorVoltage = readVcc();
  Serial.print(sensorVoltage);
  Serial.println(" ");


//Using this array to allow me to make adjustments between what the input voltage is and what the DAC outputs as Vout
float array[42] = {
// Voltage From Sensor on left // Corresponding Desired Output Voltage on right
0, 0,
250, 250,
500, 500,
750, 750, 
1000, 1000,
1250, 1250,
1500, 1500,
1750, 1750,
2000, 2000,
2250, 2250,
2500, 2500,
2750, 2750,
3000, 3000,
3250, 3250,
3500, 3500,
3750, 3750,
4000, 4000,
4250, 4250,
4500, 4500,
4750, 300,
5500, 400,
};

//Displays the top of the array for the Input Voltage (left side of array), used for debugging
Serial.println("The top of the array is: ");
Serial.println(array[40]);

//if and else if is if the input voltage value is outside the table I want to clamp it to the lowest or highest values
if (sensorVoltage<= array[0])
{
  outputVoltage = dac.setNearestActualVoltage(array[1], sensorVoltage, false);
  Serial.println("Your below the bottom end of the table");
}
else if (sensorVoltage >= array[40])
{
  outputVoltage = dac.setNearestActualVoltage(sensorVoltage, sensorVoltage, false);
  Serial.println("Your above the top end of the table");
}

//block is performing a linear interpolation for when the input voltage is between values on the table
int x;
  for (x= 0; x<42; x=x+2){
  if (sensorVoltage >=array[x]
&& sensorVoltage <=array[x+2])

        outputVoltage = dac.setNearestActualVoltage((array[x+1] + ( (array[x+3] - array[x+1]) * ( (sensorVoltage - array[x]
) / (array[x+2] - array[x]
) ) )),sensorVoltage, false);
Serial.println("The Output Voltage is:  ");
Serial.println(outputVoltage);
}


//code below is attempting to read from the A1335

SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE3)); // start the SPI library for A1335, settings out of datasheet
delay(100); //give A1335 sensor time to setup

digitalWrite (SS, LOW); //sets select slave low to allow for communication

int tempData = readRegister (0x28, 2);


Serial.print("raw tempdata for 0x28");
Serial.println();
Serial.println(tempData);
Serial.println();
delay(1000);


float realTemp = ((float)tempData / 8.0 -273.145);
Serial.print ("Temp[C]=");
Serial.print(realTemp);

int MagStrength = readRegister (0x2A, 2);

Serial.print ("Magnetic Strength");
Serial.print(MagStrength);

delay(1000);

}

//Read from register from the A1335:
unsigned int readRegister(byte thisRegister, int bytesToRead) {
  byte inByte = 0;           // incoming byte from the SPI
  unsigned int result = 0;   // result to return
  Serial.print(thisRegister, BIN);
  Serial.print("\t");
  // A1335 expects the register name in the upper 4 bits
  // of the byte. So shift the bits left by two bits:
  thisRegister = thisRegister << 2;
  // now combine the address and the command into one byte
  byte dataToSend = thisRegister & READ;
  Serial.println(thisRegister, BIN);
  // take the chip select low to select the device:
  digitalWrite(SS, LOW);
  // send the device the register you want to read:
  SPI.transfer(dataToSend);
  // send a value of 0 to read the first byte returned:
  result = SPI.transfer(0x00);
  // decrement the number of bytes left to read:
  bytesToRead--;
  // if you still have another byte to read:
  if (bytesToRead > 0) {
    // shift the first byte left, then get the second byte:
    result = result << 8;
    inByte = SPI.transfer(0x00);
    // combine the byte you just got with the previous one:
    result = result | inByte;
    // decrement the number of bytes left to read:
    bytesToRead--;
  }
  // take the chip select high to de-select:
  digitalWrite(SS, HIGH);
  // return the result:
  return (result);
}
