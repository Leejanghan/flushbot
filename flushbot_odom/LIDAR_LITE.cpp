#include <Arduino.h>
#include <Wire.h>
#include <stdarg.h>
#include "LIDAR_LITE.h"

LIDAR_Lite::LIDAR_Lite(){}

void LIDAR_Lite::begin(int configuration, bool fasti2c, char lidarliteAddress)
{
  Wire.begin(); // Start I2C
  configure(configuration, lidarliteAddress); // Configuration settings
} /* LIDARLite::begin */

void LIDAR_Lite::configure(int configuration, char lidarliteAddress)
{
  switch (configuration)
  {
    case 0: // Default mode, balanced performance
      write(0x02,0x80,lidarliteAddress); // Default
      write(0x04,0x08,lidarliteAddress); // Default
      write(0x1c,0x00,lidarliteAddress); // Default
    break;

    case 1: // Short range, high speed
      write(0x02,0x1d,lidarliteAddress);
      write(0x04,0x08,lidarliteAddress); // Default
      write(0x1c,0x00,lidarliteAddress); // Default
    break;

    case 2: // Default range, higher speed short range
      write(0x02,0x80,lidarliteAddress); // Default
      write(0x04,0x00,lidarliteAddress);
      write(0x1c,0x00,lidarliteAddress); // Default
    break;

    case 3: // Maximum range
      write(0x02,0xff,lidarliteAddress);
      write(0x04,0x08,lidarliteAddress); // Default
      write(0x1c,0x00,lidarliteAddress); // Default
    break;

    case 4: // High sensitivity detection, high erroneous measurements
      write(0x02,0x80,lidarliteAddress); // Default
      write(0x04,0x08,lidarliteAddress); // Default
      write(0x1c,0x80,lidarliteAddress);
    break;

    case 5: // Low sensitivity detection, low erroneous measurements
      write(0x02,0x80,lidarliteAddress); // Default
      write(0x04,0x08,lidarliteAddress); // Default
      write(0x1c,0xb0,lidarliteAddress);
    break;
  }
} /* LIDARLite::configure */

void LIDAR_Lite::setI2Caddr(char newAddress, char disableDefault, char lidarliteAddress)
{
  byte dataBytes[2];

  // Read UNIT_ID serial number bytes and write them into I2C_ID byte locations
  read ((0x16 | 0x80), 2, dataBytes, false, lidarliteAddress);
  write(0x18, dataBytes[0], lidarliteAddress);
  write(0x19, dataBytes[1], lidarliteAddress);

  // Write the new I2C device address to registers
  dataBytes[0] = newAddress;
  write(0x1a, dataBytes[0], lidarliteAddress);

  // Enable the new I2C device address using the default I2C device address
  dataBytes[0] = 0;
  write(0x1e, dataBytes[0], lidarliteAddress);

  // If desired, disable default I2C device address (using the new I2C device address)
  if (disableDefault)
  {
    dataBytes[0] = (1 << 3); // set bit to disable default address
    write(0x1e, dataBytes[0], newAddress);
  }
} /* LIDARLite::setI2Caddr */

void LIDAR_Lite::reset(char lidarliteAddress)
{
  write(0x00,0x00,lidarliteAddress);
} /* LIDARLite::reset */

int LIDAR_Lite::distance(bool biasCorrection, char lidarliteAddress)
{
  if(biasCorrection)
  {
    // Take acquisition & correlation processing with receiver bias correction
    write(0x00,0x04,lidarliteAddress);
  }
  else
  {
    // Take acquisition & correlation processing without receiver bias correction
    write(0x00,0x03,lidarliteAddress);
  }
  // Array to store high and low bytes of distance
  byte distanceArray[2];
  // Read two bytes from register 0x8f (autoincrement for reading 0x0f and 0x10)
  read(0x8f,2,distanceArray,true,lidarliteAddress);
  // Shift high byte and add to low byte
  int distance = (distanceArray[0] << 8) + distanceArray[1];
  return(distance);
} /* LIDARLite::distance */

void LIDAR_Lite::write(char myAddress, char myValue, char lidarliteAddress)
{
  Wire.beginTransmission((int)lidarliteAddress);
  Wire.write((int)myAddress); // Set register for write
  Wire.write((int)myValue); // Write myValue to register

  // A nack means the device is not responding, report the error over serial
  int nackCatcher = Wire.endTransmission();
  if(nackCatcher != 0)
  {
    Serial.println("> nack");
  }

  delay(1); // 1 ms delay for robustness with successive reads and writes
} /* LIDARLite::write */

void LIDAR_Lite::read(char myAddress, int numOfBytes, byte arrayToSave[2], bool monitorBusyFlag, char lidarliteAddress)
{
  int busyFlag = 0; // busyFlag monitors when the device is done with a measurement
  if(monitorBusyFlag)
  {
    busyFlag = 1; // Begin read immediately if not monitoring busy flag
  }
  int busyCounter = 0; // busyCounter counts number of times busy flag is checked, for timeout

  while(busyFlag != 0) // Loop until device is not busy
  {
    // Read status register to check busy flag
    Wire.beginTransmission((int)lidarliteAddress);
    Wire.write(0x01); // Set the status register to be read

    // A nack means the device is not responding, report the error over serial
    int nackCatcher = Wire.endTransmission();
    if(nackCatcher != 0)
    {
      Serial.println("> nack");
    }

    Wire.requestFrom((int)lidarliteAddress,1); // Read register 0x01
    busyFlag = bitRead(Wire.read(),0); // Assign the LSB of the status register to busyFlag

    busyCounter++; // Increment busyCounter for timeout

    // Handle timeout condition, exit while loop and goto bailout
    if(busyCounter > 9999)
    {
      goto bailout;
    }
  }

  // Device is not busy, begin read
  if(busyFlag == 0)
  {
    Wire.beginTransmission((int)lidarliteAddress);
    Wire.write((int)myAddress); // Set the register to be read

    // A nack means the device is not responding, report the error over serial
    int nackCatcher = Wire.endTransmission();
    if(nackCatcher != 0)
    {
      Serial.println("> nack");
    }

    // Perform read of 1 or 2 bytes, save in arrayToSave
    Wire.requestFrom((int)lidarliteAddress, numOfBytes);
    int i = 0;
    if(numOfBytes <= Wire.available())
    {
      while(i < numOfBytes)
      {
        arrayToSave[i] = Wire.read();
        i++;
      }
    }
  }

  // bailout reports error over serial
  if(busyCounter > 9999)
  {
    bailout:
      busyCounter = 0;
      Serial.println("> read failed");
  }
} 

void LIDAR_Lite::correlationRecordToSerial(char separator, int numberOfReadings, char lidarliteAddress)
{

  // Array to store read values
  byte correlationArray[2];
  // Var to store value of correlation record
  int correlationValue = 0;
  //  Selects memory bank
  write(0x5d,0xc0,lidarliteAddress);
  // Test mode enable
  write(0x40, 0x07,lidarliteAddress);
  for(int i = 0; i<numberOfReadings; i++){
    // Select single byte
    read(0xd2,2,correlationArray,false,lidarliteAddress);
    //  Low byte is the value of the correlation record
    correlationValue = correlationArray[0];
    // if upper byte lsb is set, the value is negative
    if((int)correlationArray[1] == 1){
      correlationValue |= 0xff00;
    }
    Serial.print((int)correlationValue);
    Serial.print(separator);
  }
  // test mode disable
  write(0x40,0x00,lidarliteAddress);
} 
