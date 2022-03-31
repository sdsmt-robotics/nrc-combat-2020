#include <Arduino.h>

#include "adc.h"

#define ADC_SELECT 23
#define ADC_RX 19
#define ADC_TX 18

#define CMD_POWERDOWN 0x02
#define CMD_START_SYNC 0x08
#define CMD_RESET 0x06
#define CMD_RDATA 0x10
#define CMD_RREG 0x20
#define CMD_WREG 0x40
#define SYNC_BYTE 0x55

enum {
  ADC1 = 0,
  ADC2 = 1
}ADC_NUM;

uint8_t registerWrite(uint8_t address, uint8_t data)
{
  //
  // Check that the register address is in range
  //

  Serial1.write(SYNC_BYTE);
  Serial1.write(CMD_WREG | (address << 1));
  Serial1.write(data);

  return 0;
}

uint8_t registerRead(uint8_t address)
{
  //
  // Check that the register address is in range

  uint8_t regValue = 0;
  Serial1.write(SYNC_BYTE);
  Serial1.write(CMD_RREG | (address << 1));
  regValue = Serial1.read();
  return regValue;
}

void write_ADC(bool adc_num,uint8_t register_addr,uint8_t data)
{
  digitalWrite(ADC_SELECT, adc_num);
  Serial.print("Writing to ADC ");
  Serial.println(adc_num+1);
  delay(1);
  registerWrite(register_addr,data);


}

uint8_t read_ADC(bool adc_num,uint8_t register_addr)
{
  digitalWrite(ADC_SELECT, adc_num);
  delay(1);
  return registerRead(0x04);
  
}
void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, ADC_RX, ADC_TX);

  // Serial.println("Starting main loop.");

  pinMode(ADC_SELECT, OUTPUT);
  pinMode(ADC_RX, OUTPUT);
  pinMode(ADC_TX, INPUT);

  digitalWrite(ADC_SELECT, HIGH);
}

void loop()
{
  // Serial.println("send!");
  // Serial1.write(0x55);
  // Serial1.write(0b01001000);
  // Serial1.write(0b00000110);
  // // Serial2.write(0x55);
  // // Serial2.write(0b01001000);
  // // Serial2.write(17);
  // // That's no ordinary rabbit!
  // // Serial.println("Value written");

  // delay(100);

  // Serial1.write(0x55);
  // Serial1.write(0b00101000);
  // // while(!Serial1.available()){
  // //   uint8_t readValue = Serial1.read();
  // // }
  // uint8_t readValue = Serial1.read();
  // // uint8_t readValue = Serial1.write(0b00101000);

  // Serial.println(readValue);

  // delay(100);

  write_ADC(ADC1,0x04,0xAA);
  delay(1);
  write_ADC(ADC2,0x04,0x55);



  delay(1);
  uint8_t data1 = read_ADC(ADC2,0x04);
  delay(1);
  uint8_t data2 = read_ADC(ADC1,0x04);

  Serial.print("ADC1: ");
  Serial.print(data1,HEX);
  Serial.print("\tADC2: ");
  Serial.println(data2,HEX);

  
  delay(1000);

  // digitalWrite(ADC_SELECT,LOW);

  // Serial.println("Writing to ADC 2");
  // registerWrite(0x04,0x18);
  // delay(1000);

  // Serial.print("reading ADC 2: ");
  // Serial.println(registerRead(0x04),HEX);
  // delay(1000);
  
  // Serial.println("Reading Data");
  // Serial1.write(0x55);
  // Serial1.write(0b00010110);
  // while (Serial1.available() == 0){
  //   delay(10);
  // }
  // Serial.println(Serial1.read());
  // Serial.println(Serial1.read());
  // Serial.println(Serial1.read());
  // delay(1000);


}