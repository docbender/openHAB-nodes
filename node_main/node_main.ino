#include "Arduino.h"
#include "avr/power.h"
#include "avr/sleep.h"
#include "DmxSimple.h"
//#include <DmxSimple.h>
//#include <SPI.h>
#include <Wire.h>
#include "Adafruit_HTU21DF.h"

#include "simpleBinary.h"


// UART
#define UART_SPEED  9600
#define UART_ADDRESS 1
#define RTS_PIN 2
// LED
#define LED_PIN 13
// DMX
#define DMX_PIN 7

//definice promennych vymeny dat
simpleBinary *items;
//HTU object
Adafruit_HTU21DF htu = Adafruit_HTU21DF();

volatile int secCounter = 0;
int secCounterOld = 0;

void setup() {
  //turn off unused parts
  power_adc_disable();
  power_spi_disable();
  power_timer2_disable();
  //enable timer1
  power_timer1_enable();
  //set sleep mode idle
  set_sleep_mode(SLEEP_MODE_IDLE); 
  
  //inicializace promennych
  items = new simpleBinary(UART_ADDRESS,3);
  //nastaveni ustaleni linky
  items->setSendDelay(100);

  //rgb
  items->initItem(0,100,RGBW, executeRGB);
  //temp HTU
  items->initItem(1,101,FLOAT, NULL);
  //hum HTU
  items->initItem(2,102,FLOAT, NULL);
//
//  items->initItem(3,1,BYTE, executeData);
//  items->initItem(4,2,WORD, executeData);  
//  items->initItem(5,11,BYTE, executeData);
//  items->initItem(6,12,WORD, executeData);
//  items->initItem(7,3,DWORD, executeData);
//  items->initItem(8,13,DWORD, executeData);
//  items->initItem(9,4,FLOAT, executeData);
//  items->initItem(10,14,FLOAT, executeData);  
//  items->initItem(11,20,RGB, executeData);  
//  items->initItem(12,21,RGB, executeData);  
  
  //nastaveni LED vystupu
  pinMode(LED_PIN, OUTPUT);
  //nastaveni RTS pinu
  items->enableRTS(RTS_PIN);
  
  blink();

  //inicializace seriovky
  Serial.begin(UART_SPEED);
  UCSR0B |= 0x80;
  //init HTU
  //htu.begin();
  //initDMX
  DmxSimple.usePin(DMX_PIN);
  DmxSimple.maxChannel(4); 
  writeDmx(1,0,0,0,0);
  //timer1
  setupTimer1();

  interrupts();
}

void loop() 
{  
  // check if data has been sent from the computer
  if (Serial.available()) 
  {
    items->processSerial();
  }
  else
  {    
    delay(10);

    if (!Serial.available()) 
    {
      DmxSimple.sendData();
      
      digitalWrite(LED_PIN, LOW);
      
      goSleep();
      
      digitalWrite(LED_PIN, HIGH);

      //executeRGB(&(*items)[0]);      
    }

    if(secCounter != secCounterOld)
    {
      if(secCounter >= 30)
      {
        readHtu();

        secCounter = 0;
      }
            
      secCounterOld = secCounter;

//      if(secCounter % 2 == 1)
//        digitalWrite(LED_PIN, HIGH);
//      else
//        digitalWrite(LED_PIN, LOW);
    }
  }
}

void readHtu()
{
    if(htu.begin())
    {
      (*items)[1].save(htu.readTemperature());
      (*items)[1].setNewData();
      (*items)[2].save(htu.readHumidity());
      (*items)[2].setNewData();      
    }
}

void goSleep()
{
  sleep_enable();
  power_timer0_disable();
  //power_timer2_disable();
  power_twi_disable();
  
  sleep_mode();
  
  sleep_disable();
  power_timer0_enable();
  //power_timer2_enable();
  power_twi_enable();
}

void setupTimer1()
{
  TCCR1A = 0; //
  TCCR1B = (1 << WGM12) | 5; // t/1024 prescaler
  OCR1AH = 0x3D;
  OCR1AL = 0x09;
  
  TIMSK1 = 2; //OCR1A interrupt

  secCounter = 0;
}

ISR(TIMER1_COMPA_vect) 
{   
  secCounter++;
}


void blink()
{
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);
  delay(500);
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);
  delay(500);
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);
  delay(500); 
}

///
void executeData(itemData *item)
{
  if(item->getAddress() == 1)
  {
    (*items)[5].saveByte(item->getData());
    (*items)[5].setNewData();
  }
  else if(item->getAddress() == 2)
  {
    (*items)[6].saveWord(item->getData());
    (*items)[6].setNewData();
  }
  else if(item->getAddress() == 3)
  {
    (*items)[8].saveDword(item->getData());
    (*items)[8].setNewData();
  }
  else if(item->getAddress() == 4)
  {
    (*items)[10].saveDword(item->getData());
    (*items)[10].setNewData();
  } 
  else if(item->getAddress() == 20)
  {
    (*items)[12].saveDword(item->getData());
    (*items)[12].setNewData();
  } 
}

void executeRGB(itemData *item)
{
  if(item->getType() == RGBW)
  {
    char *data = item->getData();
    
    uint8_t red = data[0];
    uint8_t green = data[1];
    uint8_t blue = data[2];
    uint8_t white = data[3];
    
    writeDmx(1,red,green,blue,white);
  }
}

void writeDmx(int address, int red, int green, int blue, int white)
{
    DmxSimple.write(address, red);
    DmxSimple.write(address+1, green);
    DmxSimple.write(address+2, blue);
    DmxSimple.write(address+3, white);
}




