#include <LowPower.h>

/*********************************************************************
 This is an example for our nRF51822 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
  #include <SoftwareSerial.h>
#endif

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"



/*=========================================================================
    APPLICATION SETTINGS

    FACTORYRESET_ENABLE       Perform a factory reset when running this sketch
   
                              Enabling this will put your Bluefruit LE module
                              in a 'known good' state and clear any config
                              data set in previous sketches or projects, so
                              running this at least once is a good idea.
   
                              When deploying your project, however, you will
                              want to disable factory reset by setting this
                              value to 0.  If you are making changes to your
                              Bluefruit LE device via AT commands, and those
                              changes aren't persisting across resets, this
                              is the reason why.  Factory reset will erase
                              the non-volatile memory where config data is
                              stored, setting it back to factory default
                              values.
       
                              Some sketches that require you to bond to a
                              central device (HID mouse, keyboard, etc.)
                              won't work at all with this feature enabled
                              since the factory reset will clear all of the
                              bonding data stored on the chip, meaning the
                              central device won't be able to reconnect.
    MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
    MODE_LED_BEHAVIOUR        LED activity, valid options are
                              "DISABLE" or "MODE" or "BLEUART" or
                              "HWUART"  or "SPI"  or "MANUAL"
    -----------------------------------------------------------------------*/
    #define FACTORYRESET_ENABLE         1
    #define MINIMUM_FIRMWARE_VERSION    "0.6.6"
    #define MODE_LED_BEHAVIOUR          "MODE"
/*=========================================================================*/

// Create the bluefruit object, either software serial...uncomment these lines
/*
SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
*/

/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
// Adafruit_BluefruitLE_UART ble(Serial1, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

byte mode=0;
//0==idle
//1==triggered
//2==time laspe
//3==video
unsigned long timeLapseDelay=1000;
unsigned long sleptTime=0;
const int CAMERAPIN=6;
const int TRIGGERPIN=5;
bool triggerPinStatus=0;
bool previousTriggerPinStatus=0;

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

void determineMode();
void handleCamera();
/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void)
{
  delay(5000);
  
  pinMode(CAMERAPIN, OUTPUT);
  digitalWrite(CAMERAPIN, HIGH);
  pinMode(TRIGGERPIN, INPUT_PULLUP);

   //while (!Serial);  // required for Flora & Micro
  delay(500);

  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit Command Mode Example"));
  Serial.println(F("---------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in UART mode"));
  Serial.println(F("Then Enter characters to send to Bluefruit"));
  Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  //while (! ble.isConnected()) {
      //delay(500);
  //}

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("******************************"));
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
    Serial.println(F("******************************"));
  }
}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
  
  // Check for user input
  char inputs[BUFSIZE+1];
  
  if ( getUserInput(inputs, BUFSIZE) )
  {
    // Send characters to Bluefruit
    Serial.print("[Send] ");
    Serial.println(inputs);

    ble.print("AT+BLEUARTTX=");
    ble.println(inputs);

    // check response stastus
    if (! ble.waitForOK() ) {
      Serial.println(F("Failed to send?"));
    }
  }

  // Check for incoming characters from Bluefruit
  ble.println("AT+BLEUARTRX");
  ble.readline();
  if (!strcmp(ble.buffer, "OK") == 0) 
  {
    determineMode();    
  }

  // Some data was found, its in the buffer
  Serial.print(F("[Recv] ")); Serial.println(ble.buffer);

  //mode determination//
    
  handleCamera();
  ble.waitForOK();
}

/**************************************************************************/
/*!
    @brief  Checks for user input (via the Serial Monitor)
*/
/**************************************************************************/
bool getUserInput(char buffer[], uint8_t maxSize)
{
  // timeout in 100 milliseconds
  TimeoutTimer timeout(100);

  memset(buffer, 0, maxSize);
  while( (!Serial.available()) && !timeout.expired() ) { delay(1); }

  if ( timeout.expired() ) return false;

  delay(2);
  uint8_t count=0;
  do
  {
    count += Serial.readBytes(buffer+count, maxSize);
    delay(2);
  } while( (count < maxSize) && (Serial.available()) );

  return true;
}

void determineMode()
{
  if(ble.buffer[0]=='m')
  {
    if((ble.buffer[1]-'0')<4)
    {
      mode=(ble.buffer[1]-'0');
    }
    else
    {
      mode=0;
    }
  }
  else if(ble.buffer[0]=='t' || ble.buffer[0]=='s')
  {
    timeLapseDelay=0;
    timeLapseDelay+=ble.buffer[4]-'0';
    timeLapseDelay+=(ble.buffer[3]-'0')*10;
    timeLapseDelay+=(ble.buffer[2]-'0')*100;
    timeLapseDelay+=(ble.buffer[1]-'0')*1000;
    if (ble.buffer[0]=='s')
    {
      timeLapseDelay*=1000;
    }
    Serial.print("Time lapse delay: ");
    Serial.println(timeLapseDelay);
  }
  else if(ble.buffer[0]=='v')
  {
    if (mode==3)
    {
      digitalWrite(CAMERAPIN, LOW);
      delay(600);
      digitalWrite(CAMERAPIN, HIGH);
    }
  }
  else if(ble.buffer[0]=='p')
  {
    digitalWrite(CAMERAPIN, LOW);
    delay(100);
    digitalWrite(CAMERAPIN, HIGH);
    Serial.println("PICTURE!");
  }
  else
  {
    mode=0;
  }
  Serial.print("Mode: ");
  Serial.println(mode);

  return;
}

void handleCamera()
{
  //Serial.println("HANDLECAMERA");
  //delay(1000);
  //0==idle
  //1==triggered
  //2==time laspe
  //3==video
  
  switch (mode)  
  {
    case 0:
    Serial.println("MODE 0");
    LowPower.idle(SLEEP_250MS, ADC_OFF, TIMER4_OFF, TIMER3_OFF, TIMER1_OFF, TIMER0_OFF, SPI_ON, USART1_OFF, TWI_OFF, USB_ON);
    break;
    
    case 1:
    triggerPinStatus=digitalRead(TRIGGERPIN);
    if(triggerPinStatus==0&&previousTriggerPinStatus==1)
    {
     digitalWrite(CAMERAPIN, LOW);
     //LowPower.idle(SLEEP_30MS, ADC_OFF, TIMER4_OFF, TIMER3_OFF, TIMER1_OFF, TIMER0_OFF, SPI_ON, USART1_OFF, TWI_OFF, USB_ON);
     delay(100);
     digitalWrite(CAMERAPIN, HIGH);
     Serial.println("TRIGGERED PICTURE");
    }
    previousTriggerPinStatus=triggerPinStatus;
    LowPower.idle(SLEEP_250MS, ADC_OFF, TIMER4_OFF, TIMER3_OFF, TIMER1_OFF, TIMER0_OFF, SPI_ON, USART1_OFF, TWI_OFF, USB_ON);
    break;
    
    case 2:
    sleptTime=0;
    
    while ((timeLapseDelay - sleptTime)>8000)
    {
      Serial.println("8000");
      LowPower.idle(SLEEP_8S, ADC_OFF, TIMER4_OFF, TIMER3_OFF, TIMER1_OFF, TIMER0_OFF, SPI_ON, USART1_OFF, TWI_OFF, USB_OFF);
      sleptTime+=8000;
    }
    while ((timeLapseDelay - sleptTime)>4000)
    {
      Serial.println("4000");
      LowPower.idle(SLEEP_4S, ADC_OFF, TIMER4_OFF, TIMER3_OFF, TIMER1_OFF, TIMER0_OFF, SPI_ON, USART1_OFF, TWI_OFF, USB_OFF);
      sleptTime+=4000;
    }
    while ((timeLapseDelay - sleptTime)>2000)
    {
      Serial.println("2000");
      LowPower.idle(SLEEP_2S, ADC_OFF, TIMER4_OFF, TIMER3_OFF, TIMER1_OFF, TIMER0_OFF, SPI_ON, USART1_OFF, TWI_OFF, USB_OFF);
      sleptTime+=2000;
    }
    while ((timeLapseDelay - sleptTime)>1000)
    {
      Serial.println("1000");
      LowPower.idle(SLEEP_1S, ADC_OFF, TIMER4_OFF, TIMER3_OFF, TIMER1_OFF, TIMER0_OFF, SPI_ON, USART1_OFF, TWI_OFF, USB_OFF);
      sleptTime+=1000;
    }
    while ((timeLapseDelay - sleptTime)>500)
    {
      Serial.println("500");
      LowPower.idle(SLEEP_500MS, ADC_OFF, TIMER4_OFF, TIMER3_OFF, TIMER1_OFF, TIMER0_OFF, SPI_ON, USART1_OFF, TWI_OFF, USB_OFF);
      sleptTime+=500;
    }
    while ((timeLapseDelay - sleptTime)>250)
    {
      Serial.println("250");
      LowPower.idle(SLEEP_250MS, ADC_OFF, TIMER4_OFF, TIMER3_OFF, TIMER1_OFF, TIMER0_OFF, SPI_ON, USART1_OFF, TWI_OFF, USB_OFF);
      sleptTime+=250;
    }
    while ((timeLapseDelay - sleptTime)>120)
    {
      Serial.println("120");
      LowPower.idle(SLEEP_120MS, ADC_OFF, TIMER4_OFF, TIMER3_OFF, TIMER1_OFF, TIMER0_OFF, SPI_ON, USART1_OFF, TWI_OFF, USB_OFF);
      sleptTime+=120;
    }
    while ((timeLapseDelay - sleptTime)>60)
    {
      Serial.println("60");
      LowPower.idle(SLEEP_60MS, ADC_OFF, TIMER4_OFF, TIMER3_OFF, TIMER1_OFF, TIMER0_OFF, SPI_ON, USART1_OFF, TWI_OFF, USB_OFF);
      sleptTime+=60;
    }
    while ((timeLapseDelay - sleptTime)>30)
    {
      Serial.println("30");
      LowPower.idle(SLEEP_30MS, ADC_OFF, TIMER4_OFF, TIMER3_OFF, TIMER1_OFF, TIMER0_OFF, SPI_ON, USART1_OFF, TWI_OFF, USB_OFF);
      sleptTime+=30;
    }
    while ((timeLapseDelay > sleptTime))
    {
      Serial.println("15");
      LowPower.idle(SLEEP_15MS, ADC_OFF, TIMER4_OFF, TIMER3_OFF, TIMER1_OFF, TIMER0_OFF, SPI_ON, USART1_OFF, TWI_OFF, USB_OFF);
      sleptTime+=15;
    }
    
    Serial.print("I slept: ");
    Serial.print(sleptTime);
    Serial.println(" ms");
    digitalWrite(CAMERAPIN, LOW);
    //LowPower.idle(SLEEP_30MS, ADC_OFF, TIMER4_OFF, TIMER3_OFF, TIMER1_OFF, TIMER0_OFF, SPI_ON, USART1_OFF, TWI_OFF, USB_OFF);
    delay(100);
    digitalWrite(CAMERAPIN, HIGH);
    
    Serial.println("Took a picture!");
    
    break;
    
    case 3:
    LowPower.idle(SLEEP_250MS, ADC_OFF, TIMER4_OFF, TIMER3_OFF, TIMER1_OFF, TIMER0_OFF, SPI_ON, USART1_OFF, TWI_OFF, USB_ON);
    Serial.println("CASE 3");
    break;
  }

  return;
}

