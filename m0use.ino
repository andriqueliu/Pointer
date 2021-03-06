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

// For BNO055
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define Serial SERIAL_PORT_USBVIRTUAL
/*=========================================================================
    APPLICATION SETTINGS

      FACTORYRESET_ENABLE     Perform a factory reset when running this sketch
     
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
    -----------------------------------------------------------------------*/
#define FACTORYRESET_ENABLE      1
/*=========================================================================*/

// Define constraints for operating modes
#define GESTURE_MODE (!gesture_button)
#define LEFT_CLICK (!digitalRead(13))


// Create IMU object
Adafruit_BNO055 bno = Adafruit_BNO055();

#define BNO055_SAMPLERATE_DELAY_MS (10)
#define MOVETHRESHOLD (3)
#define MAXMOVE (150)

// Define enum capturing possible gestures
typedef enum {
    GESTURE_START,
    GESTURE_RIGHT,
    GESTURE_LEFT,
    GESTURE_DOUBLE_SWIPE
} gesture_state;

// Create the bluefruit object, either software serial...uncomment these lines
/*
  SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

  Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
*/

/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
// Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

int count;

/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void)
{
  // BNO055 reset GPIOs
  pinMode(9,INPUT);
  pinMode(5,OUTPUT);
  digitalWrite(5,HIGH);
  pinMode(13, INPUT);
  
  pinMode(12, INPUT);
  
  // Left click button
  pinMode(11, INPUT);
  pinMode(10, INPUT);
  count = 0;
  // Uncomment this when ready to use
  // Gesture Mode button input
//    pinMode(INSERTPINHERE, INPUT);
  
  // Uncomment this when ready to use
  // Mouse click button inputs
//    pinMode(SOMELEFTCLICKPIN, INPUT);
//    pinMode(SOMERIGHTCLICKPIN, INPUT);
  Serial.begin(9600); 
  if(!bno.begin())
  {
      /* There was a problem detecting the BNO055 ... check your connections */     
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
      while(1);
  }
  
  delay(500);


  Serial.println(F("Adafruit Bluefruit LE"));
  Serial.println(F("-------------------------------------"));

  pinMode(6, INPUT);
  pinMode(5, OUTPUT);

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
    if ( ! ble.factoryReset() ) {
      error(F("Couldn't factory reset"));
    }
  }

  /* Enable HID Service */
  Serial.println(F("Enable HID Service (including Keyboard): "));

  if ( !ble.sendCommandCheckOK(F( "AT+BleHIDEn=On" ))) {
    error(F("Could not enable Keyboard"));
  }

  /* Add or remove service requires a reset */
  Serial.println(F("Performing a SW reset (service changes require a reset): "));
  if (! ble.reset() ) {
    error(F("Couldn't reset??"));
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

//  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();
}

// Reference: how the mbed schedules tasks
/*
while (1) {
    wait(0.01);
    
    // 
    imu.read_euler();
    if (!GESTURE_MODE) {
        process_click();
//            process_dpi();
        process_move();
    } else {
        process_gesture();
    }
    process_reset();
}
*/

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
  
  // Reset BNO055 chip
  if (digitalRead(9) == LOW) {
    /*
    ble.println("AT+BLEHIDMOUSEMOVE=1,0");
    ble.waitForOK();
    */
    digitalWrite(5, LOW);
    digitalWrite(5, HIGH);
    bno.begin();  
  }

  //process_click();
  //process_move();
  process_gesture();
  
  //delay(BNO055_SAMPLERATE_DELAY_MS);
}

/*
 * process_move determines the direction and magnitude of mouse movement.
 * 
 * Inputs:
 * None
 * 
 * Returns:
 * None
 */
void process_move(void)
{    
  int16_t move_x, move_y;
  
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  move_x = euler.x();
  move_y = euler.y();
  move_x = normalize(move_x); 
  
  // Control x-axis using heading
  if (move_x <= MOVETHRESHOLD && move_x >= -MOVETHRESHOLD) {
      move_x = 0;
  } else if (move_x > MOVETHRESHOLD) {
      if (move_x > MAXMOVE+MOVETHRESHOLD) {
          move_x = MAXMOVE;
      } else {
          move_x -= MOVETHRESHOLD;
      }
  } else {
      if (move_x < -MAXMOVE-MOVETHRESHOLD) { // !!! CONFIRM: this means -(MAXMOVE - MOVETHRESHOLD)???
          move_x = -MAXMOVE;
      } else {
          move_x += MOVETHRESHOLD;
      }
  }

  // Control y-axis movement using pitch
  if (move_y <= MOVETHRESHOLD && move_y  >= -MOVETHRESHOLD) {
    move_y = 0;
  } else if (move_y > MOVETHRESHOLD) {
    if (move_y > MAXMOVE+MOVETHRESHOLD) {
        move_y = MAXMOVE;
    } else {
        move_y -=MOVETHRESHOLD;
    }
  } else {
    if (move_y < -MAXMOVE-MOVETHRESHOLD) {
        move_y = -MAXMOVE;
    } else {
        move_y+=MOVETHRESHOLD;
    }
  }
  
  // Transmit AT command for mouse movement
  String base = "AT+BLEHIDMOUSEMOVE=";
  String x = String(move_x);
  String separate = ",";
  String y = String(move_y);
  
  base = base + x + separate + y;
  
  ble.println(base);
  ble.waitForOK();
}

/*
 * 
 */
int16_t normalize(int16_t value)
{
    if (value >= 0 && value <= 179) {
        // Do nothing
    } else {
        value = -(360 - value);
    }
    
    return value;
}

/*
 * 
 */
void process_click(void)
{
//    static bool preRightClick = false;
  
  String base = "AT+BLEHIDMOUSEBUTTON=";
  
  if (LEFT_CLICK) {
//        mouse.press(MOUSE_LEFT);
    base = base + "L";
  } else {
//        mouse.release(MOUSE_LEFT);
    base = base + "0";
  }
  
  // This approach might not work. What might need to be done is to assume that 
  // the M0 will maintain the last sent mouse command, and you need to keep track
  // of the PREVIOUS command, only sending a command when the curr. != prev.
  ble.println(base);
  ble.waitForOK();
  
  // Test whether left mouse click works first
//    // Right Mouse Click  ___  Falling Edge Detection
//    if (right_click == 0 && preRightClick == false) {  
//        preRightClick = true;
//    } else if (right_click == 1 && preRightClick == true) {
//        preRightClick = false;
//        mouse.click(MOUSE_RIGHT);
//    }
}

/*
 * 
 */
 
void process_gesture(void)
{
    gesture_state curr_gesture_state = GESTURE_START;
    
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    int16_t move_x_initial = euler.x();
    move_x_initial = normalize(move_x_initial);
    
    while (digitalRead(13) == LOW) {
        Serial.println("PROCESSING GESTURE. IN LOOP (BUTTON HIGH)");
        delay(10); // Wait 10 ms !!! Experiment with this pls. Worked for mbed but won't necessarily
                   // work for the M0.
        imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
        int16_t move_x = euler.x();
        move_x = normalize(move_x);
        
        switch (curr_gesture_state) {
        case GESTURE_START:
            if (move_x > (move_x_initial + MOVETHRESHOLD)) { // past right threshold
                curr_gesture_state = GESTURE_RIGHT;
//            } else if (move_x < -MAXMOVE-MOVETHRESHOLD) { // past left threshold
            } else if (move_x < (move_x_initial - 3)) {
                curr_gesture_state = GESTURE_LEFT;
            }
            break;
        case GESTURE_LEFT:
            if (move_x > (move_x_initial + MOVETHRESHOLD)) { // past right threshold
                curr_gesture_state = GESTURE_DOUBLE_SWIPE;
            }
            break;
        case GESTURE_RIGHT:
//            if (move_x < -MAXMOVE-MOVETHRESHOLD) { // past left threshold
            if (move_x < (move_x_initial - 3)) {
                curr_gesture_state = GESTURE_DOUBLE_SWIPE;
            }
        case GESTURE_DOUBLE_SWIPE:
            break;
        default:
            break;
        }
    }
    
    // Once the user lets go, begin parsing through the sampled Euler data
    // and then determine what gesture was done
    // Jk, see what state it is and act on it...
    if (curr_gesture_state == GESTURE_START) {
//        left = 0;
//        right = 0;
    } else if (curr_gesture_state == GESTURE_LEFT) {
//        left = 1;
//        right = 0;
    } else if (curr_gesture_state == GESTURE_RIGHT) {
//        left = 0;
//        right = 1;
    } else if (curr_gesture_state == GESTURE_DOUBLE_SWIPE) {
        // Send over the keystroke
        tx_keystroke(' ');
        //Serial.println("double swipe worked!");
    }
}

// !!! Assume this only has to support SPACE for now
/*
 * 
 */
void tx_keystroke(char key)
{
  String base = "AT+BLEKEYBOARD=";
  
//    switch (key) {
//        
//    }
  base = base + key;
  
  ble.println(base);
  ble.waitForOK();
}

