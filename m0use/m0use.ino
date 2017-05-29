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

<<<<<<< HEAD
// To enable Serial printing, set DEBUG_SERIAL_BEGIN 1, and uncomment DEBUG
// Vice-versa to disable Serial printing
#define DEBUG_SERIAL_BEGIN 1
#define DEBUG
=======
// Define constraints for operating modes
#define GESTURE_MODE (!gesture_button)
#define LEFT_CLICK (!digitalRead(13))
>>>>>>> 56b58ce729eba7624b55e4adb12f1d96e2aa7995


<<<<<<< HEAD
// Define constraints for operating modes
#define GESTURE_MODE (!digitalRead(13))
#define LEFT_CLICK (!digitalRead(11))       

// 
#define BNO055_SAMPLERATE_DELAY_MS (100)
=======
// Create IMU object
Adafruit_BNO055 bno = Adafruit_BNO055();

#define BNO055_SAMPLERATE_DELAY_MS (10)
>>>>>>> 56b58ce729eba7624b55e4adb12f1d96e2aa7995
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

<<<<<<< HEAD
// Global Variables:

double constant_a;
double constant_b;

=======
int count;
>>>>>>> 56b58ce729eba7624b55e4adb12f1d96e2aa7995

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
<<<<<<< HEAD
  
  // Left click button
  pinMode(11, INPUT);
  // Right click button
//  pinMode(something, INPUT);
  // Gesture Mode button
  pinMode(13, INPUT);

  // Declare initial values of constants A and B
  constant_a = 3;
  constant_b = 7;

  // Hang until connection with BNO055 has been established
=======
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
>>>>>>> 56b58ce729eba7624b55e4adb12f1d96e2aa7995
  if(!bno.begin())
  {
      /* There was a problem detecting the BNO055 ... check your connections */     
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
      while(1);
  }
  
  delay(500);


  Serial.println(F("Adafruit Bluefruit LE"));
  Serial.println(F("-------------------------------------"));

  /*
  pinMode(6, INPUT);
  pinMode(5, OUTPUT);
  */
  
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
<<<<<<< HEAD
  // Even a 10 ms delay makes the mouse unusable
  //delay(BNO055_SAMPLERATE_DELAY_MS);
  if (!GESTURE_MODE) {
    process_click();
//    process_sensitivity();
    process_move();
  } else {
    process_gesture();
  }
  
  process_reset();
}

/*
 * 
 */
void process_reset(void)
{
  if (digitalRead(9) == LOW) {
    // Toggle BNO055's reset input
    digitalWrite(5, LOW);
    digitalWrite(5, HIGH);

    // Run the BNO055's initialization sequence
=======
  
  // Reset BNO055 chip
  if (digitalRead(9) == LOW) {
    /*
    ble.println("AT+BLEHIDMOUSEMOVE=1,0");
    ble.waitForOK();
    */
    digitalWrite(5, LOW);
    digitalWrite(5, HIGH);
>>>>>>> 56b58ce729eba7624b55e4adb12f1d96e2aa7995
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
  double move_x;
  int16_t move_y;

  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  move_x = euler.x();
  move_y = euler.y();
<<<<<<< HEAD
  move_x = normalize(move_x);

//  // 
//  if (-MOVETHRESHOLD <= move_x && move_x <= MOVETHRESHOLD) {
//    move_x = 0;
//  }
//  if (-MOVETHRESHOLD <= move_y && move_y <= MOVETHRESHOLD) {
//    move_y = 0;
//  }

  // 
  move_x = process_move_x(move_x);
  move_y = process_move_y(move_y);


=======
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
>>>>>>> 56b58ce729eba7624b55e4adb12f1d96e2aa7995
  
  // Transmit AT command for mouse movement
  String base = "AT+BLEHIDMOUSEMOVE=";

  // Look at value before to string
  // Confirmed: to string not ruining anything
//  SERIAL_PRINT(move_x);
//  SERIAL_PRINT(" ");
//  SERIAL_PRINT(move_y);
//  SERIAL_PRINT('\n');

  // Might have to cast back...
  int new_move_x = floor(move_x);
  
  String x = String(new_move_x);
  String separate = ",";
  String y = String(move_y);
  
  base = base + x + separate + y;
  
  ble.println(base);
  ble.waitForOK();
}

/*
 * 
 */
int16_t normalize(double value)
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
<<<<<<< HEAD
double process_move_x(double current_move)
{
  static double prev_move_x = current_move;
  double current_diff = current_move - prev_move_x;
  double invert = 1;
  Serial.print("current_move: ");
  Serial.println(current_move);

  Serial.print("prev_move_x: ");
  Serial.println(prev_move_x);

  Serial.print("current_diff: ");
  Serial.println(current_diff);
  


  
  prev_move_x = current_move;

  // Replace constant mult. with shifting! Try that later... First, try ADDING as opposed to multiplying...
//  return (current_diff * CONSTANT_A) * ((current_diff * current_diff) * CONSTANT_B);
  if (current_diff < 0) {
    invert = -1;
  } else {
    invert = 1;
  }

  // Maybe left shift by the constants instead of multiply??? Might not be fine enough...
//  return (current_diff * constant_a) + ((current_diff * current_diff) * constant_b * invert);

//  if (current_diff == 4) {
//    SERIAL_PRINT("THE DIFF IS 4\n");
//    SERIAL_PRINT(current_move);
//    SERIAL_PRINT(" ");
//    SERIAL_PRINT(prev_move_x);
//    SERIAL_PRINT('\n');
//  }

  double final;
  
  if (current_diff >= 0) {
    final = (current_diff) + ((current_diff * current_diff) * constant_b);
//    final = (current_diff) + constant_b;
  } else {
    final = (current_diff) - ((current_diff * current_diff) * constant_b);
//    final = (current_diff) - constant_b;
  }

//  if (final == 52 || final == -52) {
//    SERIAL_PRINT(current_move);
//    SERIAL_PRINT(" ");
//    SERIAL_PRINT(prev_move_x);
//    SERIAL_PRINT('\n');
//  }

  return final;
}

/*
 * 
 */
int16_t process_move_y(int16_t current_move)
{
//  static int16_t prev_move_y = current_move;
//  int16_t current_diff = current_move - prev_move_y;
//
//  prev_move_y = current_move;
//
////  return (current_diff * CONSTANT_A) * ((current_diff * current_diff) * CONSTANT_B);
//  return (current_diff * CONSTANT_A) + ((current_diff * current_diff) * CONSTANT_B);

  static int16_t prev_move_y = current_move;
  int16_t current_diff = current_move - prev_move_y;
  int16_t invert = 1;
  
  prev_move_y = current_move;

  if (current_diff < 0) {
    invert = -1;
  } else {
    invert = 1;
  }

//  if (current_diff == 4) {
//    SERIAL_PRINT("THE DIFF IS 4!!!!!!!\n");
//  }
  
  return (current_diff * constant_a) + ((current_diff * current_diff) * constant_b * invert);
//  if (current_diff < 0) {
//    return (current_diff * constant_a) - ((current_diff * current_diff) * constant_b);
//  } else {
//    return (current_diff * constant_a) + ((current_diff * current_diff) * constant_b);
//  }
}

/*
 * Process the state of the mouse buttons and transmit mouse clicks, if
 * appropriate.
 * 
 * This function supports left and right mouse clicks.
 */
=======
>>>>>>> 56b58ce729eba7624b55e4adb12f1d96e2aa7995
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

