/*
 * 
 */
 
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

// To enable Serial printing, set DEBUG_SERIAL_BEGIN 1, and uncomment DEBUG
// Vice-versa to disable Serial printing
#define DEBUG_SERIAL_BEGIN 0
//#define DEBUG

// Macros to enable Serial printing based on debug macros:
#ifdef DEBUG
 #define SERIAL_PRINT(x)  Serial.print(x)
#else
 #define SERIAL_PRINT(x)
#endif
#ifdef DEBUG
 #define SERIAL_PRINT_F(x)  Serial.print(F(x))
#else
 #define SERIAL_PRINT_F(x)
#endif

// Define constraints for operating modes
#define GESTURE_MODE (!digitalRead(13))
#define LEFT_CLICK (!digitalRead(11))
#define RESET (!digitalRead(9))

// 
#define BNO055_SAMPLERATE_DELAY_MS (100)
#define MOVETHRESHOLD (3)
#define MAXMOVE (100)

// Constants used to influence mouse movement
//#define CONSTANT_A 2
//#define CONSTANT_B 2

// Define enum capturing possible gestures
typedef enum {
    GESTURE_START,
    GESTURE_RIGHT,
    GESTURE_LEFT,
    GESTURE_DOUBLE_SWIPE,
    GESTURE_ROLL_RIGHT,
    GESTURE_ROLL_LEFT
} gesture_state;

// Create IMU object
Adafruit_BNO055 bno = Adafruit_BNO055();

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

// A small helper
void error(const __FlashStringHelper*err) {
  SERIAL_PRINT(err);
  SERIAL_PRINT("\n");
  while (1);
}

// Global Variables:

int16_t constant_a;
int16_t constant_b;


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
  
  // Left click button
  pinMode(11, INPUT);
  // Right click button
//  pinMode(something, INPUT);
  // Gesture Mode button
  pinMode(13, INPUT);

  // Declare initial values of constants A and B
  constant_a = 2;
  constant_b = 6;

  // Hang until connection with BNO055 has been established
  if(!bno.begin())
  {
      /* There was a problem detecting the BNO055 ... check your connections */
      SERIAL_PRINT("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
      while(1);
  }

  delay(500);

  // Below doesn't work with just DEBUG alone, need DEBUG 1... will that work???
  if (DEBUG_SERIAL_BEGIN) {
    Serial.begin(9600);
  }

  SERIAL_PRINT_F("Adafruit Bluefruit LE\n");
  SERIAL_PRINT_F("-------------------------------------\n");

  /*
  pinMode(6, INPUT);
  pinMode(5, OUTPUT);
  */
  
  /* Initialise the module */
  SERIAL_PRINT_F("Initialising the Bluefruit LE module: ");

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  SERIAL_PRINT_F("OK!\n");
  
  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    SERIAL_PRINT_F("Performing a factory reset: \n");
    if ( ! ble.factoryReset() ) {
      error(F("Couldn't factory reset"));
    }
  }

  /* Enable HID Service */
  SERIAL_PRINT_F("Enable HID Service (including Keyboard): \n");

  if ( !ble.sendCommandCheckOK(F( "AT+BleHIDEn=On" ))) {
    error(F("Could not enable Keyboard"));
  }

  /* Add or remove service requires a reset */
  SERIAL_PRINT_F("Performing a SW reset (service changes require a reset): \n");
  if (! ble.reset() ) {
    error(F("Couldn't reset??"));
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  SERIAL_PRINT("Requesting Bluefruit info:\n");
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
  // Even a 10 ms delay makes the mouse unusable
  //delay(BNO055_SAMPLERATE_DELAY_MS);
  if (!GESTURE_MODE) {
    process_click();
    process_move();
  } else {
    process_gesture();
  }
  
  process_reset();
}

// !!! We can replace this later with a right click...
/*
 * Checks whether the reset button is pressed; if pressed, this function resets
 * the BNO055.
 */
void process_reset(void)
{
  if (RESET) {
    // Toggle BNO055's reset input
    digitalWrite(5, LOW);
    digitalWrite(5, HIGH);

    // Run the BNO055's initialization sequence
    bno.begin();  
  }
}

/*
 * Determine the direction and magnitude of mouse movement.
 */
void process_move(void)
{
  int16_t move_x;
  int16_t move_y;

  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  move_x = euler.x();
  move_y = euler.y();
  move_x = normalize(move_x);

  // Determine 
  move_x = process_move_x(move_x);
  // MAY NEED TO NEGATE HERE
  move_y = process_move_y(move_y);
  
  // Transmit AT command for mouse movement
  String base = "AT+BLEHIDMOUSEMOVE=";

  // Might have to cast back...
//  int new_move_x = floor(move_x);
  
  String x = String(move_x);
  String separate = ",";
  String y = String(move_y);
  
  base = base + x + separate + y;
  
  ble.println(base);
  ble.waitForOK();
}

/*
 * Normalize a value defined by the default heading convention.
 * 
 * Heading is defined from 0 to 360 degrees by default; this function redefines
 * heading to be 0 to 180 and 0 to -180 relative to the initial position.
 * 
 * @param value value to be normalized
 * @return normalized value
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
int16_t process_move_x(int16_t current_move)
{
  static int16_t prev_move_x = current_move;
  int16_t current_diff = current_move - prev_move_x;
//  Serial.print("current_move: ");
//  Serial.println(current_move);
//
//  Serial.print("prev_move_x: ");
//  Serial.println(prev_move_x);
//
//  Serial.print("current_diff: ");
//  Serial.println(current_diff);
  
  prev_move_x = current_move;

  int16_t final;
  
  if (current_diff >= 0) {
    final = (current_diff) + ((current_diff * current_diff) * constant_b);
  } else {
    final = (current_diff) - ((current_diff * current_diff) * constant_b);
  }

  return final;
}

/*
 * 
 */
int16_t process_move_y(int16_t current_move)
{
  static int16_t prev_move_y = current_move;
  int16_t current_diff = current_move - prev_move_y;
  
  prev_move_y = current_move;
  
  int16_t final;
  
  if (current_diff >= 0) {
    final = (current_diff) + ((current_diff * current_diff) * constant_b);
  } else {
    final = (current_diff) - ((current_diff * current_diff) * constant_b);
  }

  return final;
}

/*
 * Process the state of the mouse buttons and transmit mouse clicks, if
 * appropriate.
 * 
 * This function supports left and right mouse clicks.
 */
void process_click(void)
{
//    static bool preRightClick = false;
  String base = "AT+BLEHIDMOUSEBUTTON=";
  
  if (LEFT_CLICK) {
    base = base + "L";
  } else {
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
 * Initiate Gesture Mode. Then, track the user's movements and determine the
 * appropriate gesture.
 * 
 * This function only supports the gestures described in the gesture_state enum.
 */
void process_gesture(void)
{
    // Declare initial gesture
    gesture_state curr_gesture_state = GESTURE_START;
    
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    int16_t move_x_initial = euler.x();
    move_x_initial = normalize(move_x_initial);
    int16_t move_z_initial = euler.z();
    
    while (GESTURE_MODE) {
        delay(10); // Wait 10 ms !!! Experiment with this pls. Worked for mbed but won't necessarily
                   // work for the M0.
        imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
        int16_t move_x = euler.x();
        move_x = normalize(move_x);
        int16_t move_z = euler.z();
        
        //         
        switch (curr_gesture_state) {
        case GESTURE_START:
            if (move_x > (move_x_initial + MOVETHRESHOLD)) { // past right threshold
                curr_gesture_state = GESTURE_RIGHT;
            } else if (move_x < (move_x_initial - 3)) {
                curr_gesture_state = GESTURE_LEFT;
            } else if (move_z > (move_z_initial + MOVETHRESHOLD)) {
              curr_gesture_state = GESTURE_ROLL_RIGHT;
            } else if (move_z < (move_z_initial - MOVETHRESHOLD)) {
              curr_gesture_state = GESTURE_ROLL_LEFT;
            }
            // ^^^ Have heading change take priority over roll change!
            break;
        case GESTURE_LEFT:
            if (move_x > (move_x_initial + MOVETHRESHOLD)) { // past right threshold
                curr_gesture_state = GESTURE_DOUBLE_SWIPE;
            }
            break;
        case GESTURE_RIGHT:
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
    } else if (curr_gesture_state = GESTURE_ROLL_RIGHT) {
      constant_a++;
      constant_b++;
    } else if (curr_gesture_state = GESTURE_ROLL_LEFT) {
      if (constant_a > 1 && constant_b > 1) {
        constant_a--;
        constant_b--;
      }
    }
}

// !!! Assume this only has to support SPACE for now
/*
 * Transmit the character specified by key.
 * 
 * @param key the key to be transmitted
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

///*
// * Transmit a keystroke combo: a key along with a modifier
// * 
// * How to use:
// * 
// * @param key
// * @param option
// */
//void tx_key_combo(char key, char option)
//{
//  
//}

