/**
 * @file BambuNRS-V2_3-ENG.ino
 * @brief Bambu Lab Nozzle Repair Station - ESP32 PID Temperature Controller
 * @version 2.3
 * @date 2026-04-26
 *
 * @mainpage Bambu Nozzle Repair Station (BambuNRS)
 *
 * A DIY nozzle troubleshooting and repair station for Bambu Lab 3D printers.
 * Features dual-nozzle support (A/H2S/H2D/P2-series and P1/X1E-series), PID temperature
 * control with ADS1115 ADC, OLED display, and Bluetooth configuration.
 *
 * @author Feng Xue <xuefeng@printxdreams.com>
 * @license GPLv3
 *
 * @par Hardware Configuration:
 * - MCU: ESP32 Dev Module
 * - ADC: ADS1115 (16-bit, I2C)
 * - Display: SSD1306 OLED 128x64 (I2C)
 * - Encoder: Rotary encoder with push button
 *
 * @par Supported Printers:
 * - Bambu Lab A-series (24V system)
 * - Bambu Lab P-series (24V system)
 * - Bambu Lab X-series (24V system)
 *
 * @par Bluetooth Commands:
 * - aXXX : Set initial temperature (0-255)
 * - bXXX : Set minimum temperature (0-255)
 * - cXXX : Set maximum temperature (100-355)
 * - eXXX : Set encoder increment (0-255)
 * - pXXX : Set PID P value (0-255)
 * - iXXX : Set PID I value (0-255)
 * - dXXX : Set PID D value (0-255)
 * - fXXX : Set temperature method (0=calc, 1=table)
 * - r0   : Reset all EEPROM values
 */

//==============================================================================
// Header Files
//==============================================================================

#include <PID_v1.h>              // PID controller library by Brett Beauregard
#include <Wire.h>                // I2C communication
#include <Adafruit_SSD1306.h>    // OLED display driver
#include <Arduino.h>             // Arduino core
#include <EEPROM.h>              // Flash memory read/write
#include "BluetoothSerial.h"     // Bluetooth serial communication
#include <Adafruit_ADS1X15.h>    // ADS1115 ADC driver

// Verify Bluetooth configuration
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please enable Bluetooth in menuconfig
#endif

//==============================================================================
// Compile-Time Configuration
//==============================================================================

/**
 * @name ADC Oversampling
 * @brief Oversampling ratio for ADC readings
 *
 * The ADS1115 provides 16-bit resolution. Oversampling can improve
 * stability but reduces response speed.
 */
#define OVERSAMPLENR 1

/**
 * @name Display Configuration
 * @brief SSD1306 OLED display parameters
 */
#define SCREEN_WIDTH 128     ///< OLED display width in pixels
#define SCREEN_HEIGHT 64      ///< OLED display height in pixels
#define OLED_RESET     -1     ///< Reset pin sharing Arduino reset pin
#define SCREEN_ADDRESS 0x3C   ///< OLED I2C address (0x3C typical)

/**
 * @name Temperature Limits
 * @brief Default operating temperature range (in Celsius)
 */
#define INITIAL_TEMP 210  ///< Default target temperature
#define MIN_TEMP     50   ///< Minimum allowed temperature
#define MAX_TEMP     300  ///< Maximum allowed temperature

/**
 * @name EEPROM Storage Layout
 * @brief Flash memory addresses for persistent parameters
 * @note Address 0 is reserved for the initialization flag
 */
#define EEPROM_SIZE 10                      ///< Total EEPROM size allocated
#define ADDR_EPR_FLAG         0  ///< Flag: 1 = parameters initialized
#define ADDR_INITIAL_TEMP     1  ///< Initial target temperature
#define ADDR_MIN_TEMP         2  ///< Minimum temperature limit
#define ADDR_MAX_TEMP         3  ///< Maximum temperature limit
#define ADDR_ENC_INCREMENT    4  ///< Encoder step size
#define ADDR_TEMP_METHOD      5  ///< Temperature calculation method
#define ADDR_PID_P            6  ///< PID proportional gain
#define ADDR_PID_I            7  ///< PID integral gain
#define ADDR_PID_D            8  ///< PID derivative gain
#define ADDR_NOZZLE_ACTIVE    9  ///< Active nozzle selection (1=L, 0=R)

//==============================================================================
// Global Objects
//==============================================================================

BluetoothSerial SerialBT;   ///< Bluetooth serial for mobile configuration
Adafruit_ADS1115 ads;       ///< ADS1115 16-bit ADC instance
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//==============================================================================
// PID Controller Variables
//==============================================================================

double Kp = 60.0;           ///< PID proportional gain (default for P-series)
double Ki = 10.0;           ///< PID integral gain
double Kd = 5.0;            ///< PID derivative gain
double setpoint = INITIAL_TEMP;  ///< Target temperature
double input = 25.0;        ///< Current temperature input
double output = 0;          ///< PID output (PWM duty cycle)

/**
 * @brief PID controller instance
 * @note Uses DIRECT mode: output increases when error is positive
 */
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

//==============================================================================
// Encoder Configuration
//==============================================================================

static const int PIN_ENCODER_A = 18;   ///< Encoder phase A (interrupt pin)
static const int PIN_ENCODER_B = 19;   ///< Encoder phase B (interrupt pin)
static const int PIN_ENCODER_BTN = 5;   ///< Encoder push button

volatile int encoderPos = INITIAL_TEMP;       ///< Current encoder position
volatile int prevEncoderPos = INITIAL_TEMP;  ///< Previous position for direction detection

/**
 * @brief Encoder limits and step size
 */
int encLowLim = MIN_TEMP;      ///< Minimum value
int encHighLim = MAX_TEMP;    ///< Maximum value
int encIncrement = 5;         ///< Step size per detent
int encCurrent = INITIAL_TEMP; ///< Current target temperature

//==============================================================================
// Button & Debounce Configuration
//==============================================================================

const unsigned long DEBOUNCE_TIME = 10;           ///< Debounce delay (ms)
const unsigned long NOZZLE_CHANGE_TIME = 1500;    ///< Long-press duration for nozzle switch (ms)

byte oldButtonState = HIGH;          ///< Previous button state (pulled HIGH)
byte oldFanButtonState = HIGH;       ///< Previous fan button state
unsigned long buttonPressTime;       ///< Timestamp of last button press
unsigned long fanButtonPressTime;   ///< Timestamp of last fan button press
unsigned long button1stPressTime;    ///< Timestamp of initial press (for long-press detection)

boolean pressed = false;             ///< Button currently pressed flag
boolean fanpressed = false;          ///< Fan button currently pressed flag
boolean fanActive = true;           ///< Cooling fan mode (true=AUTO, false=OFF)

//==============================================================================
// Nozzle Selection
//==============================================================================

boolean nozzleLActive = true;   ///< true = Left/A-series nozzle active, false = Right/P-series
int preHeatStatus = 0;           ///< 0 = heating off, 1 = heating on

//==============================================================================
// Hardware Pin Assignments
//==============================================================================

const int PIN_BACK_BUTTON = 23;   ///< Back panel button (fan toggle)
const int PIN_FAN_L = 32;         ///< Left fan control (A-series)
const int PIN_FAN_R = 33;         ///< Right fan control (P/X-series)
const int PIN_PWM_L = 25;         ///< Left PWM output (A-series heater)
const int PIN_PWM_R = 26;         ///< Right PWM output (P/X-series heater)

//==============================================================================
// PWM Configuration (ESP32 LEDC)
//==============================================================================

const int PWM_CHANNEL_L = 5;      ///< ESP32 LEDC channel for left heater
const int PWM_CHANNEL_R = 7;      ///< ESP32 LEDC channel for right heater
const int PWM_FREQ = 500;         ///< PWM frequency (Hz) - similar to Arduino ~490Hz
const int PWM_RESOLUTION = 8;     ///< 8-bit resolution (0-255), compatible with Arduino

//==============================================================================
// ADC & Temperature Reading
//==============================================================================

int16_t adc0, adc1;             ///< ADC readings from ADS1115 channels (AIN0=A-series, AIN1=P/X-series)

/**
 * @brief NTC Thermistor parameters for temperature calculation
 * @note These values are for the voltage-divider circuit with R1 as the upper resistor
 */
double R1 = 4700.0;    ///< Voltage divider resistor value (ohms)
double Beta = 3950.0; ///< NTC Beta coefficient
double To = 298.15;   ///< Reference temperature (25°C in Kelvin)
double Ro = 100000.0; ///< NTC resistance at 25°C (ohms)

//==============================================================================
// Runtime Variables
//==============================================================================

int loopTime = 500;           ///< Main loop cycle time (ms)
unsigned long currentTime;    ///< Timestamp for loop timing

//==============================================================================
// Bluetooth Communication
//==============================================================================

int valCom;           ///< Parsed integer value from serial input
char key_type;       ///< Command character (a/b/c/d/e/p/i/d/f/r)

//==============================================================================
// EEPROM Default Values
//==============================================================================

int EPRflag = 0;              ///< EEPROM initialization flag
int EPRinitialTemp = 210;     ///< Default initial temperature
int EPRminTemp = 50;          ///< Default minimum temperature
int EPRmaxTemp = 280;         ///< Default maximum temperature (stored as value-100)
int EPRencIncrement = 5;      ///< Default encoder step
int EPRgetTempMethod = 1;      ///< Temperature method (1=lookup table)
int EPRPID_P = 60;            ///< Default PID P value
int EPRPID_I = 10;            ///< Default PID I value
int EPRPID_D = 5;             ///< Default PID D value
int EPRnozzleActive = 1;      ///< Default active nozzle (1=L, 0=R)

int getTempMethod = 1;        ///< Current temperature method

//==============================================================================
// Function Declarations
//==============================================================================

void rotaryInterrupt();           // Encoder interrupt service routine
void updateDataDisplay();          // Update OLED display
double gettabletemp();             // Get temperature from ADC using lookup table

//==============================================================================
// Setup Routine
//==============================================================================

/**
 * @brief Initialize hardware and load saved parameters
 *
 * This function is called once when the device starts. It initializes:
 * - Serial and Bluetooth communication
 * - OLED display
 * - ADS1115 ADC
 * - EEPROM parameters
 * - Encoder and buttons
 * - PID controller
 * - PWM outputs
 */
void setup() {
    // Initialize serial communication for debugging
    Serial.begin(115200);

    // Initialize Bluetooth with device name
    SerialBT.begin("BambuNRS"); // Bluetooth device name
    Serial.println("Device started. Pair via Bluetooth to configure parameters.");

    delay(10);

    // Initialize OLED display
    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println(F("SSD1306 allocation failed"));
        for(;;); // Halt on failure
    }

    // Rotate display 180 degrees for convenient viewing orientation
    display.setRotation(2);
    display.clearDisplay();

    Serial.println("Getting single-ended readings from AIN0..3");
    Serial.println("ADC Range: +/- 6.144V (1 bit = 3mV/ADS1015, 0.1875mV/ADS1115)");

    // Configure ADS1115 ADC gain
    // Gain 1: +/- 4.096V range, 1 bit = 0.125mV (16-bit mode)
    ads.setGain(GAIN_ONE);

    // Initialize ADS1115
    if (!ads.begin()) {
        Serial.println("Failed to initialize ADS1115.");
        while (1);
    }

    // Initialize EEPROM with predefined size
    EEPROM.begin(EEPROM_SIZE);

    // Read initialization flag from flash memory
    EPRflag = EEPROM.read(ADDR_EPR_FLAG);
    Serial.print("EEPROM flag: ");
    Serial.println(EPRflag);

    // First boot: write default parameters to flash
    if (EPRflag != 1) {
        EEPROM.write(ADDR_EPR_FLAG, 1);
        EEPROM.write(ADDR_INITIAL_TEMP, EPRinitialTemp);
        EEPROM.write(ADDR_MIN_TEMP, EPRminTemp);
        EEPROM.write(ADDR_MAX_TEMP, EPRmaxTemp - 100);  // Store as offset for 1-byte range
        EEPROM.write(ADDR_ENC_INCREMENT, EPRencIncrement);
        EEPROM.write(ADDR_TEMP_METHOD, EPRgetTempMethod);
        EEPROM.write(ADDR_PID_P, EPRPID_P);
        EEPROM.write(ADDR_PID_I, EPRPID_I);
        EEPROM.write(ADDR_PID_D, EPRPID_D);
        EEPROM.write(ADDR_NOZZLE_ACTIVE, EPRnozzleActive);
        EEPROM.commit();
        Serial.println("Default parameters saved to flash memory.");
    }
    // Subsequent boots: load saved parameters from flash
    else if (EPRflag == 1) {
        EPRinitialTemp = EEPROM.read(ADDR_INITIAL_TEMP);
        EPRminTemp = EEPROM.read(ADDR_MIN_TEMP);
        EPRmaxTemp = EEPROM.read(ADDR_MAX_TEMP) + 100;  // Restore offset
        EPRencIncrement = EEPROM.read(ADDR_ENC_INCREMENT);
        EPRgetTempMethod = EEPROM.read(ADDR_TEMP_METHOD);
        EPRPID_P = EEPROM.read(ADDR_PID_P);
        EPRPID_I = EEPROM.read(ADDR_PID_I);
        EPRPID_D = EEPROM.read(ADDR_PID_D);
        EPRnozzleActive = EEPROM.read(ADDR_NOZZLE_ACTIVE);

        // Apply loaded values to runtime variables
        encCurrent = EPRinitialTemp;
        encLowLim = EPRminTemp;
        encHighLim = EPRmaxTemp;
        encIncrement = EPRencIncrement;
        getTempMethod = EPRgetTempMethod;

        if (EPRnozzleActive == 1) {
            nozzleLActive = true;
        } else {
            nozzleLActive = false;
        }

        // Load PID parameters from EEPROM
        Kp = EPRPID_P;
        Ki = EPRPID_I;
        Kd = EPRPID_D;

        Serial.println("Parameters loaded from flash memory:");
        Serial.print("  encCurrent: "); Serial.println(encCurrent);
        Serial.print("  encLowLim: "); Serial.println(encLowLim);
        Serial.print("  encHighLim: "); Serial.println(encHighLim);
        Serial.print("  encIncrement: "); Serial.println(encIncrement);
        Serial.print("  getTempMethod: "); Serial.println(getTempMethod);
        Serial.print("  PID_P: "); Serial.println(EPRPID_P);
        Serial.print("  PID_I: "); Serial.println(EPRPID_I);
        Serial.print("  PID_D: "); Serial.println(EPRPID_D);
    }

    // Configure fan pins as outputs
    pinMode(PIN_FAN_L, OUTPUT);
    pinMode(PIN_FAN_R, OUTPUT);

    // Initialize PID controller
    pid.SetMode(AUTOMATIC);
    pid.SetOutputLimits(0, 255);  // PWM range: 0-255

    // Initialize input to room temperature
    input = 25.0;

    // Configure encoder pins with pull-up resistors
    pinMode(PIN_ENCODER_A, INPUT_PULLUP);
    pinMode(PIN_ENCODER_B, INPUT_PULLUP);

    // Attach encoder interrupts (both A and B phases for reliable detection)
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A), rotaryInterrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_B), rotaryInterrupt, CHANGE);

    // Configure button pins with pull-up resistors
    pinMode(PIN_ENCODER_BTN, INPUT_PULLUP);
    pinMode(PIN_BACK_BUTTON, INPUT_PULLUP);

    // Configure ESP32 LEDC PWM channels
    // ledcSetup(channel, frequency, resolution_bits)
    ledcSetup(PWM_CHANNEL_L, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_R, PWM_FREQ, PWM_RESOLUTION);

    // Attach PWM pins to LEDC channels
    ledcAttachPin(PIN_PWM_L, PWM_CHANNEL_L);
    ledcAttachPin(PIN_PWM_R, PWM_CHANNEL_R);

    Serial.println("Setup complete.");
}

//==============================================================================
// Main Loop
//==============================================================================

/**
 * @brief Main program loop
 *
 * Executes continuously:
 * 1. Process Bluetooth commands
 * 2. Read ADC channels for temperature
 * 3. Compute PID output
 * 4. Update PWM heater outputs
 * 5. Control cooling fans
 * 6. Update OLED display
 * 7. Handle button inputs (encoder + back button)
 */
void loop() {
    // Process Bluetooth commands from mobile app
    if (SerialBT.available()) {
        key_type = SerialBT.read();
        SerialBT.print("Command: ");
        SerialBT.print(key_type);
        valCom = SerialBT.parseInt();
        SerialBT.print("  Value: ");
        SerialBT.println(valCom);

        switch (key_type) {
            case 'a':  // Set initial temperature
                EEPROM.write(ADDR_INITIAL_TEMP, valCom);
                EEPROM.commit();
                SerialBT.println("initialTemp saved to flash memory");
                break;

            case 'b':  // Set minimum temperature
                EEPROM.write(ADDR_MIN_TEMP, valCom);
                EEPROM.commit();
                SerialBT.println("minTemp saved to flash memory");
                break;

            case 'c':  // Set maximum temperature
                EEPROM.write(ADDR_MAX_TEMP, (valCom - 100));  // Store as offset
                EEPROM.commit();
                SerialBT.println("maxTemp saved to flash memory");
                break;

            case 'e':  // Set encoder increment
                EEPROM.write(ADDR_ENC_INCREMENT, valCom);
                EEPROM.commit();
                SerialBT.println("encIncrement saved to flash memory");
                break;

            case 'f':  // Set temperature method
                EEPROM.write(ADDR_TEMP_METHOD, valCom);
                EEPROM.commit();
                SerialBT.println("getTempMethod saved to flash memory");
                break;

            case 'p':  // Set PID P value
                EEPROM.write(ADDR_PID_P, valCom);
                EEPROM.commit();
                SerialBT.println("PID_P saved to flash memory");
                break;

            case 'i':  // Set PID I value
                EEPROM.write(ADDR_PID_I, valCom);
                EEPROM.commit();
                SerialBT.println("PID_I saved to flash memory");
                break;

            case 'd':  // Set PID D value
                EEPROM.write(ADDR_PID_D, valCom);
                EEPROM.commit();
                SerialBT.println("PID_D saved to flash memory");
                break;

            case 'r':  // Reset EEPROM
                EEPROM.write(ADDR_EPR_FLAG, 0);
                EEPROM.commit();
                SerialBT.println("Flash memory reset! Reboot to restore defaults.");
                break;
        }
    }

    // Record loop start time
    currentTime = millis();

    // Read ADC channels for temperature
    adc0 = ads.readADC_SingleEnded(0);  // A-series nozzle NTC
    adc1 = ads.readADC_SingleEnded(1);  // P/X-series nozzle NTC

    // Get temperature using lookup table method
    input = gettabletemp();

    // Compute PID output
    pid.Compute();

    // Update PWM output based on active nozzle and heating status
    if (preHeatStatus == 1) {
        if (nozzleLActive == true) {
            ledcWrite(PWM_CHANNEL_L, output);  // A-series nozzle
        } else {
            ledcWrite(PWM_CHANNEL_R, output);  // P/X-series nozzle
        }
    } else {
        ledcWrite(PWM_CHANNEL_L, 0);  // Turn off both
        ledcWrite(PWM_CHANNEL_R, 0);
    }

    int temp = input;

    // Control cooling fan based on temperature and fan mode
    if (temp > 50) {
        if (fanActive == true) {
            if (nozzleLActive == true) {
                digitalWrite(PIN_FAN_L, HIGH);  // A-series fan
            } else {
                digitalWrite(PIN_FAN_R, HIGH);  // P/X-series fan
            }
        } else {
            digitalWrite(PIN_FAN_L, LOW);
            digitalWrite(PIN_FAN_R, LOW);
        }
    } else {
        digitalWrite(PIN_FAN_L, LOW);
        digitalWrite(PIN_FAN_R, LOW);
    }

    // Update OLED display
    updateDataDisplay();

    // Button debouncing loop - run for specified loopTime
    while (millis() < currentTime + loopTime) {
        // === Fan toggle button (back button) ===
        byte fanButtonState = digitalRead(PIN_BACK_BUTTON);
        if (fanButtonState != oldFanButtonState) {
            if (millis() - fanButtonPressTime >= DEBOUNCE_TIME) {
                fanButtonPressTime = millis();
                oldFanButtonState = fanButtonState;

                if (fanButtonState == LOW) {
                    fanpressed = true;
                } else {
                    if (fanpressed == true) {
                        fanpressed = false;
                        fanActive = !fanActive;  // Toggle fan mode
                    }
                }
            }
        }

        // === Encoder button ===
        byte buttonState = digitalRead(PIN_ENCODER_BTN);
        if (buttonState != oldButtonState) {
            if ((millis() - buttonPressTime) >= DEBOUNCE_TIME) {
                buttonPressTime = millis();
                oldButtonState = buttonState;

                if (buttonState == LOW) {
                    pressed = true;
                    button1stPressTime = millis();
                } else {
                    if (pressed == true) {
                        pressed = false;

                        // Long press (>1.5s): switch active nozzle
                        if ((millis() - button1stPressTime) >= NOZZLE_CHANGE_TIME) {
                            if (nozzleLActive == true) {
                                nozzleLActive = false;
                                EEPROM.write(ADDR_NOZZLE_ACTIVE, 0);
                                Serial.println("Active nozzle changed to P/X-series");
                            } else {
                                nozzleLActive = true;
                                EEPROM.write(ADDR_NOZZLE_ACTIVE, 1);
                                Serial.println("Active nozzle changed to A-series");
                            }
                            EEPROM.commit();
                            delay(5);
                        }
                        // Short press: toggle heating
                        else {
                            if (preHeatStatus == 0) {
                                if (temp > 3) {  // Valid temperature reading
                                    preHeatStatus = 1;
                                    encCurrent = EPRinitialTemp;
                                }
                            } else {
                                preHeatStatus = 0;  // Turn off heating
                            }
                        }
                    }
                }
            }
        }

        // Update PID setpoint from encoder position
        setpoint = encCurrent;
    }
}

//==============================================================================
// Temperature Reading Functions
//==============================================================================

/**
 * @brief Read temperature using lookup table interpolation
 *
 * Reads the ADC value for the active nozzle and converts it to
 * temperature using linear interpolation within the thermistor
 * lookup table.
 *
 * @return Temperature in degrees Celsius
 * @retval 0 If ADC reading is invalid
 * @retval "NC" (displayed on screen) If NTC is open circuit
 */
double gettabletemp() {
    int raw = 0;
    float celsius = 0;

    // Select ADC channel based on active nozzle
    if (nozzleLActive == true) {
        // A-series nozzle - use AIN0
        // Conversion: raw = adc / 25.78125 * OVERSAMPLENR (for gain=1, 4.096V range)
        raw = adc0 / 25.78125 * OVERSAMPLENR;
    } else {
        // P/X-series nozzle - use AIN1
        raw = adc1 / 25.78125 * OVERSAMPLENR;
    }

    // Linear interpolation within temperature table
    uint8_t i;
    int tableLen = 65;  // Number of entries in temptable_80

    for (i = 1; i < tableLen; i++) {
        if (temptable_80[i][0] > raw) {
            // Interpolate between two table entries
            celsius = temptable_80[i - 1][1] +
                      (raw - temptable_80[i - 1][0]) *
                      (float)(temptable_80[i][1] - temptable_80[i - 1][1]) /
                      (float)(temptable_80[i][0] - temptable_80[i - 1][0]);
            break;
        }
    }

    // Handle overflow: return last table value
    if (i == 65) {
        celsius = temptable_80[i - 1][1];
    }

    return celsius;
}

//==============================================================================
// Encoder Interrupt Service Routine
//==============================================================================

/**
 * @brief Rotary encoder interrupt handler
 *
 * Called on both rising and falling edges of encoder phases A and B.
 * Determines rotation direction by comparing the current phase A state
 * with the previous state, and checks phase B for direction.
 *
 * @note Uses interrupt-safe volatile variables
 */
void rotaryInterrupt() {
    encoderPos = digitalRead(PIN_ENCODER_A);

    if ((prevEncoderPos == 0) && (encoderPos == 1)) {
        // Rising edge detected on phase A
        if (digitalRead(PIN_ENCODER_B) == 0) {
            // Phase B is LOW = clockwise rotation
            if (encCurrent < encHighLim) {
                encCurrent += encIncrement;  // Increase temperature
            }
        } else {
            // Phase B is HIGH = counter-clockwise rotation
            if (encCurrent > encLowLim) {
                encCurrent -= encIncrement;  // Decrease temperature
            }
        }
    }

    prevEncoderPos = encoderPos;
}

//==============================================================================
// Display Functions
//==============================================================================

/**
 * @brief Update OLED display with current status
 *
 * Displays:
 * - Active nozzle indicator (-AH- or -PE-)
 * - P.V.: Present Value (actual temperature)
 * - S.V.: Setpoint Value (target temperature)
 * - FAN: Fan status (AUTO or OFF)
 */
void updateDataDisplay() {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);

    // Large text for main values
    display.setTextSize(2);

    display.setCursor(4, 12);
    display.print("P.V.:");
    display.setCursor(4, 29);
    display.print("S.V.:");
    display.setCursor(4, 48);
    display.print("FAN :");

    int temp = (int)input;
    int setPointInt = (int)setpoint;

    // Display actual temperature (P.V.)
    if (temp >= 100) {
        display.setCursor(85, 12);
        display.print(temp);
    } else {
        display.setCursor(97, 12);
        if (temp <= 3) {
            display.print("NC");  // Open circuit detected
        } else {
            display.print(temp);
        }
    }

    // Display target temperature (S.V.)
    if (preHeatStatus == 1) {
        if (setPointInt >= 100) {
            display.setCursor(85, 31);
        } else {
            display.setCursor(97, 31);
        }
        display.print(setPointInt);
    } else {
        display.setCursor(85, 31);
        display.print("---");  // Heating off
    }

    // Display fan status
    if (fanActive == true) {
        display.setCursor(79, 48);
        display.print("AUTO");
    } else {
        display.setCursor(85, 48);
        display.print("OFF");
    }

    // Display active nozzle indicator
    display.setTextSize(1);
    if (nozzleLActive == true) {
        display.setCursor(0, 0);
        display.print(F("-AH-"));  // A-series High-temp nozzle
    } else {
        display.setCursor(104, 0);
        display.print(F("-PE-"));  // P/X-series nozzle
    }

    display.display();
}

//==============================================================================
// Temperature Table Header
//==============================================================================

/**
 * @thermistortable_80.h
 *
 * Lookup table for NTC thermistor temperature calculation.
 * Format: { ADC_raw_value, Temperature_in_Celsius }
 *
 * Temperature range: 0°C to 360°C
 * Number of entries: 65
 *
 * This table is specifically calibrated for the NTC thermistors
 * used in Bambu Lab hotends with the voltage divider circuit
 * (R1 = 4.7k ohms upper resistor).
 */
