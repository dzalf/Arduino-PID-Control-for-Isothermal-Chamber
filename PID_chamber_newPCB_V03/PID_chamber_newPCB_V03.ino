
/*
  ================================================================================
    Based on  https://www.allaboutcircuits.com/projects/measuring-temperature-with-an-ntc-thermistor/
    File........... Thermistor_Demo_Code
    Purpose........ Thermistor demonstration code
    Author......... Joseph Corleto/Daniel Melendrez
    Description: This version is capable of reading both temoeraturee and humidity inside the chamber

  /**   LIBRARIES DECLARATION **/

#include <RotaryEncoder.h>
#include <DHT.h>
#include <Keypad.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h> // use the library: new_liquidCrystal from fmalpartida
#include <PID_v1.h>
#include <elapsedMillis.h>
#define KP 50
#define KI 2.00
#define KD 0.03
#define SETPOINT 20.0
#define OFFSET 4.6      // The thermal offset is 5.1 degrees Celsius --> to obtain 24C  // With respect to the temperature reported by the openQCMD
#define DHTTYPE DHT22

/**   VARIABLES DECLARATION **/
// Microwave matrix keypad configuration (reverse engineered ;) )
const byte ROWS = 4; //four rows
const byte COLS = 2; //four columns
unsigned int intervalHumidity = 3000;
// Each key is coded from 'A' to 'E'
char hexaKeys[ROWS][COLS] = {

  {'C', 'F'},
  {'B', 'E'},
  {'A', 'D'}
};
byte rowPins[ROWS] = {6, 7, 8}; //connect to the row pinouts of the keypad
byte colPins[COLS] = {4, 5}; //connect to the column pinouts of the keypad
char customKey = '-'; // Initialize pressed key holder variable
String toPrint = "   ";

/*I/O PORTS */
const int pinLED = 10; // Blue LED attached
const int relay = 15;  // Relay activation Pin. Active HIGH orignal 16
const int thermistorPin = 0;  // Where the ADC samples the resistor divider's output
const int DHTPIN = 0;
/* MULTIPLE VARIABLES */
float counter = 0.0;     // Rotary encoder variable
float value = 0.0;
double WindowSize = 5000;
double minWindow = 50;
unsigned long windowStartTime;
double newSetpoint, oldSetpoint, InputPID, DigitalSetpointPID, Output, alpha, dummy2;;
double oldKp, oldKd, oldKi, newKp, newKd, newKi;
double adcAverage  = 0;            // Holds the average voltage measurement
double humidityVal;
int PWMOn  = 255;   // PWM pulse duty cicles (ON)
int PWMOff  = 0;    // (OFF)
boolean flag = false;      // Activation/enable flag (for STOPPING conversion routine)
boolean enterFlag = false;
boolean paramChange = false;
//unsigned long previousTime = 0;   // Used for timing purposes
const int    SAMPLE_NUMBER  = 10; // Capture samples to avoid noise
double currentTemperature;    // save the current temperature
/* In order to use the Beta equation, we must know our other resistor
   within our resistor divider. If you are using something with large tolerance,
   like at 5% or even 1%, measure it and place your result here in ohms. */
const double BALANCE_RESISTOR   = 9999;//9976.1; // Measured
// This helps calculate the thermistor's resistance (check article for details).
const double MAX_ADC            = 1023.0;
/* This is thermistor dependent and it should be in the datasheet, or refer to the
   article for how to calculate it using the Beta equation.
   I had to do this, but I would try to get a thermistor with a known
   beta if you want to avoid empirical calculations. */
const double BETA               = 3977.0;      // eBay Vishay Thermistor
/* This is also needed for the conversion equation as "typical" room temperature
   is needed as an input. */
const double ROOM_TEMP          = 298.15;   // room temperature in Kelvin
/* Thermistors will have a typical resistance at room temperature so write this
   down here. Again, needed for conversion equations. */
const double RESISTOR_ROOM_TEMP = 10000.0;
// Degrees symbol creation
byte degreesSymbol[8] = {
  0b00000,
  0b01110,
  0b01010,
  0b01110,
  0b00000,
  0b00000,
  0b00000,
  0b00000
};
// Separating bar creation
byte separatorSymbol[8] = {
  0b01110,
  0b01110,
  0b01110,
  0b01110,
  0b01110,
  0b01110,
  0b01110,
  0b01110
};

DHT dht(DHTPIN, DHTTYPE);
Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS);
LiquidCrystal_I2C lcd(0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE); //0x3F 0x27
RotaryEncoder encoder(16, 14); // Setup a RoraryEncoder for pins 14 and 15-RETURN THIS VALUE: 14 and 15 originals

void setup() {
  delay(50);
  //initialize the variables we're linked to
  oldSetpoint = SETPOINT - OFFSET;
  oldKp = KP;
  oldKi = KI;
  oldKd = KD;
  alpha = 0.0;
  dummy2 = 0.0;

  // Load "new" values into the memory in 'AUTOMATIC' mode
  proportionalValue(0);
  integralValue(0);
  derivativeValue(0);
  setpointValue(0);

  // The Interrupt Service Routine for Pin Change Interrupt 1. Changed using datasheet
  // **
  //                                **DO NOT TOUCH!!!!!
  //**
  PCICR |= (1 << PCIE0);    // This enables Pin Change Interrupt 1 that covers the Analog input pins or Port B.
  PCMSK0 |= (1 << PCINT2) | (1 << PCINT3); // This enables the interrupt for pin 2 and 3 of Port B.
  // **
  // ******************************  IT WORKS! D O  N O T  T O U C H  ****************************

  pinMode(relay, OUTPUT);
  pinMode(thermistorPin, INPUT);

  digitalWrite(relay, LOW);
  dht.begin();
  //Serial.begin(115200);
  lcd.begin(20, 4);// initialize the lcd for 20 chars 4 lines
  lcd.backlight(); // backlight on
  lcd.clear();
  lcd.createChar(0, degreesSymbol);
  lcd.createChar(1, separatorSymbol);
  lcd.setCursor(0, 0);
  lcd.print("*Thermistor reading*");
  lcd.setCursor(0, 2);
  lcd.print("   Initializing ");

  for (int i = 0; i < 20; i++) {
    lcd.setCursor(i, 3);
    lcd.print(".");
    delay(50);
  }

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("*Thermistor reading*");
  lcd.setCursor(0, 3);
  lcd.print(" Kp-> ");
  lcd.print(oldKp);

  lcd.setCursor(11, 2);
  lcd.write((uint8_t)1);
  lcd.setCursor(12, 2);
  lcd.print("Key-> ");

  lcd.setCursor(11, 3);
  lcd.write((uint8_t)1);
  lcd.print("STOPPED");

  lcd.setCursor(4, 1);
  lcd.print("PELTIER OFF!");

}

void loop() {

  lcd.setCursor(0, 0);
  lcd.print("*Thermistor reading*");
  dummy2 = readThermistorADC();
  currentTemperature = Celsius(dummy2);
  readKeyPad();

  if (flag) {
    PIDRoutine();
  }

  //  Serial.print("Current Temperature (NO OFFSET)[°C]-> ");
  //  Serial.print(currentTemperature);
  //  Serial.print("Humidity --> ");
  //  Serial.print(humidityVal);
  //  Serial.print("; ");
  //  Serial.print("Kp-> ");
  //  Serial.print(newKp);
  //  Serial.print("; ");
  //  Serial.print("Ki-> ");
  //  Serial.print(newKi);
  //  Serial.print("; ");
  //  Serial.print("Kd-> ");
  //  Serial.print(newKd);
  //  Serial.print("; ");
  //  Serial.print("DesiredTemp-> ");
  //  Serial.print(newSetpoint);
  //  Serial.print("\n");

}

/********************************************** FUNCTIONS********************************************** **/

void readKeyPad() {

  customKey = customKeypad.getKey();

  if (customKey) {
    flash(50);
    lcd.setCursor(11, 2);
    lcd.write((uint8_t)1);
    lcd.setCursor(12, 2);
    lcd.print("Key->");

    keyPressed();

    lcd.print(toPrint);
  }
  //*** PROPORTIONAL VALUE ***
  if (customKey == 'A' ) {
    proportionalValue(1);  //manual mode
  }
  //*** INTEGRAL VALUE  ***
  if (customKey == 'B' ) {
    integralValue(1);

  }
  //*** DERIVATIVE VALUE  ***
  if (customKey == 'C' ) {
    derivativeValue(1);
  }
  //*** SET POINT VALUE  ***
  if (customKey == 'D' ) {
    setpointValue(1);
  }

  //*** START ***
  if (customKey == 'E' ) {
    flag = true;
  }

  //*** STOP ***
  if (customKey == 'F' ) {
    flag = false;
  }


}
void PIDRoutine() {

  elapsedMillis timeEnlapsed;

  windowStartTime = millis();
  lcd.setCursor(0, 3);
  lcd.print(" Sp->");
  lcd.print(newSetpoint + OFFSET);
  lcd.setCursor(11, 3);
  lcd.write((uint8_t)1);
  lcd.print("RUNNING");

  lcd.setCursor(11, 1);
  lcd.write((uint8_t)1);
  lcd.print("H=");
  lcd.print("0.00%");

  // Convert degrees to ADC Digital values
  alpha = (BETA * (ROOM_TEMP - (newSetpoint + 273.15))) / ((newSetpoint + 273.15) * ROOM_TEMP);
  dummy2 = RESISTOR_ROOM_TEMP * exp(alpha);
  DigitalSetpointPID = (MAX_ADC * BALANCE_RESISTOR) / (dummy2 + BALANCE_RESISTOR);

  PID myPID(&InputPID, &Output, &DigitalSetpointPID, newKp, newKi, newKd, REVERSE);
  myPID.SetOutputLimits(0, WindowSize);  //tell the PID to range between 0 and the full window size
  myPID.SetMode(AUTOMATIC);  //turn the PID on
  myPID.SetOutputLimits(0, WindowSize);  //tell the PID to range between 0 and the full window size

  do {

    InputPID = readThermistorADC();
    currentTemperature = Celsius(InputPID);
    myPID.Compute(); //Approach 1
    unsigned long now = millis();

    if (now - windowStartTime > WindowSize) { //Approach 1
      windowStartTime += WindowSize; //time to shift the Relay Window and recalculate

    }
    if (Output > now - windowStartTime) {//Approach 1

      lcd.setCursor(0, 1);
      lcd.print("PELTIER ON!");
      digitalWrite(relay, HIGH);
      toggleLED(1);
    } else {
      lcd.setCursor(0, 1);
      lcd.print("PELTIER OFF");
      digitalWrite(relay, LOW);
      toggleLED(0);
    }

    customKey = customKeypad.getKey();

    keyPressed();

    //*** STOP ***
    if (customKey == 'F' ) {
      flag = false;
      lcd.setCursor(0, 1);
      lcd.print("     PELTIER OFF!   ");
      lcd.setCursor(11, 3);
      lcd.write((uint8_t)1);
      lcd.print("STOPPED");
      lcd.setCursor(12, 2);
      lcd.print("Key->");
      lcd.print(toPrint);
      toggleLED(0);
    }

    // Routine for humidity detection

    if (timeEnlapsed > intervalHumidity) {

      humidityVal = dht.readHumidity();
      lcd.setCursor(11, 1);
      lcd.write((uint8_t)1);
      lcd.print("H=");
      lcd.print(humidityVal);
      lcd.print("%");
    }

    //    Serial.print("windowStartTime-> ");
    //    Serial.print(windowStartTime);
    //    Serial.print("; ");
    //    Serial.print("now-> ");
    //    Serial.print(now);
    //    Serial.print("; ");
    //    Serial.print("InputPID-> ");
    //    Serial.print(InputPID);
    //    Serial.print("; ");
    //    Serial.print("DigitalSetpointPID-> ");
    //    Serial.print(DigitalSetpointPID);
    //    Serial.print("; ");
    //    Serial.print("Output-> ");
    //    Serial.println(Output);
    //    Serial.print("\n ");
  } while (flag);

  digitalWrite(relay, LOW);
  toggleLED(0);

  lcd.setCursor(0, 1);
  lcd.print("     PELTIER OFF!   ");
  delay(50);
}

void keyPressed() {

  switch (customKey) {

    case 'A':
      toPrint = "Kp ";
      break;

    case 'B':
      toPrint = "Ki ";

      break;

    case 'C':
      toPrint = "Kd ";
      break;

    case 'D':

      toPrint = "Sp ";
      break;

    case 'E':
      toPrint = "RUN";
      break;

    case 'F':

      toPrint = "OFF";
      break;
  }
}


void proportionalValue(int mode) {

  if (mode == 1) { //if manual mode selected
    toggleFlag();
    toggleLED(1);
    encoder.setPosition(0);
    lcd.setCursor(0, 3);
    lcd.print(" Kp-> ");
    lcd.print(oldKp);

    while (enterFlag) {

      static int pos = 0.0;
      counter = encoder.getPosition();
      value = counter / 100;

      if (pos != counter) {
        toggleParameter(1);    // If the parameter is changed a flag indicates so (T -> F)
        newKp = oldKp + value;

        if (newKp < 0) {
          newKp = 0;
          encoder.setPosition(0);
        }
        lcd.setCursor(0, 3);
        lcd.print(" Kp-> ");
        lcd.print(newKp);
        pos = counter;
      } else if (paramChange == false) { // If the parameter is NOT changed just keep the last one
        newKp = oldKp;
      }

      customKey = customKeypad.getKey();
      if (customKey == 'A' ) {
        toggleFlag();
        toggleLED(0);
        encoder.setPosition(0);
        oldKp = newKp;
      }
    }
    toggleParameter(0); // It's FALSE again to indicate that the knob hasn't been moved
  }
  else if (mode == 0) { //AUTOMATIC MODE
    newKp = oldKp;
  }
}

void integralValue(int mode) {
  if (mode == 1) { //if manual mode selected
    toggleFlag();
    toggleLED(1);
    encoder.setPosition(0);
    lcd.setCursor(0, 3);
    lcd.print(" Ki-> ");
    lcd.print(oldKi);
    while (enterFlag) {

      static int pos = 0.0;
      counter = encoder.getPosition();
      value = counter / 100;

      if (pos != counter) {
        toggleParameter(1);    // If the parameter is changed a flag indicates so (T -> F)
        newKi = oldKi + value;

        if (newKi < 0) {
          newKi = 0;
          encoder.setPosition(0);
        }
        lcd.setCursor(0, 3);
        lcd.print(" Ki-> ");
        lcd.print(newKi);
        pos = counter;
      } else if (paramChange == false) { // If the parameter is NOT changed just keep the last one
        newKi = oldKi;
      }

      customKey = customKeypad.getKey();
      if (customKey == 'B' ) {
        toggleFlag();
        toggleLED(0);
        encoder.setPosition(0);
        oldKi = newKi;
      }
    }
    toggleParameter(0); // It's FALSE again to indicate that the knob hasn't been moved
  }
  else if (mode == 0) {
    newKi = oldKi;
  }
}

void derivativeValue(int mode) {
  if (mode == 1) { //if manual mode selected
    toggleFlag();
    toggleLED(1);
    encoder.setPosition(0);
    lcd.setCursor(0, 3);
    lcd.print(" Kd-> ");
    lcd.print(oldKd);
    while (enterFlag) {

      static int pos = 0.0;
      counter = encoder.getPosition();
      value = counter / 100;

      if (pos != counter) {
        toggleParameter(1);    // If the parameter is changed a flag indicates so (T -> F)
        newKd = oldKd + value;

        if (newKd < 0) {
          newKd = 0;
          encoder.setPosition(0);
        }
        lcd.setCursor(0, 3);
        lcd.print(" Kd-> ");
        lcd.print(newKd);
        pos = counter;
      } else if (paramChange == false) { // If the parameter is NOT changed just keep the last one
        newKd = oldKd;
      }

      customKey = customKeypad.getKey();
      if (customKey == 'C' ) {
        toggleFlag();
        toggleLED(0);
        encoder.setPosition(0);
        oldKd = newKd;
      }
    }
    toggleParameter(0); // It's FALSE again to indicate that the knob hasn't been moved
  }
  else if (mode == 0) {
    newKd = oldKd;
  }
}

void setpointValue(int mode) {
  if (mode == 1) { //if manual mode selected
    toggleFlag();
    toggleLED(1);
    encoder.setPosition(0);
    lcd.setCursor(0, 3);
    lcd.print(" Sp->");
    lcd.print(oldSetpoint + OFFSET);

    while (enterFlag) {

      static int pos = 0.0;
      counter = encoder.getPosition();
      value = counter / 10;

      if (pos != counter) {
        toggleParameter(1);    // If the parameter is changed a flag indicates so (T -> F)
        newSetpoint = oldSetpoint + value;

        if (newSetpoint <= 0) {
          newSetpoint = 0;
          encoder.setPosition(0);
          value = 0;
          oldSetpoint = 0;
        }
        lcd.setCursor(0, 3);
        lcd.print(" Sp->");
        lcd.print(newSetpoint + OFFSET);
        pos = counter;
      } else if (paramChange == false) { // If the parameter is NOT changed just keep the last one
        newSetpoint = oldSetpoint;
      }

      customKey = customKeypad.getKey();
      if (customKey == 'D' ) {
        toggleFlag();
        toggleLED(0);
        encoder.setPosition(0);
        oldSetpoint = newSetpoint;
      }
    }
    toggleParameter(0); // It's FALSE again to indicate that the knob hasn't been moved
  }
  else if (mode == 0) {
    newSetpoint = oldSetpoint;
  }
}

double readEncoder() {
  static int pos = 0;
  counter = encoder.getPosition();
  if (pos != counter) {
    pos = counter;
  }
  return counter / 10;
}

double readThermistorADC() { // for InputPID
  // variables that live in this function

  adcAverage  = 0;            // Holds the average voltage measurement

  int    adcSamples[SAMPLE_NUMBER];  // Array to hold each voltage measurement

  for (int i = 0; i < SAMPLE_NUMBER; i++) {
    adcSamples[i] = analogRead(thermistorPin);  // read from pin and store
    delay(5);        // wait SAMPLE_NUMBER milliseconds
  }

  for (int i = 0; i < SAMPLE_NUMBER; i++) {
    adcAverage += adcSamples[i];      // add all samples up . . .
  }
  adcAverage /= SAMPLE_NUMBER;        // . . . average it w/ divide

  return adcAverage;    // Return the temperature in Celsius
}

double Celsius(double X) {
  double rThermistor = 0;            // Holds thermistor resistance value
  double tKelvin     = 0;            // Holds calculated temperature
  double tCelsius    = 0;            // Hold temperature in celsius
  /* Here we calculate the thermistor’s resistance using the equation
     discussed in the article. */
  rThermistor = BALANCE_RESISTOR * ( (MAX_ADC / X) - 1);

  tKelvin = (BETA * ROOM_TEMP) / (BETA + (ROOM_TEMP * log(rThermistor / RESISTOR_ROOM_TEMP)));

  tCelsius = tKelvin - 273.15;  // convert kelvin to celsius

  lcd.setCursor(1, 2);
  lcd.print("T=");
  lcd.print(tCelsius + OFFSET);
  lcd.write((uint8_t)0);
  lcd.print("C ");

  return tCelsius;    // Return the temperature in Celsius
}


void flash(int time) {
  analogWrite(pinLED, PWMOn);
  delay(time);
  analogWrite(pinLED, PWMOff);
}

void toggleLED(int state) {
  if (state == 1) {
    analogWrite(pinLED, PWMOn);
  } else {
    analogWrite(pinLED, PWMOff);
  }
}

void toggleParameter(int val) {
  if (val == 0) {
    paramChange = false;
  }
  if (val == 1) {
    paramChange = true;
  }
}

void toggleFlag() {
  enterFlag = !enterFlag;
}

ISR(PCINT0_vect) {
  // flash(10);
  encoder.tick(); // just call tick() to check the state.
}
