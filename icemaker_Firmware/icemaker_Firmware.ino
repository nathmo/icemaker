// sparkfun MCU, add board manager : File->Preference->
//add link https://raw.githubusercontent.com/sparkfun/Arduino_Boards/main/IDE_Board_Manager/package_sparkfun_index.json
// then find the board : Tools -> Board -> Board manager -> sparkfun AVR
// once installed, select sparkfun Micro Pro

#include <math.h>  // Include math library for sin, cos, and M_PI

// Pin Definitions
const int thawValvePin = 4;      // Thaw valve (active high)
const int scoopMotorPin = 5;     // Scoop motor (active high)
const int waterPumpPin = 6;      // Water pump (active high)
const int fanPin = 7;            // Fan (active high)
const int compressorPin = 9;     // Compressor (active high)
const int signalSquarePin = 3;   // square signal at 1kHz
const int ledOnOffPin = 16;      // LED signaling on/off (active high)
const int ledPins[3] = {15, 14, 10}; // Charlieplexed LEDs on D15, D14, D10

// Analog Inputs
const int buttonPin = A1;        // Two buttons on resistor divider (Size/On-Off)
const int switchPin = A3;        // Two end switches on resistor divider
const int irPhotodiodePin = A2;  // IR photodiode with 100k resistor (analog input)
const int waterContactPin = A0;  // Water contact detection (analog input)
const int ntcPin = A8;           // NTC with 100k resistor (ambient temperature)

// Constants
const int debounceSamples = 10;  // Debouncing requires stable state over 10 samples
const int irPwmFrequency = 1000; // IR signal is modulated at 1kHz
const float debounceStateMargin = 0.2;  // 20% margin for debounce
const int bufferSize = 100;      // Circular buffer size

int detectionThresholdWater = 1000000;  // Threshold for water signal detection
int detectionThresholdIR = 1000000;  // Threshold for IR signal detection

// Internal state variables
bool thawValveState = false;
bool scoopMotorState = false;
bool waterPumpState = false;
bool fanState = false;
bool compressorState=false;
bool ledOnOffState = false;
bool ledSizeSState = false;
bool ledSizeLState = false;
bool ledAddWaterState = false;
bool ledIceFullState = false;
bool buttonState = false;
bool buttonSizeSelectState = false;
bool onMachine = true;             // start after power is restored (allow a simple smartwitch to stop and start the machine)
bool SizeSelectLarge = true;
bool frontSwitchState = false;
bool backSwitchState = false;
bool waterDetected = false;
bool irDetected = false;
bool ambientTemperatureAlarm = false;
bool ambientTemperatureCoolEnough = false;


// Circular buffers for IR photodiode and water contact
int irBuffer[bufferSize];
int waterBuffer[bufferSize];
int irBufferIndex = 0;
int waterBufferIndex = 0;
bool irPwmDetected = false;
bool waterPwmDetected = false;

// Debounce samples
int buttonSamples[debounceSamples];
int switchSamples[debounceSamples];

int avg(int *samples) {
    // Compute the average of the samples
    long int sum = 0;
    for (int i = 0; i < debounceSamples; i++) {
        sum += samples[i];
    }
    int average = sum / debounceSamples;

    return average;
}

// Debouncing function, add the new sample to the list and return true if the sample are stable
bool debounce(int *samples) {
    int average = avg(samples);

    // Check if all samples are within the margin of the average
    for (int i = 0; i < debounceSamples; i++) {
        if (abs(samples[i] - average) > average * debounceStateMargin) {
            return false;  // One or more samples are outside the margin
        }
    }
    return true;  // All samples are within the margin, consider debounced
}

void sampleButtons() {
    // Shift samples and add the new one
    for (int i = debounceSamples - 1; i > 0; i--) {
        buttonSamples[i] = buttonSamples[i - 1];
    }
    buttonSamples[0] = analogRead(buttonPin);
    // Shift samples and add the new one
    for (int i = debounceSamples - 1; i > 0; i--) {
        switchSamples[i] = switchSamples[i - 1];
    }
    switchSamples[0] = analogRead(switchPin);
}

void updateLEDs() {
    // run at 100 Hz
    static int currentLed = 0;  // Keeps track of which LED to refresh (static)
    if (ledOnOffState != digitalRead(ledOnOffPin)){
      digitalWrite(ledOnOffPin, ledOnOffState);   // the on/off LED is not charlieplexed.
    }
    // Turn off all pins before changing states (set to high impedance)
    for (int i = 0; i < 3; i++) {
        pinMode(ledPins[i], INPUT);  // Set all pins to INPUT to turn off LEDs
    }
    // Alternate between LEDs based on internal state using if-else if
    if (currentLed == 0 && ledSizeSState) {  // Size S
        pinMode(ledPins[0], OUTPUT);     // D15
        pinMode(ledPins[2], OUTPUT);     // D10
        digitalWrite(ledPins[0], LOW);   // 0V
        digitalWrite(ledPins[2], HIGH);  // D10 = 5V
    }
    else if (currentLed == 1 && ledSizeLState) {  // Size L
        pinMode(ledPins[0], OUTPUT);     // D15
        pinMode(ledPins[2], OUTPUT);     // D10
        digitalWrite(ledPins[0], HIGH);  // 5V
        digitalWrite(ledPins[2], LOW);   // D10 = 0V
    }
    else if (currentLed == 2 && ledAddWaterState) {  // Add Water
        pinMode(ledPins[1], OUTPUT);     // D14
        pinMode(ledPins[2], OUTPUT);     // D10
        digitalWrite(ledPins[1], LOW);   // 0V
        digitalWrite(ledPins[2], HIGH);  // D10 = 5V
    }
    else if (currentLed == 3 && ledIceFullState) {  // Ice Full
        pinMode(ledPins[1], OUTPUT);     // D14
        pinMode(ledPins[2], OUTPUT);     // D10
        digitalWrite(ledPins[1], HIGH);  // 5V
        digitalWrite(ledPins[2], LOW);   // D10 = 0V
    }
    // Move to the next LED in the next refresh
    currentLed = (currentLed + 1) % 4;
}

void sampleIRandWater() {
    // run at 2 khz
    // Read IR photodiode and water contact
    int irSample = analogRead(irPhotodiodePin);
    int waterSample = analogRead(waterContactPin);

    // Store samples in circular buffer
    irBuffer[irBufferIndex] = irSample;
    waterBuffer[waterBufferIndex] = waterSample;

    // Update buffer index
    irBufferIndex = (irBufferIndex + 1) % bufferSize;
    waterBufferIndex = (waterBufferIndex + 1) % bufferSize;
}

// Goertzel function for detecting a target frequency (e.g., 1 kHz)
float goertzel(int numSamples, int* data, float targetFrequency, float sampleRate) {
    float k = 0.5 + ((numSamples * targetFrequency) / sampleRate);  // Frequency bin index
    float omega = (2.0 * M_PI * k / numSamples);  // Calculate omega
    float coeff = 2.0 * cos(omega);  // Coefficient for Goertzel recurrence
    float q0 = 0, q1 = 0, q2 = 0;  // Goertzel filter variables

    for (int i = 0; i < numSamples; i++) {
        q0 = coeff * q1 - q2 + data[i];  // Apply Goertzel recurrence
        q2 = q1;
        q1 = q0;
    }

    // Calculate the real and imaginary components of the result
    float realPart = q1 - q2 * cos(omega);
    float imagPart = q2 * sin(omega);

    // Return magnitude squared (real^2 + imag^2)
    return realPart * realPart + imagPart * imagPart;
}

void detectIRandWater() {
    // SAMPLE_RATE 5000  // Sampling rate (Hz)
    // TARGET_FREQUENCY 1000  // Target frequency (Hz)

    float  irMagnitude = goertzel(bufferSize, irBuffer, 1000.0, 2000.0);
    float  waterMagnitude = goertzel(bufferSize, waterBuffer, 1000.0, 2000.0);

    // Update flags based on magnitude exceeding threshold
    irDetected = (irMagnitude < detectionThresholdIR);
    waterDetected = (waterMagnitude < detectionThresholdWater);
}

float readTemperature() {
    const float R_nominal = 5500;   // 10k ohms nominal resistance at 25°C
    const float beta = 1500;        // Beta value of the NTC roughly calculated
    const float T_nominal = 25.0;   // Temperature at nominal resistance (25°C)
    const float R_series = 10000;   // 10k ohms series resistor
    // Step 1: Read the voltage at the NTC pin (A8)
    int analogValue = analogRead(ntcPin);

    // Step 2: Calculate the voltage across the NTC (V_ntc)
    float V_ntc = analogValue * (5.0 / 1023);

    // Step 3: Calculate the resistance of the NTC using the voltage divider formula
    //float R_ntc = (R_series * V_ntc) / (5.0 - V_ntc);
    float R_ntc = R_series * ((5.0 / V_ntc) - 1.0);

    // Step 4: Convert the NTC resistance to temperature using the Beta formula
    float tempK = 1.0 / (1.0 / (T_nominal + 273.15) + (1.0 / beta) * log(R_ntc / R_nominal));  // Temperature in Kelvin
    float tempC = tempK - 273.15;  // Convert Kelvin to Celsius
    
    return tempC;  // Return temperature in Celsius
}

// First Refresh Function (100Hz Timer Interrupt)
void refreshState() {
    // 10 Hz refresh rate
    detectIRandWater(); // check if have the PWM
    
    // Read button analog value
    if (debounce(buttonSamples)){
      if (avg(buttonSamples) < 100) {
          if(!buttonState){
            onMachine = !onMachine;
          }
          buttonState = true;  // On/Off button pressed
          buttonSizeSelectState = false; 
      } else if (avg(buttonSamples) < 1024*0.8) {
          buttonSizeSelectState = true;  // Size Select button pressed
          if(!buttonState){
            SizeSelectLarge = !SizeSelectLarge;
          }
          buttonState = false;  
      } else {
          buttonState = false;
          buttonSizeSelectState = false;
      }
    }
    // Read end switches analog value
    if (debounce(switchSamples)){
      if (avg(switchSamples) < 100) {
          backSwitchState = true;  // Back switch pressed
          frontSwitchState = false;
      } else if (avg(switchSamples) < 1024*0.8) {
          frontSwitchState = true;  // Front switch pressed
          backSwitchState = false;
      } else {
          backSwitchState = false;
          frontSwitchState = false;
      }
    }

    if(irDetected){
      ledIceFullState = false;
    } else {
      ledIceFullState = true;
    }

    if(onMachine){
      ledOnOffState = true;
    } else {
      ledOnOffState = false;
    }

    if(SizeSelectLarge){
      ledSizeLState = true;
      ledSizeSState = false;
    } else {
      ledSizeLState = false;
      ledSizeSState = true;
    }

    if (thawValveState != digitalRead(thawValvePin)){
      digitalWrite(thawValvePin, thawValveState);   
    }
    if (scoopMotorState != digitalRead(scoopMotorPin)){
      digitalWrite(scoopMotorPin, scoopMotorState);   
    }
    if (waterPumpState != digitalRead(waterPumpPin)){
      digitalWrite(waterPumpPin, waterPumpState); 
    }
    if (fanState != digitalRead(fanPin)){
      digitalWrite(fanPin, fanState); 
    }
    if (compressorState != digitalRead(compressorPin)){
      digitalWrite(compressorPin, compressorState);
    }
    //get the current temperature and set alarm if greater than 60°
    ambientTemperatureAlarm = (readTemperature()>60);
    ambientTemperatureCoolEnough = (readTemperature()<30);
}

void processMachineState() {
  // run at 10Hz
  static int state = 0;             // State variable to track progress
  static unsigned long timer = 0;   // Timer for delays
  unsigned long currentMillis = millis(); // Get current time
  Serial.print("---- State : ");
  Serial.print(state);            // Print the value with a newline
  Serial.print(" Statetimer : ");
  Serial.println(timer);            // Print the value with a newline

  switch (state) {
    case 0: // state off
      compressorState = false;
      fanState = false;
      if(onMachine){
        state = 4;                   // Move to on state
      } else {
        state = 0;                   // Move to the next state
      }
      timer = currentMillis;         // Start timer for delay
    case 1: // state no water
      compressorState = false;
      fanState = false;
      if(onMachine){
        state = 1;                   // stay here
      } else {
        state = 0;                   // go to off (power cycle on water reffiled)
      }
      timer = currentMillis;       // Start timer for delay
      state = 1;                   // Move to the next state
    case 2: // state ice full
      compressorState = false;
      fanState = false;
      if(irDetected){
        state = 4;                   // ice melted, re-run
      } else {
        state = 2;                   // ice still full, stay
      }
      timer = currentMillis;       // Start timer for delay
    case 3: // state overtemp
      compressorState = false;
      fanState = true;
      if(ambientTemperatureCoolEnough){
        state = 4;                   // Resume Ice production
      } else {
        state = 3;                   // system too hot, waiting to cooldown
      }
      timer = currentMillis;       // Start timer for delay
    case 4: // commence a recule la pelle
        if(backSwitchState){
          scoopMotorState = false;    // dont need to move if aldready in position
        } else  {
          scoopMotorState = true;     // start the scoop
        }
        state = 5;
        timer = currentMillis;       // Start timer for delay
      break;
    case 5: // verifie que la pelle a atteint la position voulu.
      if ((currentMillis - timer >= 2000)) { // Wait for 2 seconds
        if(backSwitchState){           // if correct side
          scoopMotorState = false;     // stop la pelle 
          state = 6;                   // Move to the next state
        } else if (frontSwitchState) { // if wrong side
          scoopMotorState = false;     // stop la pelle 
          state = 4;                   // Move to the previous state and try again
        } else {
          state = 5;                   // keep moving until a switch is reached
        }
        timer = currentMillis;       // Start timer for delay
      }
      break;
    case 6: // commence a faire tomber les glaçon
      thawValveState = true;       // Ouvrir bypass
      compressorState = true;      // needed to melt the ice
      fanState = true;             // needed to cool the compressor gaz
      timer = currentMillis;       // Start timer for delay
      state = 7;                   // Move to the next state
      break;
    case 7: // arrete de faire tomber les glaçon
      if (currentMillis - timer >= 10000) { // Wait for 10 seconds so ice have time to melt
        thawValveState = false;      // Fermer bypass
        timer = currentMillis;       // Reset timer for next delay
        state = 8;                   // Move to the next state

      }
      break;
    case 8: // start moving forward the scoop
        if(frontSwitchState){
          scoopMotorState = false;    // dont start if aldready in position
        } else  {
          scoopMotorState = true;     // start la pelle 
        }
        state = 9;                    // keep moving until a switch is reached
        timer = currentMillis;        // Start timer for delay
      break;
    case 9: // check that the scoop reached the front (or try again)
      if ((currentMillis - timer >= 2000)) { // Wait for 2 seconds
        if(backSwitchState){           // if the scoop went the wrong way,
          scoopMotorState = false;     // stop la pelle 
          state = 8;                   // Move to the previous state and try again
        } else if (frontSwitchState) { // if it went to the correct position
          scoopMotorState = false;     // stop la pelle 
          if(irDetected){
            state = 10;                 // continue the cycle
          } else {
            state = 2;                  // ice is full, pause
          }
        } else {
          state = 9;                   // keep moving until a switch is reached
        }
        timer = currentMillis;         // Start timer for delay
      }
      break;
    case 10:// start filling up the scoop
        waterPumpState = true;      // Start the water pump
        timer = currentMillis;      // Reset timer for next delay
        state = 11;                  // Move to the next state
      break;
    case 11:// check that water is flowing in the scoop
      if(currentMillis - timer >= 5000){ // wait 5 sec for the water to start flowing
        if(waterDetected or true){      // water detection is unreliable. disabling for now.
          ledAddWaterState = false;
          state = 12;                 // Move to the next state
        } else {
          ledAddWaterState = true;
          state = 1; // go to state add water
        }
        timer = currentMillis;      // Reset timer for next delay
      }
      break;
    case 12: // stop filling up the scoop start the ice
      if (currentMillis - timer >= 15000) { // Wait for 15 seconds
        waterPumpState = false;     // Stop the water pump
        compressorState = true;     // enable compressor to make ice (should aldready be enabled)
        fanState = true;            // needed to cool the compressor radiator
        timer = currentMillis;
        if (ledSizeSState) {
          state = 13;                // Move to 5 min delay state
        } else {
          state = 14;                // Move to 10 min delay state
        }
      }
      break;
    case 13: // make small ice cube
      if (currentMillis - timer >= 300000) { // Wait for 5 minutes
        state = 4;                  // Reset to the beginning
      }
      break;
    case 14: // make big ice cube
      if (currentMillis - timer >= 600000) { // Wait for 10 minutes
        state = 4;                  // Reset to the beginning
      }
      break;
  }
}

void pwmManualSerialCalibration() {
  Serial.println("----------------------------------");
  waterPumpState = true;
  Serial.print("waterDetected: "); Serial.println(waterDetected);
  Serial.print("irDetected: "); Serial.println(irDetected);
  Serial.print("irMagnitude: "); Serial.println(goertzel(bufferSize, irBuffer, 1000.0, 2000.0));
  Serial.print("waterMagnitude: "); Serial.println(goertzel(bufferSize, waterBuffer, 1000.0, 2000.0));
  Serial.print("Temperature: "); Serial.println(readTemperature());
}

void serialCommandParser() {
  static int parserState = 0; // Internal state for the parser function
  static int flagChoice = -1; // Store flag choice across calls

  switch (parserState) {
    case 0:
      // Print the current flag states
      Serial.println("=== Current Flag States ===");
      Serial.print("Current air temperature : "); Serial.println(readTemperature()); // temperature is negative and get "colder" when heating probe
      Serial.print("1. onMachine: "); Serial.println(onMachine);
      Serial.print("2. irDetected: "); Serial.println(irDetected);
      Serial.print("3. backSwitchState: "); Serial.println(backSwitchState);
      Serial.print("4. frontSwitchState: "); Serial.println(frontSwitchState);
      Serial.print("5. scoopMotorState: "); Serial.println(scoopMotorState);
      Serial.print("6. compressorState: "); Serial.println(compressorState);
      Serial.print("7. thawValveState: "); Serial.println(thawValveState);
      Serial.print("8. waterPumpState: "); Serial.println(waterPumpState);
      Serial.print("9. fanState: "); Serial.println(fanState);
      Serial.print("10. ledAddWaterState: "); Serial.println(ledAddWaterState);
      Serial.print("11. waterDetected: "); Serial.println(waterDetected);
      Serial.print("12. ledSizeSState: "); Serial.println(ledSizeSState);

      // Prompt user to modify a flag
      Serial.println("Enter the number of the flag you wish to modify (1-11), or 0 to exit:");
      parserState = 1;  // Move to the state where we wait for flag choice
      return;           // Exit and wait for next call

    case 1:
      // Wait for user to provide the flag choice
      if (Serial.available() > 0) {
        flagChoice = Serial.parseInt();
        
        // If choice is 0, exit the parser
        if (flagChoice == 0) {
          Serial.println("Exiting flag modification.");
          parserState = 0;  // Reset the state for next time
          return;
        }

        // Validate the input and move to the next state
        if (flagChoice >= 1 && flagChoice <= 11) {
          Serial.println("Enter new value (0 for false, 1 for true):");
          parserState = 2;  // Move to the state where we wait for the new value
        } else {
          Serial.println("Invalid choice. Please enter a number between 1 and 11.");
          parserState = 0;  // Reset to start state
        }
      }
      return;  // Return until user provides input

    case 2:
      // Wait for user to provide the new flag value
      if (Serial.available() > 0) {
        int newValue = Serial.parseInt();
        bool newFlagValue = (newValue != 0); // Set to true if non-zero, false otherwise

        // Modify the corresponding flag based on user input
        switch (flagChoice) {
          case 1: onMachine = newFlagValue; break;
          case 2: irDetected = newFlagValue; break;
          case 3: backSwitchState = newFlagValue; break;
          case 4: frontSwitchState = newFlagValue; break;
          case 5: scoopMotorState = newFlagValue; break;
          case 6: compressorState = newFlagValue; break;
          case 7: thawValveState = newFlagValue; break;
          case 8: waterPumpState = newFlagValue; break;
          case 9: fanState = newFlagValue; break;
          case 10: ledAddWaterState = newFlagValue; break;
          case 11: waterDetected = newFlagValue; break;
          case 12: ledSizeSState = newFlagValue; break;
          default: break;
        }

        // Print the updated state and reset parser for the next round
        Serial.println("Flag updated successfully.");
        parserState = 0;  // Reset to the initial state
      }
      return;  // Return until user provides input
  }
}

void setup() {
  // Output pins
  pinMode(thawValvePin, OUTPUT);
  pinMode(scoopMotorPin, OUTPUT);
  pinMode(waterPumpPin, OUTPUT);
  pinMode(fanPin, OUTPUT);
  pinMode(compressorPin, OUTPUT);
  pinMode(signalSquarePin, OUTPUT);
  pinMode(ledOnOffPin, OUTPUT);

  for (int i = 0; i < 3; i++) {
      pinMode(ledPins[i], OUTPUT);
  }
  Serial.begin(9600);  // Initialize serial communication at 9600 bits per second
  Serial.println("Serial log started!");  // Print a message when starting
}

void loop() {
  static unsigned long lastCallback2kHz = 0;
  static unsigned long lastCallback100Hz = 0;
  static unsigned long lastCallback10Hz = 0;
  static unsigned long lastCallback1Hz = 0;

  // Interval periods in microseconds for each frequency
  const unsigned long interval2kHz = 500;  // 2000 Hz = 500 microseconds
  const unsigned long interval100Hz = 10000; // 100 Hz = 10,000 microseconds
  const unsigned long interval10Hz = 100000; // 10 Hz = 100,000 microseconds
  const unsigned long interval1Hz  = 1000000; // 1Hz = 1'000'000 microseconds

  unsigned long currentTime = micros();

  static int toggleCounter = 0;
  // Check for 2000 Hz callback
  if (currentTime - lastCallback2kHz >= interval2kHz) {
    lastCallback2kHz = currentTime;
    sampleIRandWater(); // Sample the current value of the square signal
    toggleCounter++;  // Increment the counter
    // Toggle the pin every other time the code block runs (i.e., divide by 2)
    if (toggleCounter >= 2) {
        int state = digitalRead(signalSquarePin); // Read the current state of the pin
        digitalWrite(signalSquarePin, !state);    // Toggle the pin to make a 1 kHz square wave
        
        toggleCounter = 0;  // Reset the counter after toggling
    }
  }

  // Check for 100 Hz callback
  if (currentTime - lastCallback100Hz >= interval100Hz) {
    lastCallback100Hz = currentTime;
    updateLEDs();
    sampleButtons();
  }

  // Check for 10 Hz callback
  if (currentTime - lastCallback10Hz >= interval10Hz) {
    lastCallback10Hz = currentTime;
    refreshState();
    //serialCommandParser();
    processMachineState();
  }

  // Check for 1 Hz callback
  if (currentTime - lastCallback1Hz >= interval1Hz) {
    lastCallback1Hz = currentTime;
    //pwmManualSerialCalibration();
  }
}
