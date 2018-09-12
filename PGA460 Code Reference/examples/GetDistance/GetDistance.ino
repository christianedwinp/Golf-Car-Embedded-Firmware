/*------------------------------------------------- GetDistance -----
  PROJECT:     PGA460 UART, TCI, OWU, & SPI Ultrasonic Time-of-Flight
  DESCRIPTION: Transmits and receives ultrasonic echo data to measure
              time-of-flight distance, width, and/or amplitude.1
  CREATED:     22 February 2017
  UPDATED:     11 December 2017
  REVISION:    C
  AUTHOR:      A. Whitehead
  NOTES:       This example code is in the public domain.
  -------------------------------------------------------------------*/
#include <PGA460_USSC.h>

/*------------------------------------------------- run mode -----
  |  userInputMode
  |
  |  Purpose:  This code can be operated in two run modes:
  |    • userInputMode = allows the user to configure the device using
  |      the COM serial terminal. Resulting data is printed in the
  |      terminal view. Recommended run mode.
  |    • standAloneMode = waits for the user to press the
  |      LaucnhPad's PUSH2 button to automatically execute the
  |      initializaiton routine, and begin the burst-and-listen captures.
  |      The device is configured based on the hard-coded global
  |      variables. LEDs are illumanted to represent approximate
  |      object distance. Results also printed on serial COM terminal.
  |      Comment out the run mode to use standAloneMode.
  -------------------------------------------------------------------*/
#define userInputMode

/*------------------------------------------------- Global Variables -----
  |  Global Variables
  |
  |  Purpose:  Variables shared throughout the GetDistance sketch for
  |    both userInput and standAlone modes. Hard-code these values to
  |    the desired conditions when automatically updating the device
  |    in standAlone mode.
  -------------------------------------------------------------------*/

// Configuration variables
byte commMode = 0;            // Communication mode: 0=UART, 1=TCI, 2=OneWireUART
byte fixedThr = 1;            // set P1 and P2 thresholds to 0=%25, 1=50%, or 2=75% of max; initial minDistLim (i.e. 20cm) ignored
byte xdcr = 1;                // set PGA460 to recommended settings for 0=Murata MA58MF14-7N, 1=Murata MA40H1S-R
byte agrTVG = 2;              // set TVG's analog front end gain range to 0=32-64dB, 1=46-78dB, 2=52-84dB, or 3=58-90dB
byte fixedTVG = 1;            // set fixed TVG level at 0=%25, 1=50%, or 1=75% of max
byte runDiag = 0;             // run system diagnostics and temp/noise level before looping burst+listen command
byte edd = 0;                 // echo data dump of preset 1, 2, or neither
byte burn = 0;                // trigger EE_CNTRL to burn and program user EEPROM memory
byte cdMultiplier = 1;        // multiplier for command cycle delay
byte numOfObj = 1;            // number of object to detect set to 1-8
byte uartAddrUpdate = 0;      // PGA460 UART address to interface to; default is 0, possible address 0-7
bool objectDetected = false;  // object detected flag to break burst+listen cycle when true
bool demoMode = false;        // only true when running UART/OWU multi device demo mode
bool alwaysLong = false;      // always run preset 2, regardless of preset 1 result (hard-coded only)
double minDistLim = 0.1;      // minimum distance as limited by ringing decay of single transducer and threshold masking
uint16_t commandDelay = 0;    // Delay between each P1 and Preset 2 command
uint32_t baudRate = 9600;     // UART baud rate: 9600, 19200, 38400, 57600, 74800, 115200

//PUSH BUTTON used for standAlone mode
const int buttonPin = PUSH2;  // the number of the pushbutton pin
int buttonState = 0;          // variable for reading the pushbutton status

// Result variables
double distance = 0;          // one-way object distance in meters
double width = 0;             // object width in microseconds
double peak = 0;              // object peak in 8-bit
double diagnostics = 0;       // diagnostic selector
byte echoDataDumpElement = 0; // echo data dump element 0 to 127
String interruptString = "";  // a string to hold incoming data
boolean stringComplete = false; // whether the string is complete

// PGA460_USSC library class
pga460 ussc;

/*------------------------------------------------- setup -----
  |  function Setup
  |
  |  Purpose: (see funciton initPGA460 for details)
  -------------------------------------------------------------------*/
void setup() {                // put your setup code here, to run once
  initPGA460();
}

/*------------------------------------------------- initPGA460 -----
  |  function initPGA460
  |
  |  Purpose: One-time setup of PGA460-Q1 EVM hardware and software
  |      in the following steps:
  |    1) Configure the master to operate in UART, TCI, or OWU
  |      communication mode.
  |    2) Confgiure the EVM for compatiblity based on the selected
  |      communicaton mode.
  |    3) Option to update user EEPROM and threhsold registers with
  |      pre-defined values.
  |    4) Option to burn the EEPROM settings (not required unless
  |      values are to be preserved after power cycling device).
  |    5) Option to report echo data dump and/or system diagnostics.
  |
  |  In userInput mode, the user is prompted to enter values through
  |   the Serial COM terminal to configure the device.
  |
  |  In standAlone mode, the user must hard-code the configuration
  |   variables in the globals section for the device to
  |   auto-configure in the background.
  -------------------------------------------------------------------*/
void initPGA460() {

#ifdef userInputMode
  int inByte = 0;         // incoming serial byte

  Serial.begin(baudRate); // initialize COM UART serial channel
  delay(1000);
  Serial.println("PGA460-Q1 EVM UART/TCI/OWU/SPI Energia Demo for Ultrasonic Time-of-Flight");
  Serial.println("-------------------------------------------------------------------------");
  Serial.println("Instructions: Configure the EVM by entering a byte value between 0-9 or 'x' per request.");
  Serial.println("--- Input can be entered as a single string to auto-increment/fill each request. E.g. 0011211000510");
  Serial.println("--- To skip the COM setup at any point, and use the hard-coded values from thereon, enter a value of 's'.");
  Serial.println("--- To reset the program, and re-prompt for user input, enter a value of 'q'.");
  Serial.println("--- To pause/resume the program when running, enter a value of 'p'.");

  int numInputs = 13;
  for (int i = 0; i < numInputs; i++)
  {
    switch (i)
    {
      case 0: Serial.print("1. Communication Mode: 0=UART, 1=TCI, 2=OneWireUART, 3=SPI ... "); break;
      case 1:
        if (commMode == 0 || commMode == 2) // UART, OWU, or TCI
        {
          Serial.print("2. UART kBaud: 0=9.6, 1=19.2, 2=38.4, 3=57.6, 4=74.8, 5=115.2 ... "); break;
        }
        else if (commMode == 3) // SPI
        {
          Serial.print("2. Clock Divider (MHz): 0=16/1, 1=16/2, 2=16/4, 3=16/8, 4=16/16, 5=16/32, 6=16/64, 7=16/128 ... "); break;
        }
      case 2: Serial.print("3. P1 and P2 Thresholds: 0=%25, 1=50%, or 2=75% of max ... "); break;
      case 3: Serial.print("4. Transducer Settings: 0=Murata MA58MF14-7N, 1=Murata MA40H1S-R, x=Skip ... "); break;
      case 4: Serial.print("5. TVG Range: 0=32-64dB, 1=46-78dB, 2=52-84dB, or 3=58-90dB, x=Skip ... "); break;
      case 5: Serial.print("6. Fixed TVG Level: 0=%25, 1=50%, or 2=75% of max, x=Skip ... "); break;
      case 6: Serial.print("7. Minimum Distance = 0.1m * BYTE ... "); break;
      case 7: Serial.print("8. Run System Diagnostics?: 0=No, 1=Yes ... "); break;
      case 8: Serial.print("9. Echo Data Dump: 0=None, 1=P1BL, 2=P2BL, 3=P1LO, 4=P2LO,... "); break;
      case 9: Serial.print("10. Burn/Save User EEPROM?: 0=No, 1=Yes ... "); break;
      case 10: Serial.print("11. Command Cycle Delay: 10ms * BYTE ... "); break;
      case 11: Serial.print("12. Number of Objects to Detect (1-8) = BYTE ... "); break;
      case 12: Serial.print("13. USART Address of PGA460 (0-7) = BYTE ... "); break;
    }

    // only accept input as valid if 0-9, q, s, or x; otherwise, wait until valid input
    bool validInput = false;
    while (validInput == false)
    {
      while (Serial.available() == 0) {}
      inByte = Serial.read();
      if (inByte == 48 || inByte == 49 || inByte == 50 || inByte == 51 ||
          inByte == 52 || inByte == 53 || inByte == 54 || inByte == 55 ||
          inByte == 56 || inByte == 57 || inByte == 113 || inByte == 115 || inByte == 120)
      {
        validInput = true; // valid input, break while loop
      }
      else
      {
        // not a valid value
      }
    }

    //subtract 48d since ASCII '0' is 48d as a printable character
    inByte = inByte - 48;
    if (inByte != 115 - 48 && inByte != 113 - 48) // if input is neither 's' or 'q'
    {
      delay(300);
      Serial.println(inByte);
      switch (i)
      {
        case 0: commMode = inByte; break;
        case 1:
          if (commMode == 0 || commMode == 2) // UART, OWU, or TCI
          {
            switch (inByte)
            {
              case 0: baudRate = 9600; break;
              case 1: baudRate = 19200; break;
              case 2: baudRate = 38400; break;
              case 3: baudRate = 57600; break;
              case 4: baudRate = 74800; break;
              case 5: baudRate = 115200; break;
              default: baudRate = 9600; break;
            }
          }
          else if (commMode == 3) // SPI
          {
            switch (inByte)
            {
              case 0: baudRate = 1; break;
              case 1: baudRate = 2; break;
              case 2: baudRate = 4; break;
              case 3: baudRate = 8; break;
              case 4: baudRate = 16; break;
              case 5: baudRate = 32; break;
              case 6: baudRate = 64; break;
              case 7: baudRate = 128; break;
              default: baudRate = 16; break;
            }
          }
          else
          {
            baudRate = inByte;
          }
          break;
        case 2: fixedThr = inByte; break;
        case 3: xdcr = inByte; break;
        case 4: agrTVG = inByte; break;
        case 5: fixedTVG = inByte; break;
        case 6: minDistLim = inByte * 0.1; break;
        case 7: runDiag = inByte; break;
        case 8: edd = inByte; break;
        case 9: burn = inByte; break;
        case 10: cdMultiplier = inByte; break;
        case 11: numOfObj = inByte; break;
        case 12: uartAddrUpdate = inByte; break;
        default: break;
      }
    }
    else if (inByte == 113 - 48) //  'q'
    {
      initPGA460(); // restart initializaiton routine
    }
    else //   's'
    {
      i = numInputs - 1; // force for-loop to break
      Serial.println("");
    }
  }

  Serial.println("Configuring the PGA460 with the selected settings. Wait...");
  delay(300);

#else  // standAlone mode  
  pinMode(buttonPin, INPUT_PULLUP);         // initialize the pushbutton pin as an input
  while (digitalRead(buttonPin) == HIGH) {} // wait until user presses PUSH2 button to run standalone mode
#endif

  /*------------------------------------------------- userInput & standAlone mode initialization -----
    Configure the EVM in the following order:
    1) Select PGA460 interface, device baud, and COM terminal baud up to 115.2k for targeted address.
    2) Bulk write all threshold values to clear the THR_CRC_ERR.
    3) Bulk write user EEPROM with pre-define values in PGA460_USSC.c.
    4) Update analog front end gain range, and bulk write TVG.
    5) Run system diagnostics for frequency, decay, temperature, and noise measurements
    6) Program (burn) EEPROM memory to save user EEPROM values
    7) Run a preset 1 or 2 burst and/or listen command to capture the echo data dump

    if the input is 'x' (72d), then skip that configuration
    -------------------------------------------------------------------*/
  // -+-+-+-+-+-+-+-+-+-+- 1 : interface setup   -+-+-+-+-+-+-+-+-+-+- //
  ussc.initBoostXLPGA460(commMode, baudRate, uartAddrUpdate);

  // -+-+-+-+-+-+-+-+-+-+- 2 : bulk threshold write   -+-+-+-+-+-+-+-+-+-+- //
  if (fixedThr != 72) {
    ussc.initThresholds(fixedThr);
  }
  // -+-+-+-+-+-+-+-+-+-+- 3 : bulk user EEPROM write   -+-+-+-+-+-+-+-+-+-+- //
  if (xdcr != 72) {
    ussc.defaultPGA460(xdcr);
  }
  // -+-+-+-+-+-+-+-+-+-+- 4 : bulk TVG write   -+-+-+-+-+-+-+-+-+-+- //
  if (agrTVG != 72 && fixedTVG != 72) {
    ussc.initTVG(agrTVG, fixedTVG);
  }
  // -+-+-+-+-+-+-+-+-+-+- 5 : run system diagnostics   -+-+-+-+-+-+-+-+-+-+- //
  if (runDiag == 1)
  {
    diagnostics = ussc.runDiagnostics(1, 0);      // run and capture system diagnostics, and print freq diag result
    Serial.print("System Diagnostics - Frequency (kHz): "); Serial.println(diagnostics);
    diagnostics = ussc.runDiagnostics(0, 1);      // do not re-run system diagnostic, but print decay diag result
    Serial.print("System Diagnostics - Decay Period (us): "); Serial.println(diagnostics);
    diagnostics = ussc.runDiagnostics(0, 2);      // do not re-run system diagnostic, but print temperature measurement
    Serial.print("System Diagnostics - Die Temperature (C): "); Serial.println(diagnostics);
    diagnostics = ussc.runDiagnostics(0, 3);      // do not re-run system diagnostic, but print noise level measurement
    Serial.print("System Diagnostics - Noise Level: "); Serial.println(diagnostics);
  }
  // -+-+-+-+-+-+-+-+-+-+- 6 : burn EEPROM   -+-+-+-+-+-+-+-+-+-+- //
  if (burn == 1)
  {
    byte burnStat = ussc.burnEEPROM();
    if (burnStat == true) {
      Serial.println("EEPROM programmed successfully.");
    }
    else {
      Serial.println("EEPROM program failed.");
    }
  }
  // -+-+-+-+-+-+-+-+-+-+- 7 : capture echo data dump   -+-+-+-+-+-+-+-+-+-+- //
  if (edd != 0)                                   // run or skip echo data dump
  {
    Serial.println("Retrieving echo data dump profile. Wait...");
    ussc.runEchoDataDump(edd - 1);                // run preset 1 or 2 burst and/or listen command
    for (int n = 0; n < 128; n++)                 // get all echo data dump results
    {
      echoDataDumpElement = ussc.pullEchoDataDump(n);
      Serial.print(echoDataDumpElement);
      Serial.print(",");
    }
    Serial.println("");
  }
  // -+-+-+-+-+-+-+-+-+-+-  others   -+-+-+-+-+-+-+-+-+-+- //
  commandDelay = 10 * cdMultiplier;                   // command cycle delay result in ms
  if (numOfObj == 0 || numOfObj > 8) {
    numOfObj = 1;  // sets number of objects to detect to 1 if invalid input
  }

}

/*------------------------------------------------- main loop -----
  |  main loop  GetDistance
  |
  |   The PGA460 is initiated with a Preset 1 Burst-and-Listen
  |     Time-of-Flight measurement. Preset 1 is ideally configured for
  |     short-range measurements (sub-1m range) when using the pre-defined
  |     user EEPROM configurations.
  |
  |   If no object is detected, the PGA460 will then be issued a
  |     Preset 2 Burst-and-Listen Time-of-Flight measurement.
  |     Preset 2 is configured for long-range measurements (beyond
  |     1m range).
  |
  |   Depending on the resulting distance, the diagnostics LEDs will
  |     illuminate to represent a short, mid, or long range value.
  |
  |   In userInput mode, the distance, width, and/or amplitude value
  |     of each object is serial printed on the COM terminal.
  |
  |   In standAlone mode, only distance can be represented visually
  |     on the LEDs. The resulting values are still serial printed
  |     on a COM terminal for debug, and to view the numerical values
  |     of the data captured.
  |
  -------------------------------------------------------------------*/

void loop() {                 // put your main code here, to run repeatedly
  // -+-+-+-+-+-+-+-+-+-+-  PRESET 1 (SHORT RANGE) MEASUREMENT   -+-+-+-+-+-+-+-+-+-+- //
  objectDetected = false;                       // Initialize object detected flag to false
  ussc.ultrasonicCmd(0, numOfObj);              // run preset 1 (short distance) burst+listen for 1 object
  ussc.pullUltrasonicMeasResult(demoMode);      // Pull Ultrasonic Measurement Result
  for (byte i = 0; i < numOfObj; i++)
  {
    // Log uUltrasonic Measurement Result: Obj1: 0=Distance(m), 1=Width, 2=Amplitude; Obj2: 3=Distance(m), 4=Width, 5=Amplitude; etc.;
    distance = ussc.printUltrasonicMeasResult(0 + (i * 3));
    //width = ussc.printUltrasonicMeasResult(1+(i*3));
    //peak = ussc.printUltrasonicMeasResult(2+(i*3));

    delay(commandDelay);

    if (distance > minDistLim && distance < 11.2)  // turn on DS1_LED if object is above minDistLim
    {
      ussc.toggleLEDs(HIGH, LOW, LOW);
      Serial.print("P1 Obj"); Serial.print(i + 1); Serial.print(" Distance (m): "); Serial.println(distance);
      objectDetected = true;
    }
  }

  // -+-+-+-+-+-+-+-+-+-+-  PRESET 2 (LONG RANGE) MEASUREMENT   -+-+-+-+-+-+-+-+-+-+- //
  if (objectDetected == false || alwaysLong == true)                      // If no preset 1 (short distance) measurement result, switch to Preset 2 B+L command
  {
    ussc.ultrasonicCmd(1, numOfObj);               // run preset 2 (long distance) burst+listen for 1 object
    ussc.pullUltrasonicMeasResult(demoMode);                // Get Ultrasonic Measurement Result
    for (byte i = 0; i < numOfObj; i++)
    {
      distance = ussc.printUltrasonicMeasResult(0 + (i * 3)); // Print Ultrasonic Measurement Result i.e. Obj1: 0=Distance(m), 1=Width, 2=Amplitude; Obj2: 3=Distance(m), 4=Width, 5=Amplitude;
      //width = ussc.printUltrasonicMeasResult(1+(i*3));
      //peak = ussc.printUltrasonicMeasResult(2+(i*3));

      delay(commandDelay);

      if (distance < 1 && distance > minDistLim)    // turn on DS1_LED and F_DIAG_LED if object is within 1m
      {
        ussc.toggleLEDs(HIGH, LOW, LOW);
        Serial.print("P2 Obj"); Serial.print(i + 1); Serial.print(" Distance (m): "); Serial.println(distance);
        objectDetected = true;
      }
      else if (distance < 3 && distance >= 1)      // turn on DS1_LED and F_DIAG_LED if object is within 3m
      {
        ussc.toggleLEDs(HIGH, HIGH, LOW);
        Serial.print("P2 Obj"); Serial.print(i + 1); Serial.print(" Distance (m): "); Serial.println(distance);
        objectDetected = true;
      }
      else if (distance >= 3 && distance < 11.2)     // turn on DS1_LED, F_DIAG_LED, and V_DIAG_LED if object is greater than 3m
      {
        ussc.toggleLEDs(HIGH, HIGH, HIGH);
        Serial.print("P2 Obj"); Serial.print(i + 1); Serial.print(" Distance (m): "); Serial.println(distance);
        objectDetected = true;
      }
      else if (distance == 0)                         // turn off all LEDs if no object detected
      {
        ussc.toggleLEDs(LOW, LOW, LOW);
        //Serial.print("Error reading measurement results..."); //Serial.println(distance);
      }
      else //(distance > 11.2 && distance < minDistLim)         // turn off all LEDs if no object detected or below minimum distance limit
      {
        if (i == numOfObj - 1 && objectDetected == false)
        {
          ussc.toggleLEDs(LOW, LOW, LOW);
          Serial.println("No object...");
        }
      }
    }
  }

  // -+-+-+-+-+-+-+-+-+-+-  STATUS   -+-+-+-+-+-+-+-+-+-+- //
  digitalWrite(GREEN_LED, !digitalRead(GREEN_LED));   //toggle green LED after each sequence
  digitalWrite(RED_LED, !digitalRead(GREEN_LED));     //toggle red LED after each sequence
}

// -+-+-+-+-+-+-+-+-+-+-  SERIAL MONITORING   -+-+-+-+-+-+-+-+-+-+- //
/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read(); // get the new byte

    // if the incoming character is a 'q', set a flag, stop the main loop, and re-run initialization
    if (inChar == 'q') {
      stringComplete = true;
      initPGA460();
    }

    // if the incoming character is a 'p', set a flag, pause the loop, and resume loop upon receiving another 'p' character
    else if (inChar == 'p')
    {
      Serial.println("PAUSE");
      stringComplete = false;
      while (stringComplete == false)
      {
        while (Serial.available())
        {
          inChar = (char)Serial.read(); // get the new byte
          if (inChar == 'p')
          {
            stringComplete = true;
          }
          if (inChar == 'q')
          {
            stringComplete = true;
            initPGA460();
          }
          delay(1000);
        }
      }
      stringComplete = false;
      Serial.println("");
    }

    else {} //do nothing
  }
}


