/*------------------------------------------------- Autoset Threshold -----
 PROJECT:     PGA460 Autoset Threshold
 DESCRIPTION: Collects the echo data dump data for a no-object burst/listen
              commmand, and automatically wraps the threshold times and levels
              based on downsampled maximums.
 CREATED:     26 March 2018
 UPDATED:     N/A
 REVISION:    Initial
 AUTHOR:      A. Whitehead
 NOTES:       This example code is in the public domain.
*-------------------------------------------------------------------*/
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
*-------------------------------------------------------------------*/
#define userInputMode         

/*------------------------------------------------- Global Variables -----
|  Global Variables
|
|  Purpose:  Variables shared throughout the GetDistance sketch for 
|    both userInput and standAlone modes. Hard-code these values to
|    the desired conditions when automatically updating the device
|    in standAlone mode.
*-------------------------------------------------------------------*/

// Configuration variables
  byte commMode = 0;            // Communication mode: 0=UART, 1=TCI, 2=OneWireUART
  byte fixedThr = 1;            // set P1 and P2 thresholds to 0=%25, 1=50%, or 2=75% of max; initial minDistLim (i.e. 20cm) ignored
  byte xdcr = 1;                // set PGA460 to recommended settings for 0=Murata MA58MF14-7N, 1=Murata MA40H1S-R
  byte agrTVG = 2;              // set TVG's analog front end gain range to 0=32-64dB, 1=46-78dB, 2=52-84dB, or 3=58-90dB
  byte fixedTVG = 1;            // set fixed TVG level at 0=%25, 1=50%, or 1=75% of max
  byte burn = 0;                // trigger EE_CNTRL to burn and program user EEPROM memory
  byte numOfObj = 1;            // number of object to detect set to 1-8
  byte uartAddrUpdate = 0;      // PGA460 UART address to interface to; default is 0, possible address 0-7
  bool objectDetected = false;  // object detected flag to break burst+listen cycle when true
  bool demoMode = false;        // only true when running UART/OWU multi device demo mode
  double minDistLim = 0.1;      // minimum distance as limited by ringing decay of single transducer and threshold masking
  uint16_t commandDelay = 0;    // Delay between each P1 and Preset 2 command
  uint32_t baudRate = 9600;     // UART baud rate: 9600, 19200, 38400, 57600, 74800, 115200
  byte atPc = 0;                // preset 1 or 2 burst and/or listen command to run
  byte atNf = 0;                // margin between maximum downsampled noise value and the threshold level in intervals of 8
  byte atTi = 0;                // spacing between each threshold time as an index (refer to datasheet for microsecond equivalent). To use the existing threshold times, enter a value of '16'
  byte atUp = 0;                // automatically set threshold levels up to this threshold point (maximum is 12). Remaining levels will not change.
  byte copyThr = 2;             // 0 = save autoset threshold to EEPROM; 1 = load initial threshold from EEPROM; 2 = both

//PUSH BUTTON used for standAlone mode
  const int buttonPin = PUSH2;  // the number of the pushbutton pin
  int buttonState = 0;          // variable for reading the pushbutton status

// Result variables
  String interruptString = "";  // a string to hold incoming data
  boolean stringComplete = false; // whether the string is complete

// PGA460_USSC library class
  pga460 ussc;

/*------------------------------------------------- setup -----
|  function Setup
|
|  Purpose: (see funciton initPGA460 for details)
*-------------------------------------------------------------------*/
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
|    3) Configure auto-threshold settings
|  
|  In userInput mode, the user is prompted to enter values through 
|   the Serial COM terminal to configure the device.
|
|  In standAlone mode, the user must hard-code the configuration 
|   variables in the globals section for the device to 
|   auto-configure in the background.
*-------------------------------------------------------------------*/
void initPGA460() {
  
    #ifdef userInputMode
    int inByte = 0;         // incoming serial byte    
    
    Serial.begin(baudRate); // initialize COM UART serial channel
    delay(1000);
    Serial.println("                  PGA460-Q1 EVM Autoset Threshold Demo                   ");
    Serial.println("-------------------------------------------------------------------------");
    Serial.println("Instructions: Configure the EVM by entering a byte value between 0-9 or 'x' (next) per request.");
    Serial.println("--- Input can be entered as a single string to auto-increment/fill each request. E.g. 00023x10220");
    Serial.println("--- To skip the COM setup at any point, and use the hard-coded values from thereon, enter a value of 's'.");
    Serial.println("--- To reset the program, and re-prompt for user input, enter a value of 'q'.");
    Serial.println("--- To pause/resume the program when running, enter a value of 'p'.");
    
    int numInputs = 11;
    for (int i=0; i<numInputs; i++)
    {
      switch(i)
      {
        case 0: Serial.print("1. Communication Mode: 0=UART, 1=TCI, 2=OneWireUART, 3=SPI ... "); break;     
        case 1: 
          if(commMode == 0 || commMode == 2) // UART, OWU, or TCI
          {
            Serial.print("2. UART kBaud: 0=9.6, 1=19.2, 2=38.4, 3=57.6, 4=74.8, 5=115.2 ... "); break;
          }
          else if (commMode == 3) // SPI
          {
            Serial.print("2. Clock Divider (MHz): 0=16/1, 1=16/2, 2=16/4, 3=16/8, 4=16/16, 5=16/32, 6=16/64, 7=16/128 ... "); break;
          }                          
        case 2: Serial.print("3. Autoset Threshold Preset Command: 0=P1BL, 1=P2BL, 2=P1LO, 3=P2LO ..."); break;  //Preset 1 Burst Listen, Preset 2 Burst Listen, Preset 1 Listen Only, Preset 2 Listen Only
        //case 2: Serial.print("3. P1 and P2 Thresholds: 0=%25, 1=50%, or 2=75% of max ... "); break;   
        case 3: Serial.print("4. Autoset Threshold Noise Floor: 8 * BYTE ..."); break;
        case 4: Serial.print("5. Autoset Threshold Time Index: BYTE ..."); break;
        case 5: Serial.print("6. Autoset Threshold Up to Point (12 Total): 0=None, BYTE=MaxPoint, or x=All ..."); break;       
        case 6: Serial.print("7. Transducer Settings: 0=Murata MA58MF14-7N, 1=Murata MA40H1S-R, x=Skip ... "); break;  
        case 7: Serial.print("8. TVG Range: 0=32-64dB, 1=46-78dB, 2=52-84dB, or 3=58-90dB, x=Skip ... "); break;   
        case 8: Serial.print("9. Fixed TVG Level: 0=%25, 1=50%, or 2=75% of max, x=Skip ... "); break;     
        case 9: Serial.print("10. EEPROM Load and/or Copy Threshold?: 0=Copy, 1=Load, 2=Load&Copy ... "); break;
        case 10: Serial.print("11. USART Address of PGA460 (0-7) = BYTE ... "); break;
      }

      // only accept input as valid if 0-9, q, s, or x; otherwise, wait until valid input
      bool validInput = false;
      while (validInput == false)
      {
        while (Serial.available() == 0){}
        inByte = Serial.read();
         if (inByte==48 || inByte==49 || inByte==50 || inByte==51 || inByte==52 || inByte==53 || inByte==54 || inByte==55 || inByte==56 || inByte==57 || inByte==113 || inByte==115 || inByte==120)
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
      if (inByte != 115-48 && inByte != 113-48) // if input is neither 's' or 'q'
      {        
        delay(300);
        Serial.println(inByte);
        switch(i)
        {
          case 0: commMode = inByte; break;     
          case 1: 
            if(commMode == 0 || commMode == 2) // UART, OWU, or TCI
            {
              switch(inByte)
              {
                case 0: baudRate=9600; break;
                case 1: baudRate=19200; break; 
                case 2: baudRate=38400; break; 
                case 3: baudRate=57600; break; 
                case 4: baudRate=74800; break;
                case 5: baudRate=115200; break; 
                default: baudRate=9600; break;   
              }
            }
            else if (commMode == 3) // SPI
            {
              switch(inByte)
              {
                case 0: baudRate=1; break;
                case 1: baudRate=2; break; 
                case 2: baudRate=4; break; 
                case 3: baudRate=8; break; 
                case 4: baudRate=16; break;
                case 5: baudRate=32; break; 
                case 6: baudRate=64; break;
                case 7: baudRate=128; break;
                default: baudRate=16; break; 
              }
            } 
            else
            {
              baudRate = inByte; 
            }
            break;
          case 2: atPc = inByte; break; //0 = Preset 1 Burst Listen
          case 3: atNf = inByte; break; //8 * byte  
          case 4: atTi = inByte; break; //Byte  
          case 5: atUp = inByte; break; //x=All    
          case 6: xdcr = inByte; break; //custom transducer or x = skip
          case 7: agrTVG = inByte; break; //tvg range : 2->52-84db  
          case 8: fixedTVG = inByte; break; //custom tvg or x = skip
          case 9: copyThr = inByte; break; //0=Copy, 1=Load, 2=Load&Copy
          case 10: uartAddrUpdate = inByte; break;
          default: break;
        }
      }
      else if(inByte == 113-48) //  'q'
      {
        initPGA460(); // restart initializaiton routine
      }
      else //   's'
      {
        i=numInputs-1; // force for-loop to break
        Serial.println("");        
      }      
    }    
     
    Serial.println("Configuring the PGA460 with the selected settings. Wait...");
    delay(300);

  #else  // standAlone mode  
    pinMode(buttonPin, INPUT_PULLUP);         // initialize the pushbutton pin as an input
    while (digitalRead(buttonPin) == HIGH){}  // wait until user presses PUSH2 button to run standalone mode
  #endif

/*------------------------------------------------- userInput & standAlone mode initialization -----
  Configure the EVM in the following order:
  1) Select PGA460 interface, device baud, and COM terminal baud up to 115.2k for targeted address.
  2) If a threshold load command is issued, then the values from USER_DATA1-16 are copied to the designated
    preset threshold space in SRAM. Otherwise, a bulk write all threshold command will clear the THR_CRC_ERR. 
  3) Bulk write user EEPROM with pre-define values in PGA460_USSC.c. 
  4) Update analog front end gain range, and bulk write TVG.
  
  if the input is 'x' (72d), then skip that configuration
*-------------------------------------------------------------------*/
  // -+-+-+-+-+-+-+-+-+-+- 1 : interface setup   -+-+-+-+-+-+-+-+-+-+- //
    ussc.initBoostXLPGA460(commMode, baudRate, uartAddrUpdate); 
    
  // -+-+-+-+-+-+-+-+-+-+- 2 : bulk threshold write   -+-+-+-+-+-+-+-+-+-+- //
    if (copyThr == 1 || copyThr == 2)
    {
      Serial.println("Loading threshold map from EEPROM memory space...");
      ussc.eepromThreshold(atPc+1,true);
    }
    else
    {
      if (fixedThr != 72){ussc.initThresholds(fixedThr);}
    }
  
  // -+-+-+-+-+-+-+-+-+-+- 3 : bulk user EEPROM write   -+-+-+-+-+-+-+-+-+-+- //
    if (xdcr != 72){ussc.defaultPGA460(xdcr);}
  
  // -+-+-+-+-+-+-+-+-+-+- 4 : bulk TVG write   -+-+-+-+-+-+-+-+-+-+- //
    if (agrTVG != 72 && fixedTVG != 72){ussc.initTVG(agrTVG,fixedTVG);}

  // -+-+-+-+-+-+-+-+-+-+-  others   -+-+-+-+-+-+-+-+-+-+- //
    if (atUp == 72){atUp = 12;}                
}

/*------------------------------------------------- main loop -----
|  main loop  AutosetThreshold
|
|   The autothreshold command forces a preset burst/listen command
|     in echo data dump mode. The echo data dump is then downsampled
|     to capture the maximum peak value in each window partition.
|     This allows the threhsold level to be set with margin against 
|     a known maximum data value.
|
|   The autoset threhsold can then be copied to the USER_DATA1-16
|     memory space in EEPROM. The threhsold values can then be
|     saved to EEPROM if a burn EEPROM command is issued. 
|
|   The resulting autoset threshold values are then printed on the
|     COM terminal port.
|
*-------------------------------------------------------------------*/

void loop() {                 // put your main code here, to run repeatedly

  
  //Serial.println("Ensure no object is in the transducer's field of view. Press PUSH2 to continue...");
  //while (digitalRead(PUSH2) == HIGH) { delay(100); } // wait until user presses PUSH2 button to run standalone mode
  delay(1000);

  // -+-+-+-+-+-+-+-+-+-+-  automatically configure threshold   -+-+-+-+-+-+-+-+-+-+- //
    Serial.println("Automatically configuring the preset's threhsold map...");
    // runs burst and/or listen command in echo data dump mode
    ussc.autoThreshold(atPc,atNf*8,atTi,atUp,1);

          case 2: atPc = inByte; break; //0 = Preset 1 Burst Listen
          case 3: atNf = inByte; break; //8 * byte  Autoset Threshold Noise Floor: 8 * BYTE ...
          case 4: atTi = inByte; break; //Byte   Threshold Time Index
          case 5: atUp = inByte; break; //x=All    
          case 6: xdcr = inByte; break; //custom transducer or x = skip
          case 7: agrTVG = inByte; break; //tvg range : 2->52-84db  
          case 8: fixedTVG = inByte; break; //custom tvg or x = skip
          case 9: copyThr = inByte; break; //0=Copy, 1=Load, 2=Load&Copy

  // -+-+-+-+-+-+-+-+-+-+-  copy threshold into USER_DATA   -+-+-+-+-+-+-+-+-+-+- //
    // Copy SRAM threhsold to EEPROM, or load EEPROM threshold to SRAM
    if (copyThr == 0 || copyThr == 2)
    {
      Serial.println("Copying threshold map to EEPROM memory space...");
      ussc.eepromThreshold(atPc+1,false);
    }
    /*
    // -+-+-+-+-+-+-+-+-+-+- 6.1 : burn threshold data to USER_DATA EEPROM   -+-+-+-+-+-+-+-+-+-+- //
    if(burn == 1)
    {
      byte burnStat = ussc.burnEEPROM();
      if(burnStat == true){Serial.println("EEPROM programmed successfully.");}
      else{Serial.println("EEPROM program failed.");}
    }
    */
    
  // -+-+-+-+-+-+-+-+-+-+-  print auto threhsold map   -+-+-+-+-+-+-+-+-+-+- //
    byte thrAddr = 0x5F; // beginning of threshold memory space
    if (atPc == 1 || atPc == 3) 
    {
      thrAddr += 16;
    }
    Serial.println("Autoset Threshold Results (HEX): ");
    for (int i = 0; i <16; i++)
    { 
      Serial.print(ussc.registerRead(i + thrAddr),HEX); Serial.print(" ");
    }
    Serial.println("");
  
  // -+-+-+-+-+-+-+-+-+-+-  STATUS   -+-+-+-+-+-+-+-+-+-+- //
    digitalWrite(GREEN_LED, !digitalRead(GREEN_LED));   //toggle green LED after each sequence
    digitalWrite(RED_LED, !digitalRead(GREEN_LED));     //toggle red LED after each sequence
  
  // -+-+-+-+-+-+-+-+-+-+-  restart prompt for user input to rerun   -+-+-+-+-+-+-+-+-+-+- //
    initPGA460();
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
      if (inChar == 'q'){stringComplete = true; initPGA460();}
      
      // if the incoming character is a 'p', set a flag, pause the loop, and resume loop upon receiving another 'p' character
      else if (inChar == 'p')
      {
        Serial.println("PAUSE");
        stringComplete = false; 
        while(stringComplete == false)
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
        stringComplete=false;
        Serial.println("");
      }    
       
      else {} //do nothing
  }
}


