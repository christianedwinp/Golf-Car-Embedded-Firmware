/*------------------------------------------------- BusDemo -----
 PROJECT:     PGA460 UART & OWU Bus Topology Demo for up to 8 Devices
 DESCRIPTION: Prints the distance (m) equivlant for the 
              ultrasonic time-of-flight capture of a single object in 
              UART or OWU mode. Cycles through each device address 
              once per loop.
 CREATED:     17 August 2017
 UPDATED:     01 Decemeber 2017
 REVISION:    A
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
  byte commMode = 2;            // Communication mode: 0=UART, 2=OneWireUART
  byte uartAddrUpdate = 0;      // PGA460 UART address to interface to; default is 0, possible address 0-7
  byte fixedThr = 1;            // set P1 and P2 thresholds to 0=%25, 1=50%, or 2=75% of max; initial minDistLim (i.e. 20cm) ignored
  byte xdcr = 0;                // set PGA460 to recommended settings for 0=Murata MA58MF14-7N, 1=Murata MA40H1S-R
  byte agrTVG = 1;              // set TVG's analog front end gain range to 0=32-64dB, 1=46-78dB, 2=52-84dB, or 3=58-90dB
  byte fixedTVG = 1;            // set fixed TVG level at 0=%25, 1=50%, or 1=75% of max
  byte burn = 0;                // trigger EE_CNTRL to burn and program user EEPROM memory
  byte cdMultiplier = 0;        // multiplier for command cycle delay
  bool demoMode = true;         // only true when running UART/OWU multi device demo mode
  bool broadcastTVG = false;    // serial transmit TVG bulk write broadcast command
  bool broadcastThr = true;     // serial transmit threshold bulk write broadcast command
  bool broadcastEE = false;      // serial transmit user EEPROM bulk write broadcast command
  uint16_t commandDelay = 10;   // Delay between each main loop
  uint32_t baudRate = 9600;     // UART baud rate: 9600, 19200, 38400, 57600, 74800, 115200 

//PUSH BUTTON
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
  
// BusDemo exclusive variables
  byte numDevices = 3;          // number of PGA460 devices connected to the bus
  bool successMeas = false;     // successfully read back ultrasonic measurement results?
  byte demoType = commMode+7;   // 7 = UART, 9 = OWU
  byte owuReady = 0;            // is the device already eeprom progammmed for OWU mode instead of TCI mode? 0=Yes, 1=No_UART-to-OWU_config, 2=No_IO-to-OWU_config

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
|    1) Configure the master to operate in UART or OWU 
|      communication mode.
|    2) Confgiure the EVM for compatiblity based on the selected 
|      communicaton mode.
|    3) Option to update user EEPROM and threhsold registers with 
|      pre-defined values using broadcast commands for all devices
|      on the bus to be updated simulatenously.
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
        Serial.println("                                 PGA460-Q1 8-Sensor UART & OWU Bus Demo                                  ");
        Serial.println("_________________________________________________________________________________________________________");
        Serial.println("Instructions: Configure the EVM by entering a byte value between 0-9 or 'x' per request.");
        Serial.println("--- Input can be entered as a single string to auto-increment/fill each request. E.g. 2011305011100");
        Serial.println("--- To skip the COM setup at any point, and use the hard-coded values from thereon, enter a value of 's'.");
        Serial.println("--- To reset the program, and re-prompt for user input, enter a value of 'q'.");
    
    int numInputs = 13;
    for (int i=0; i<numInputs; i++)
    {
        switch(i)
        {
          case 0: Serial.print("1. Communication Mode: 0=UART, 2=OneWireUART ... "); break;     
          case 1: Serial.print("2. UART kBaud: 0=9.6, 1=19.2, 2=38.4, 3=57.6, 4=74.8, 5=115.2 ... "); break;   
          case 2: Serial.print("3. P1 and P2 Thresholds: 0=%25, 1=50%, or 2=75% of max ... "); break;  
          case 3: Serial.print("4. Transducer Settings: 0=Murata MA58MF14-7N, 1=Murata MA40H1S-R, x=Skip ... "); break;  
          case 4: Serial.print("5. TVG Range: 0=32-64dB, 1=46-78dB, 2=52-84dB, or 3=58-90dB, x=Skip ... "); break;
          case 5: Serial.print("6. Fixed TVG Level: 0=%25, 1=50%, or 2=75% of max, x=Skip ... "); break;     
          case 6: Serial.print("7. Command Cycle Delay: 10ms * BYTE ... "); break;
          case 7: Serial.print("8. Bulk User EEPROM Write: 0=false, 1=true ... "); break;
          case 8: Serial.print("9. Bulk TVG Write: 0=false, 1=true ... "); break;
          case 9: Serial.print("10. Bulk Threshold Write: 0=false, 1=true ... "); break;
          case 10: Serial.print("11. Number of Devices on Bus: 1-8 ... "); break;
          case 11: Serial.print("12. OWU Ready: 0=Yes, 1=No,UART-Config, 2=No,IO-Config ... "); break;
          case 12: Serial.print("13. Burn User EEPROM?: 0=No, 1=Yes ... "); break;
        }
        
        // only accept input as valid if 0-9, s or x; otherwise, wait until valid input
          bool validInput = false;
          while (validInput == false)
          {
            while (Serial.available() == 0){}
            inByte = Serial.read();
             if (inByte==48 || inByte==49 || inByte==50 || inByte==51 ||
            inByte==52 || inByte==53 || inByte==54 || inByte==55 || 
            inByte==56 || inByte==57 || inByte==113 || inByte==115 || inByte==120)
            {
              validInput = true; // valid input, break while loop       
            }
            else
            {
              // not a valid value   
            }
          } 
        
        inByte = inByte - 48;  //subtract 48d since ASCII '0' is 48d printable character
        if (inByte != 115-48 && inByte != 113-48) // if input is neither 's' or 'q'
        {  
          delay(300);
          Serial.println(inByte);
          switch(i)
          {
            case 0: commMode = inByte; 
                    demoType = commMode+7;
                    break; //add seven to indicate demo
            case 1: 
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
            case 2: fixedThr = inByte; break;
            case 3: xdcr = inByte; break;   
            case 4: agrTVG = inByte; break;    
            case 5: fixedTVG = inByte; break;     
            case 6: cdMultiplier = inByte; break;
            case 7: broadcastEE = inByte; break;
            case 8: broadcastTVG = inByte; break;   
            case 9: broadcastThr = inByte; break; 
            case 10: numDevices = inByte; break;     
            case 11: owuReady = inByte; break;
            case 12: burn = inByte; break;       
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
    #else // standAloneMode
    //wait until user presses PUSH2 on the LaunchPad
      pinMode(buttonPin, INPUT_PULLUP); //initialize the pushbutton pin as an input
      while (digitalRead(buttonPin) == HIGH){}
      ussc.initBoostXLPGA460(commMode, baudRate, uartAddrUpdate); //initialize serial channels for OWU communication mode
    #endif
/*------------------------------------------------- userInput & standAlone mode initialization -----
Configure the EVM in the following order:
  1) If either broadcast command is requested, update TVG and 
     threshold values with pre-defined PGA460_USSC.c values.
  2) Setup device interface for UART or OWU mode.
  3) Serial transmit broadcast write command for TVG and/or threhsolds. 
  4) Option to burn the EEPROM settings (not required unless 
  values are to be preserved after power cycling device). 
  
  if the input is 'x' (72d), then skip that configuration
*-------------------------------------------------------------------*/
  // -+-+-+-+-+-+-+-+-+-+- 1 : broadcast setup   -+-+-+-+-+-+-+-+-+-+- //
    if (broadcastThr == true || broadcastTVG == true){ussc.initBoostXLPGA460(6, baudRate, uartAddrUpdate);}
    if(broadcastEE == true){if (xdcr != 72){ussc.defaultPGA460(xdcr);}}
    if(broadcastThr == true){if (fixedThr != 72){ussc.initThresholds(fixedThr);}}
    if(broadcastTVG == true){if (agrTVG != 72 && fixedTVG != 72){ussc.initTVG(agrTVG,fixedTVG);}}
    
  // -+-+-+-+-+-+-+-+-+-+- 2 : interface setup   -+-+-+-+-+-+-+-+-+-+- //
    if (demoType == 9)      // OWU bus demo
    {
      if (owuReady == 2) // devices are not yet configured for OWU, but will be through IO interface selection
      {
        pinMode (15, OUTPUT);   // 15 = TCI_TX
        // transmit IO-Pin Interface Toggle Pattern
          digitalWrite(15, HIGH);
          delay(10);
          
          digitalWrite(15, LOW);
          delayMicroseconds(260);
          digitalWrite(15, HIGH);
          delayMicroseconds(321);
          digitalWrite(15, LOW);
          delayMicroseconds(208);
          digitalWrite(15, HIGH);
          delayMicroseconds(364);
          digitalWrite(15, LOW);
          delayMicroseconds(156);
          digitalWrite(15, HIGH);
          delayMicroseconds(416);
          
          delay (10);

        // assume pattern was received
        owuReady = true;  
      }  

      // set EVM hardware into OWU mode 
      ussc.initBoostXLPGA460(commMode, baudRate, uartAddrUpdate);
      
      if (owuReady == 1) // device are not yet configured for OWU, but will be through UART
      {
        digitalWrite(17, LOW); // 17 = COM_SEL, standard UART channel select
        for(int i = 0; i < numDevices ; i++)
        {
          uartAddrUpdate = i;
          ussc.initBoostXLPGA460(8, baudRate, uartAddrUpdate); //for all devices, use UART to enable OWU mode (if not already enabled)
        }
        digitalWrite(17, HIGH); // 17 = COM_SEL, OWU channel select
      }
 
    }
    else if (demoType == 7) // UART bus demo
    {
      for(int i = 0; i < numDevices ; i++)
      {
        uartAddrUpdate = i;
        ussc.initBoostXLPGA460(0, baudRate, uartAddrUpdate); //for all devices, use standard UART
      }
    }
    else
    {
      // invalid demoType
      return;
    }
    
  // -+-+-+-+-+-+-+-+-+-+- 3 : broadcast user EEPROM, TVG, and/or threshold bulk writes  -+-+-+-+-+-+-+-+-+-+- //
    ussc.broadcast(broadcastEE, broadcastTVG, broadcastThr); //send broadcast commands for bulk userEEPROM, TVG, and thresholds
  // -+-+-+-+-+-+-+-+-+-+-  others   -+-+-+-+-+-+-+-+-+-+- //    
    commandDelay = 10 * cdMultiplier;           // command cycle delay result in ms


  // -+-+-+-+-+-+-+-+-+-+- 4 : burn EEPROM   -+-+-+-+-+-+-+-+-+-+- //
    if(burn == 1)
    {
      bool burnStat = ussc.burnEEPROM();
      if(burnStat == true){Serial.println("EEPROM programmed successfully.");}
      else{Serial.println("EEPROM program failed.");}
    }
}
/*------------------------------------------------- main loop -----
|  main loop  BusDemo
|   
|   The main loop incrementally and serially issues a burst-and-listen
|     command for one-object, followed by an ultrasonic measurement
|     result command to read back the distance (m) equivalent of the
|     time-of-flight value recorded for a detected object.
|
|   If no object is detected, the serial COM terminal prints a value 
|     of 11.24 on the resulting table.
|
|   If the a communicaiton error prevented a successful burst-and-
|     listen command, or the ultrasonic measurement results could.
|     not be read, the serial COM terminal prints a value 
|     of 0.00 on the resulting table.
|
|   In standAlone mode, the resulting values are still serial printed 
|     on a COM terminal for debug, and to view the numerical values 
|     of the data captured.
|
*-------------------------------------------------------------------*/
void loop() {                 // put your main code here, to run repeatedly
  #ifndef userInputMode
  delay(1000);
  Serial.println("               PGA460-Q1 8-Sensor UART & OWU Bus Demo                ");
  #endif
  Serial.println("_____________________________________________________________________");
  Serial.println("ADDR 0 |    1   |    2   |    3   |    4   |    5   |    6   |    7  ");
  Serial.println("-------|--------|--------|--------|--------|--------|--------|-------");
  delay(100);
  
  while(1){
  
      for(int i = 0; i < numDevices ; i++)
      {
        // -+-+-+-+-+-+-+-+-+-+-  PRESET 2 BURST-AND-LISTEN   -+-+-+-+-+-+-+-+-+-+- //
          distance = 0.00;
          uartAddrUpdate = i;
          // do not reconfigure comm mode, do not care about baud here, only updating command addresses
          ussc.initBoostXLPGA460(demoType, baudRate, uartAddrUpdate); 
          
          ussc.ultrasonicCmd(1, 1); //Preset 2 burst-and-listen for 1 Obj
          successMeas = ussc.pullUltrasonicMeasResult(demoMode);
          
          if(successMeas == true){distance = ussc.printUltrasonicMeasResult(0);}
          else{distance = 0.00;} //UMR command fail, force print 0.00m

        // -+-+-+-+-+-+-+-+-+-+-  SERIAL FORMATTING   -+-+-+-+-+-+-+-+-+-+- //
          int dSize = (8-sizeof(distance))/2;
          for (int j = 0; j < dSize;j++){Serial.print(" ");}  
          Serial.print(distance);
          for (int j = 0; j < dSize ;j++){Serial.print(" ");}        
          if(i == numDevices-1){Serial.println("");}
          else{Serial.print(" ");}

        // -+-+-+-+-+-+-+-+-+-+-  SERIAL MONITORING   -+-+-+-+-+-+-+-+-+-+- //
          // Check for serial character at COM terminal
          while (Serial.available()) 
          {     
           char inChar = (char)Serial.read(); // get the new byte
           // if the incoming character is a 'q', set a flag, stop the main loop, and re-run initialization
            if (inChar == 'q'){stringComplete = true; initPGA460();}
          }  
      }
    }
}


