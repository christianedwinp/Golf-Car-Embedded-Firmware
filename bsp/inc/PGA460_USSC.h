/*
	PGA460_USSC.h
	
	BSD 2-clause "Simplified" License
	Copyright (c) 2017, Texas Instruments
	All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions are met:

	1. Redistributions of source code must retain the above copyright notice, this
	   list of conditions and the following disclaimer.
	2. Redistributions in binary form must reproduce the above copyright notice,
	   this list of conditions and the following disclaimer in the documentation
	   and/or other materials provided with the distribution.

	THIS SOFTWARE IS PROVIDED BY TEXAS INSTRUMENTS "AS IS" AND
	ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
	WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
	DISCLAIMED. IN NO EVENT SHALL TEXAS INSTRUMENTS BE LIABLE FOR
	ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
	(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
	ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
	(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
	SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

	The views and conclusions contained in the software and documentation are those
	of the authors and should not be interpreted as representing official policies,
	either expressed or implied, of the FreeBSD Project.
	
	Last Updated: Nov 2017
	By: A. Whitehead <make@energia.nu>
*/
 
//#include <Energia.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <bsp_ultrasonic.h>
#define  byte unsigned char
byte pullEchoDataDump(byte element,short uartIndex);
byte registerRead(byte Regaddr, short uartIndex);
byte registerWrite(byte Regaddr, byte data, short uartIndex);
void SetPGAAddress(byte uartaddress);
void initSTM32F1PGA460(byte mode, uint32_t baud, int setAddress, byte uartAddrUpdate);
void defaultPGA460(byte xdcr,byte usartaddress);
void initThresholds(byte thr,short uartIndex);
void initTVG(byte agr, byte tvg,short uartIndex);
void ultrasonicCmd(byte cmd, byte numObjUpdate, short uartIndex);
void runEchoDataDump(byte preset,short uartIndex);
void broadcast(bool eeBulk, bool tvgBulk, bool thrBulk);
void toggleLEDs(bool ds1State, bool fdiagState, bool vdiagState);
void autoThreshold(byte cmd, byte noiseMargin, byte windowIndex, byte autoMax, byte avgLoops,short uartIndex);
void eepromThreshold(byte preset, bool saveLoad,short uartIndex);
void thresholdBulkRead(byte preset,short uartIndex);
void thresholdBulkWrite(byte p1ThrMap[], byte p2ThrMap[],short uartIndex);
bool burnEEPROM(short uartIndex);
bool pullUltrasonicMeasResult(bool busDemo,short uartIndex);
double printUltrasonicMeasResult(byte umr,Stack *ultrasonic, short index);
double runDiagnostics(byte run, byte diag,short uartIndex);
double triangulation(double a, double b, double c);

byte calcChecksum(byte cmd);
void pga460SerialFlush(void);
void tciRecord(byte numObj);
void tciByteToggle(byte data, byte zeroPadding);
void tciIndexRW(byte index, bool write);
void tciCommand(byte cmd);
void spiTransfer(byte* mosi, byte size);
void spiMosiIdle(byte size);
void pinMode(void);
