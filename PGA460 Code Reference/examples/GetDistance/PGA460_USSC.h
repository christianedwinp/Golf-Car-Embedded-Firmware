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
 
#include <Energia.h>
#include <string.h>
 
class pga460
{
  public:
	pga460();
	byte pullEchoDataDump(byte element);
	byte registerRead(byte addr);
	byte registerWrite(byte addr, byte data);
	void initBoostXLPGA460(byte mode, uint32_t baud, byte uartAddrUpdate);
	void defaultPGA460(byte xdcr);
	void initThresholds(byte thr);
	void initTVG(byte agr, byte tvg);
	void ultrasonicCmd(byte cmd, byte numObjUpdate);
	void runEchoDataDump(byte preset);
	void broadcast(bool eeBulk, bool tvgBulk, bool thrBulk);
	void toggleLEDs(bool ds1State, bool fdiagState, bool vdiagState);
	void autoThreshold(byte cmd, byte noiseMargin, byte windowIndex, byte autoMax, byte avgLoops);
	void eepromThreshold(byte preset, bool saveLoad);
	void thresholdBulkRead(byte preset);
	void thresholdBulkWrite(byte p1ThrMap[], byte p2ThrMap[]);
	bool burnEEPROM();
	bool pullUltrasonicMeasResult(bool busDemo);
	double printUltrasonicMeasResult(byte umr);
	double runDiagnostics(byte run, byte diag);
	double triangulation(double a, double b, double c);

  private:
	byte calcChecksum(byte cmd);
	void pga460SerialFlush();
	void tciRecord(byte numObj);
	void tciByteToggle(byte data, byte zeroPadding);
	void tciIndexRW(byte index, bool write);
	void tciCommand(byte cmd);
	void spiTransfer(byte* mosi, byte size);
	void spiMosiIdle(byte size);
};
