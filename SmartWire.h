/*
Event structure (Based on modbus)
 Head:
 0 - Sender ID
 1 - Command (0 - broadcasting event)
 2 - data length X
 X - data
 X+1, X+2 - message CRC
 
 Data:
 First byte defines value type:
  0 - other
  1 - relay/switch/regulator position (0 - off, 255 - on)
  2 - temperature
  3 - general float value (1 byte for value ID, 4 bytes for value)
  4 - electricity
      4.1 (byte) - sensor no
      4.2 (float) - real power
      4.3 (float) - power factor
      4.4 (float) - Vrms
      4.5 (float) - Irms
      4.6 (float) - total kWh consumed per sensor
 */


// SimpleModbusSlaveV7

/*
 SimpleModbusSlave allows you to communicate
 to any slave using the Modbus RTU protocol.
 
 This implementation DOES NOT fully comply with the Modbus specifications.
 
 Specifically the frame time out have not been implemented according
 to Modbus standards. The code does however combine the check for
 inter character time out and frame time out by incorporating a maximum
 time out allowable when reading from the message stream.
 
 SimpleModbusSlave implements an unsigned int return value on a call to modbus_update().
 This value is the total error count since the slave started. It's useful for fault finding.
 
 This code is for a Modbus slave implementing functions 3 and 16
 function 3: Reads the binary contents of holding registers (4X references)
 function 16: Presets values into a sequence of holding registers (4X references)
 
 All the functions share the same register array.
 
 Note:  
 The Arduino serial ring buffer is 128 bytes or 64 registers.
 Most of the time you will connect the arduino to a master via serial
 using a MAX485 or similar.
 
 In a function 3 request the master will attempt to read from your
 slave and since 5 bytes is already used for ID, FUNCTION, NO OF BYTES
 and two BYTES CRC the master can only request 122 bytes or 61 registers.
 
 In a function 16 request the master will attempt to write to your 
 slave and since a 9 bytes is already used for ID, FUNCTION, ADDRESS, 
 NO OF REGISTERS, NO OF BYTES and two BYTES CRC the master can only write
 118 bytes or 59 registers.
 
 Using a USB to Serial converter the maximum bytes you can send is 
 limited to its internal buffer which differs between manufactures. 
 
 The functions included here have been derived from the 
 Modbus Specifications and Implementation Guides
 
 http://www.modbus.org/docs/Modbus_over_serial_line_V1_02.pdf
 http://www.modbus.org/docs/Modbus_Application_Protocol_V1_1b.pdf
 http://www.modbus.org/docs/PI_MBUS_300.pdf
*/

#ifndef SmartWire_h
#define SmartWire_h

#include "Wire.h"

#define SW_READINGS_BUFFER_LENGTH 20

typedef struct {
	unsigned char buffer[BUFFER_LENGTH];
	unsigned char length;
} SmartData;

class SmartTwoWire: public TwoWire
{
	private:
		static unsigned int holdingRegsSize; // size of the register array
		static unsigned int* regs; // user array address
		static unsigned char broadcastFlag;
		static unsigned char slaveID;
		static unsigned char function;
		static unsigned char framePos;
		static SmartData readingsBuffer[SW_READINGS_BUFFER_LENGTH];
		static unsigned char assignedBufferIndex;
		static unsigned char currentBufferIndex;
		static unsigned char isDataAvailable;
        static void (*user_onEventReceive)(void);
		static void onDataReceived(int);
		static void onEventReceived(unsigned char);
		void exceptionResponse(unsigned char exception);
		void readData(int);
	public:
		static unsigned char frame[];
		static unsigned char frameLength;
		static unsigned int errorCount;
		void begin(unsigned char _slaveID, unsigned int _holdingRegsSize, unsigned int* _regs);
		unsigned int calculateCRC(unsigned char* buffer, unsigned char bufferSize);
		void sendPacket(unsigned char bufferSize);
		void initEvent();
		void writeToBuf(unsigned char b);
		void writeToBuf(unsigned int b);
		void writeToBuf(float b);
		void flush();
		int available();
		SmartData readBuffer();
		void onEventReceive( void (*)(void) );
};

extern SmartTwoWire SmartWire;

#endif
