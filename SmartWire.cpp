#include "Wire.h"
#include "SmartWire.h"

// frame[] is used to recieve and transmit packages. 
// The maximum serial ring buffer size is 128
unsigned char SmartTwoWire::frame[BUFFER_LENGTH];
unsigned int SmartTwoWire::holdingRegsSize; // size of the register array
unsigned int* SmartTwoWire::regs; // user array address
unsigned char SmartTwoWire::broadcastFlag;
unsigned char SmartTwoWire::slaveID;
unsigned char SmartTwoWire::function;
unsigned int SmartTwoWire::errorCount;
unsigned char SmartTwoWire::framePos;

void (*SmartTwoWire::user_onDataReceive)(void);

void SmartTwoWire::begin(unsigned char _slaveID, unsigned int _holdingRegsSize, unsigned int* _regs){
	TwoWire::begin(_slaveID);
  	TWAR = (_slaveID << 1) | 1;  // enable broadcasts to be received
	slaveID = _slaveID;
    onReceive(onDataReceived);
  holdingRegsSize = _holdingRegsSize; 
  regs = _regs;
  errorCount = 0; // initialize errorCount
}

// sets function called on slave write
void SmartTwoWire::onDataReceive( void (*function)(void) )
{
  user_onDataReceive = function;
}

void SmartTwoWire::onDataReceived(int howMany)
{
	SmartWire.readData(howMany);
}

void SmartTwoWire::readData(int howMany)
{
	if (available() == 0) {
		return;
	}

	  unsigned char buffer = 0;
	  unsigned char overflow = 0;

	  while (available() > 0){
		  if (overflow) 
			  read();
		  else
		  {
			  if (buffer == BUFFER_LENGTH)
				  overflow = 1;
			  frame[buffer] = read();
			  buffer++;
		  }
	  }

	  // If an overflow occurred increment the errorCount
	  // variable and return to the main sketch without 
	  // responding to the request i.e. force a timeout
	  if (overflow)
		  return;

	  // The minimum request packet is 8 bytes for function 3 & 16
    if (buffer > 7) 
	{
		unsigned char id = frame[0];
		
        unsigned int crc = ((frame[buffer - 2] << 8) | frame[buffer - 1]); // combine the crc Low & High bytes
        if (calculateCRC(buffer - 2) == crc) // if the calculated crc matches the recieved crc continue
        {
				  function = frame[1];
				  unsigned int startingAddress = ((frame[2] << 8) | frame[3]); // combine the starting address bytes
				  unsigned int no_of_registers = ((frame[4] << 8) | frame[5]); // combine the number of register bytes	
				  unsigned int maxData = startingAddress + no_of_registers;
				  unsigned char index;
				  unsigned char address;
				  unsigned int crc16;
				
				  // broadcasting is not supported for function 3 
				  if (function == 3)
				  {
					  if (startingAddress < holdingRegsSize) // check exception 2 ILLEGAL DATA ADDRESS
					  {
						  if (maxData <= holdingRegsSize) // check exception 3 ILLEGAL DATA VALUE
						  {
							  unsigned char noOfBytes = no_of_registers * 2; 
                // ID, function, noOfBytes, (dataLo + dataHi)*number of registers,
                //  crcLo, crcHi
							  unsigned char responseFrameSize = 5 + noOfBytes; 
							  frame[0] = slaveID;
							  frame[1] = function;
							  frame[2] = noOfBytes;
							  address = 3; // PDU starts at the 4th byte
							  unsigned int temp;
							
							  for (index = startingAddress; index < maxData; index++)
						  	{
								  temp = regs[index];
								  frame[address] = temp >> 8; // split the register into 2 bytes
								  address++;
								  frame[address] = temp & 0xFF;
								  address++;
							  }	
							
							  crc16 = calculateCRC(responseFrameSize - 2);
							  frame[responseFrameSize - 2] = crc16 >> 8; // split crc into 2 bytes
							  frame[responseFrameSize - 1] = crc16 & 0xFF;
							  sendPacket(responseFrameSize);
						  }
						  else	
							  exceptionResponse(3); // exception 3 ILLEGAL DATA VALUE
					  }
					  else
						  exceptionResponse(2); // exception 2 ILLEGAL DATA ADDRESS
				  }
				  else if (function == 16)
				  {
					  // Check if the recieved number of bytes matches the calculated bytes 
            // minus the request bytes.
					  // id + function + (2 * address bytes) + (2 * no of register bytes) + 
            // byte count + (2 * CRC bytes) = 9 bytes
					  if (frame[6] == (buffer - 9)) 
					  {
						  if (startingAddress < holdingRegsSize) // check exception 2 ILLEGAL DATA ADDRESS
						  {
							  if (maxData <= holdingRegsSize) // check exception 3 ILLEGAL DATA VALUE
							  {
								  address = 7; // start at the 8th byte in the frame
								
								  for (index = startingAddress; index < maxData; index++)
							  	{
									  regs[index] = ((frame[address] << 8) | frame[address + 1]);
									  address += 2;
								  }	
								
								  // only the first 6 bytes are used for CRC calculation
								  crc16 = calculateCRC(6); 
								  frame[6] = crc16 >> 8; // split crc into 2 bytes
								  frame[7] = crc16 & 0xFF;
								
								  // a function 16 response is an echo of the first 6 bytes from 
                  // the request + 2 crc bytes
								  if (!broadcastFlag) // don't respond if it's a broadcast message
									  sendPacket(8); 
							  }
							  else	
								  exceptionResponse(3); // exception 3 ILLEGAL DATA VALUE
						  }
						  else
							  exceptionResponse(2); // exception 2 ILLEGAL DATA ADDRESS
					  }
					  else 
						  errorCount++; // corrupted packet
          }					
				  else
					  exceptionResponse(1); // exception 1 ILLEGAL FUNCTION
        }
			  else // checksum failed
				  errorCount++;
    }
	else if (buffer > 0 && buffer < 8) {
		  errorCount++; // corrupted packet
	}
}

void SmartTwoWire::exceptionResponse(unsigned char exception)
{
  // each call to exceptionResponse() will increment the errorCount
	errorCount++; 
	if (!broadcastFlag) // don't respond if its a broadcast message
	{
		frame[0] = slaveID;
		frame[1] = (function | 0x80); // set MSB bit high, informs the master of an exception
		frame[2] = exception;
		unsigned int crc16 = calculateCRC(3); // ID, function|0x80, exception code
		frame[3] = crc16 >> 8;
		frame[4] = crc16 & 0xFF;
    // exception response is always 5 bytes 
    // ID, function + 0x80, exception code, 2 bytes crc
		sendPacket(5); 
	}
}

unsigned int SmartTwoWire::calculateCRC(unsigned char bufferSize) 
{
  unsigned int temp, temp2, flag;
  temp = 0xFFFF;
  for (unsigned char i = 0; i < bufferSize; i++)
  {
    temp = temp ^ frame[i];
    for (unsigned char j = 1; j <= 8; j++)
    {
      flag = temp & 0x0001;
      temp >>= 1;
      if (flag)
        temp ^= 0xA001;
    }
  }
  // Reverse byte order. 
  temp2 = temp >> 8;
  temp = (temp << 8) | temp2;
  temp &= 0xFFFF; 
  // the returned value is already swapped
  // crcLo byte is first & crcHi byte is last
  return temp; 
}

void SmartTwoWire::sendPacket(unsigned char bufferSize)
{
  beginTransmission(0);
		
  for (unsigned char i = 0; i < bufferSize; i++)
    write(frame[i]);

  endTransmission();
}

void SmartTwoWire::initEvent() {
	frame[0] = slaveID;
	frame[1] = 0x00;
	frame[2] = 0x00;	// no of bytes
	framePos = 3;
}

void SmartTwoWire::writeToBuf(unsigned char b) {
	frame[framePos] = b;
	framePos++;
}

void SmartTwoWire::writeToBuf(unsigned int b) {
	frame[framePos] = b >> 8;
	frame[framePos+1] = b & 0xFF;
	framePos += 2;
}

void SmartTwoWire::writeToBuf(float b) {
    char* floatPtr = (char*) &b;
	frame[framePos] = floatPtr[0];
	frame[framePos + 1] = floatPtr[1];
	frame[framePos + 2] = floatPtr[2];
	frame[framePos + 3] = floatPtr[3];
	framePos += 4;
}

void SmartTwoWire::flush() {
	frame[2] = framePos - 4;
	
	unsigned int crc16;
	crc16 = calculateCRC(framePos);
	frame[framePos + 1] = crc16 >> 8; // split crc into 2 bytes
	frame[framePos] = crc16 & 0xFF;
	framePos += 2;
	sendPacket(framePos);
}

SmartTwoWire SmartWire = SmartTwoWire();
