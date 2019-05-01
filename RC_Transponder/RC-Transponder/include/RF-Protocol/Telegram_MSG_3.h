/*
	Telegram_MSG_3.h

	Copyright (c) 2019 Lagoni
	Not for commercial use
 */ 



#ifndef Telegram_MSG_3_H_
#define Telegram_MSG_3_H_

#include "Telegram.h"
#include "E28-2G4M20S.h" // for RadioData_t struct.

class Telegram_MSG_3 : public Telegram
{
	public:
	virtual void SerialPrintMessage( void );	// Function for each massage to print out the data to Serial.print.
	ProtocolCMD_t GetCommand(void);				// Returns the Command.

	// Constructor to create empty messages. (only used for compile to allocate memory)
	//Telegram_MSG_3();
				
	// Constructor to create message from variables.
	Telegram_MSG_3(uint32_t _Unique_ID_1, uint32_t _Unique_ID_2, uint32_t _Unique_ID_3, uint32_t _Unique_ID_4, ProtocolCMD_t _command);

	// Constructor to create messages from Radio data struct.
	Telegram_MSG_3(RadioData_t *radioData);
	
	// Constructor to create messages from Base64 encoded Unique ID.
	Telegram_MSG_3(std::string Unique_ID, ProtocolCMD_t cmd);
	
	virtual	~Telegram_MSG_3(){}; 	// destructor.

	protected:
	
	private:
	ProtocolCMD_t cmd;
};


#endif /* Telegram_MSG_3_H_ */