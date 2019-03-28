/*
	Telegram.cpp

	Copyright (c) 2019 Lagoni
	Not for commercial use
 */ 
 #include "Telegram.h"
 #include "Arduino.h" // for serial.print

 
 Telegram::Telegram(){
	memset(&Payload, 0,  MAX_PAYLOAD_LENGTH);
 }

 /*
 Telegram::Telegram(uint8_t *payload, uint8_t payloadSize)
 {
	memcpy(payload, &Payload, payloadSize);	
	PayloadLength = payloadSize;
	ReadPayloadHeader();
 }*/

 Telegram::Telegram(uint8_t *data, uint8_t size)
 {
 	if(size >= MAX_PAYLOAD_LENGTH){
		Serial.println("ERROR! - Radio Telegram too long! (size=" + String(size) + ", Max=" + String(MAX_PAYLOAD_LENGTH));
	 	return;
	 }

	/*
	Serial.println("Input to memcpy:");
	for(int a=0;a<size;a++)
		Serial.println("data["+String(a)+"]=" + String(data[a]));
	 */
	memset(&Payload, 0,  MAX_PAYLOAD_LENGTH);
	memcpy(&Payload,data,size);	
	if(size >= HEADER_SIZE){
		ReadPayloadHeader();
	}
		/*
	Serial.println("");
	Serial.println("Output from memcpy:");
	for(int a=0;a<size;a++)
		Serial.println("data["+String(a)+"]=" + String(Payload[a]));
	*/
 }


 void Telegram::ReadPayloadHeader(void){
	MSG_ID = (ProtocolMSG_t)Payload[0];		
	Unique_ID_1 = (uint32_t)((Payload[1] << 24) + (Payload[2] << 16) + (Payload[3] << 8) + Payload[4]);
	Unique_ID_2 = (uint32_t)((Payload[5] << 24) + (Payload[6] << 16) + (Payload[7] << 8) + Payload[8]);
	Unique_ID_3 = (uint32_t)((Payload[9] << 24) + (Payload[10] << 16) + (Payload[11] << 8) + Payload[12]);
	Unique_ID_4 = (uint32_t)((Payload[13] << 24) + (Payload[14] << 16) + (Payload[15] << 8) + Payload[16]);
 }

 void Telegram::GenerateHeader(uint8_t *data){
	// Header:
	*data++  = (uint8_t)(MSG_ID & 0xFF);
	*data++  = (uint8_t)((Unique_ID_1 >> 24) & 0xFF);
	*data++  = (uint8_t)((Unique_ID_1 >> 16) & 0xFF);
	*data++  = (uint8_t)((Unique_ID_1 >>  8) & 0xFF);
	*data++  = (uint8_t)(Unique_ID_1 & 0xFF);
	*data++  = (uint8_t)((Unique_ID_2 >> 24) & 0xFF);
	*data++  = (uint8_t)((Unique_ID_2 >> 16) & 0xFF);
	*data++  = (uint8_t)((Unique_ID_2 >>  8) & 0xFF);
	*data++  = (uint8_t)(Unique_ID_2 & 0xFF);
	*data++  = (uint8_t)((Unique_ID_3 >> 24) & 0xFF);
	*data++ = (uint8_t)((Unique_ID_3 >> 16) & 0xFF);
	*data++ = (uint8_t)((Unique_ID_3 >>  8) & 0xFF);
	*data++ = (uint8_t)(Unique_ID_3 & 0xFF);
	*data++ = (uint8_t)((Unique_ID_4 >> 24) & 0xFF);
	*data++ = (uint8_t)((Unique_ID_4 >> 16) & 0xFF);
	*data++ = (uint8_t)((Unique_ID_4 >>  8) & 0xFF);
	*data   = (uint8_t)(Unique_ID_4 & 0xFF);
 }

uint8_t Telegram::GetRadioMSGLength(void){
	return RadioMSGLength;
}

uint8_t Telegram::GetSerialMSGLength(void){
	return SerialMSGLength;
}

uint32_t Telegram::GetUniqueID1(void){
	return Unique_ID_1;
}

uint32_t Telegram::GetUniqueID2(void){
	return Unique_ID_2;
}

uint32_t Telegram::GetUniqueID3(void){
	return Unique_ID_3;
}

uint32_t Telegram::GetUniqueID4(void){
	return Unique_ID_4;
}
/*
uint16_t Telegram::CalculateCRC(uint8_t offset, uint8_t length){
	uint16_t  crc;
	uint8_t i;
	uint8_t *ptr = &Payload[offset];
	crc = 0;
	while (length >= 1)
	{
		length--;
		crc = crc ^ (uint16_t) *ptr++ << 8;
		i = 8;
		do
		{
			if (crc & 0x8000)
				crc = crc << 1 ^ 0x1021;
			else
				crc = crc << 1;
		} while(--i);
	}
	return (crc);
}
*/
 /*
 uint16_t Telegram::CalculateCRC(uint8_t Payloadlength){
	 uint16_t temp_crc_res=0;
	 uint16_t temp_crc=0;
	 uint16_t temp_crc_2=0;
	 
	 for(int a=0 ; a<= Payloadlength ; a=a+2){
		 if( (a+1) >= Payloadlength){
			temp_crc = (uint16_t)Payload[a];
			temp_crc = temp_crc << 8;
		 }
		 else{
			temp_crc = (uint16_t)Payload[a];
			temp_crc = temp_crc << 8; 
			temp_crc_2 = (uint16_t)Payload[a+1];
			temp_crc = temp_crc + temp_crc_2;
		 }
		 
		 temp_crc_res ^= temp_crc;
	 }
	 return temp_crc_res;
 }
 */
 

  
 bool Telegram::TelegramMatchUniqueID(uint32_t _destinationID_1, uint32_t _destinationID_2, uint32_t _destinationID_3, uint32_t _destinationID_4){
	 if(_destinationID_1 == Unique_ID_1){
		 if(_destinationID_2 == Unique_ID_2){
			if(_destinationID_3 == Unique_ID_3){
				if(_destinationID_4 == Unique_ID_4){
					return true;
				}
			}
		 }
	 }
	 return false;
 }
 
 bool Telegram::CRCValid( void ){
	return _CRCValid;
 }

    	
uint16_t Telegram::CalculateCRC(uint8_t *data, uint8_t length){
        uint16_t count;
        uint16_t crc = 0xFFFF;
        uint16_t temp;

        for (count = 0; count < length; ++count)
        {
	        temp = (uint16_t)((*data++ ^ (crc >> 8)) & 0xff);
	        crc = (uint16_t)(crc_table[temp] ^ (crc << 8));
        }

        return (uint16_t)(crc ^ 0x0000);
}	

ProtocolMSG_t Telegram::GetRadioMSG_ID(){
	return MSG_ID;
}

        
       