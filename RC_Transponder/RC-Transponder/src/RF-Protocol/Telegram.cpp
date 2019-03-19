/*
	Telegram.cpp

	Copyright (c) 2019 Lagoni
	Not for commercial use
 */ 
 #include "Telegram.h"
 #include "Arduino.h" // for serial.print
 #include "hw.h" // for SerialAUX

 
 Telegram::Telegram(void){
	 crcValid = false;
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
	crcValid = false;
 	if(size >= MAX_PAYLOAD_LENGTH){
		Serial.println("ERROR! - Radio Telegram too long! (size=" + String(size) + ", Max=" + String(MAX_PAYLOAD_LENGTH));
	 	return;
	 }

	/*
	Serial.println("Input to memcpy:");
	for(int a=0;a<size;a++)
		Serial.println("data["+String(a)+"]=" + String(data[a]));
	 */
	memcpy(&Payload,data,size);	
	PayloadLength=size;
		/*
	Serial.println("");
	Serial.println("Output from memcpy:");
	for(int a=0;a<size;a++)
		Serial.println("data["+String(a)+"]=" + String(Payload[a]));
	*/
	ReadPayloadHeader();
 }


 void Telegram::ReadPayloadHeader(void)
 {
	 if(PayloadLength >= HEADER_SIZE){
		MSG_ID = (ProtocolMSG_t)Payload[0];		
		Unique_ID_1 = (uint32_t)((Payload[1] << 24) + (Payload[2] << 16) + (Payload[3] << 8) + Payload[4]);
		Unique_ID_2 = (uint32_t)((Payload[5] << 24) + (Payload[6] << 16) + (Payload[7] << 8) + Payload[8]);
		Unique_ID_3 = (uint32_t)((Payload[9] << 24) + (Payload[10] << 16) + (Payload[11] << 8) + Payload[12]);
		Unique_ID_4 = (uint32_t)((Payload[13] << 24) + (Payload[14] << 16) + (Payload[15] << 8) + Payload[16]);
	 }
 }

 void Telegram::GenerateHeader(void)
 {
	// Header:
	Payload[0]  = (uint8_t)(MSG_ID & 0xFF);
	Payload[1]  = (uint8_t)((Unique_ID_1 >> 24) & 0xFF);
	Payload[2]  = (uint8_t)((Unique_ID_1 >> 16) & 0xFF);
	Payload[3]  = (uint8_t)((Unique_ID_1 >>  8) & 0xFF);
	Payload[4]  = (uint8_t)(Unique_ID_1 & 0xFF);
	Payload[5]  = (uint8_t)((Unique_ID_2 >> 24) & 0xFF);
	Payload[6]  = (uint8_t)((Unique_ID_2 >> 16) & 0xFF);
	Payload[7]  = (uint8_t)((Unique_ID_2 >>  8) & 0xFF);
	Payload[8]  = (uint8_t)(Unique_ID_2 & 0xFF);
	Payload[9]  = (uint8_t)((Unique_ID_3 >> 24) & 0xFF);
	Payload[10] = (uint8_t)((Unique_ID_3 >> 16) & 0xFF);
	Payload[11] = (uint8_t)((Unique_ID_3 >>  8) & 0xFF);
	Payload[12] = (uint8_t)(Unique_ID_3 & 0xFF);
	Payload[13] = (uint8_t)((Unique_ID_4 >> 24) & 0xFF);
	Payload[14] = (uint8_t)((Unique_ID_4 >> 16) & 0xFF);
	Payload[15] = (uint8_t)((Unique_ID_4 >>  8) & 0xFF);
	Payload[16] = (uint8_t)(Unique_ID_4 & 0xFF);
 }

 uint8_t Telegram::GetPayloadLength(void){
	return PayloadLength;
 }
 
 uint8_t * Telegram::GetPayload(void){
	  return Payload;
 }
 
 
 uint16_t Telegram::CalculateCRC(uint8_t Payloadlength){
	 uint16_t temp_crc_res=0;
	 uint16_t temp_crc=0;
	 
	 for(int a=0 ; a<= Payloadlength ; a=a+2){
		 if( (a+1) >= Payloadlength){
			temp_crc = (uint16_t)(Payload[a] << 8);
		 }
		 else{
			temp_crc = (uint16_t)((Payload[a] << 8) + Payload[a+1]);
		 }
		 
		 temp_crc_res = temp_crc_res ^ temp_crc;
	 }
	 SerialAUX->println("CRC:" + String(temp_crc_res));
	 return temp_crc_res;
 }
 
 bool Telegram::TelegramValid(void){
	return crcValid;
 }
 
  
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
   
 	