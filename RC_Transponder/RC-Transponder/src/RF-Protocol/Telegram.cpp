/*
 * Telegram.cpp
 *
 * Created: 08-02-2019 11:28:50
 *  Author: klo
 */ 
 #include "Telegram.h"
 #include "Arduino.h" // for serial.print

 
 Telegram::Telegram(void){}

 /*
 Telegram::Telegram(uint8_t *payload, uint8_t payloadSize)
 {
	memcpy(payload, &Payload, payloadSize);	
	PayloadLength = payloadSize;
	ReadPayloadHeader();
 }*/
 Telegram::Telegram(uint8_t *data, uint8_t size)
 {
 	if(size >= MAX_PAYLOAD_LENGTH)
	 	return;

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
		To	 = (uint16_t)(Payload[0] << 8) + Payload[1];
		From = (uint16_t)(Payload[2] << 8) + Payload[3];
		MSG_ID = (ProtocolMSG_t)Payload[4];		
	 }
 }

 void Telegram::GenerateHeader(void)
 {
	// Header:
	Payload[0] = (uint8_t)((To >> 8) & 0xFF);
	Payload[1] = (uint8_t)(To & 0xFF);
	Payload[2] = (uint8_t)((From >> 8) & 0xFF);
	Payload[3] = (uint8_t)(From & 0xFF);
	Payload[4] = (uint8_t)(MSG_ID & 0xFF);
 }

 uint8_t Telegram::GetPayloadLength(void){
	return PayloadLength;
 }