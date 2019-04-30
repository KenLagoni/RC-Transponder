/*
	Telegram.cpp

	Copyright (c) 2019 Lagoni
	Not for commercial use
 */ 
 #include "Telegram.h"
 
 
 Telegram::Telegram(ProtocolMSG_t _MSG_ID,  uint32_t _Unique_ID_1, uint32_t _Unique_ID_2, uint32_t _Unique_ID_3, uint32_t _Unique_ID_4)
 {
 	// header:
 	this->MSG_ID = _MSG_ID;
 	this->Unique_ID_1 = _Unique_ID_1;
 	this->Unique_ID_2 = _Unique_ID_2;
 	this->Unique_ID_3 = _Unique_ID_3;
 	this->Unique_ID_4 = _Unique_ID_4;
	
	//Telegram MSG
	TelegramData.payload[0] = (uint8_t)MSG_ID;

	// Unique_ID_1 transformed to 32bits.
	TelegramData.payload[1] = (uint8_t)((Unique_ID_1 >> 24) & 0xFF);
	TelegramData.payload[2] = (uint8_t)((Unique_ID_1 >> 16) & 0xFF);
	TelegramData.payload[3] = (uint8_t)((Unique_ID_1 >>  8) & 0xFF);
	TelegramData.payload[4] = (uint8_t)(Unique_ID_1 & 0xFF);

	// Unique_ID_1 transformed to 32bits.
	TelegramData.payload[5] = (uint8_t)((Unique_ID_2 >> 24) & 0xFF);
	TelegramData.payload[6] = (uint8_t)((Unique_ID_2 >> 16) & 0xFF);
	TelegramData.payload[7] = (uint8_t)((Unique_ID_2 >>  8) & 0xFF);
	TelegramData.payload[8] = (uint8_t)(Unique_ID_2 & 0xFF);

	// Unique_ID_1 transformed to 32bits.
	TelegramData.payload[9] = (uint8_t)((Unique_ID_3 >> 24) & 0xFF);
	TelegramData.payload[10] = (uint8_t)((Unique_ID_3 >> 16) & 0xFF);
	TelegramData.payload[11] = (uint8_t)((Unique_ID_3 >>  8) & 0xFF);
	TelegramData.payload[12] = (uint8_t)(Unique_ID_3 & 0xFF);

	// Unique_ID_1 transformed to 32bits.
	TelegramData.payload[13] = (uint8_t)((Unique_ID_4 >> 24) & 0xFF);
	TelegramData.payload[14] = (uint8_t)((Unique_ID_4 >> 16) & 0xFF);
	TelegramData.payload[15] = (uint8_t)((Unique_ID_4 >>  8) & 0xFF);
	TelegramData.payload[16] = (uint8_t)(Unique_ID_4 & 0xFF);	
}
 

 Telegram::Telegram(RadioData_t *data)
 {
	memcpy(&TelegramData.payload,data->payload,data->payloadLength);
	TelegramData.payloadLength = data->payloadLength;
	TelegramData.rssi = data->rssi;
	TelegramData.snr = data->snr;

	MSG_ID = (ProtocolMSG_t)TelegramData.payload[0];
	Unique_ID_1 = (uint32_t)((TelegramData.payload[1] << 24) + (TelegramData.payload[2] << 16) + (TelegramData.payload[3] << 8) + TelegramData.payload[4]);
	Unique_ID_2 = (uint32_t)((TelegramData.payload[5] << 24) + (TelegramData.payload[6] << 16) + (TelegramData.payload[7] << 8) + TelegramData.payload[8]);
	Unique_ID_3 = (uint32_t)((TelegramData.payload[9] << 24) + (TelegramData.payload[10] << 16) + (TelegramData.payload[11] << 8) + TelegramData.payload[12]);
	Unique_ID_4 = (uint32_t)((TelegramData.payload[13] << 24) + (TelegramData.payload[14] << 16) + (TelegramData.payload[15] << 8) + TelegramData.payload[16]);	
 }
 
ProtocolMSG_t Telegram::GetRadioMSG_ID(){
	return MSG_ID;
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
  
int8_t Telegram::GetRSSI(){
	return TelegramData.rssi;
}

int8_t Telegram::GetSNR(){
	return TelegramData.snr;
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
  
RadioData_t * Telegram::GetRadioData(){
	return &TelegramData;
}       