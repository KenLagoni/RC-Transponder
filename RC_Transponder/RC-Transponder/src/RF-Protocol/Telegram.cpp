/*
	Telegram.cpp

	Copyright (c) 2019 Lagoni
	Not for commercial use
 */ 
 #include "Telegram.h"
  
 const std::string Telegram::base64_chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

 Telegram::Telegram(ProtocolMSG_t _MSG_ID,  uint32_t _Unique_ID_1, uint32_t _Unique_ID_2, uint32_t _Unique_ID_3, uint32_t _Unique_ID_4)
 {
	 this->setup(_MSG_ID, _Unique_ID_1, _Unique_ID_2, _Unique_ID_3, _Unique_ID_4);
	/*
 	// header:
 	this->MSG_ID = _MSG_ID;
 	this->Unique_ID_1 = _Unique_ID_1;
 	this->Unique_ID_2 = _Unique_ID_2;
 	this->Unique_ID_3 = _Unique_ID_3;
 	this->Unique_ID_4 = _Unique_ID_4;

	memset(&TelegramData.payload, 0x00, MAX_PAYLOAD_LENGTH); // Zero fills the buffer

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
	*/
}
 

 Telegram::Telegram(RadioData_t *data)
 {
	if(data!=NULL){
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
 }
 
 
Telegram::Telegram(ProtocolMSG_t _MSG_ID, std::string Unique_ID)
{
	// header:
	this->MSG_ID = _MSG_ID;
	
	// base64 decode:
	//std::cout << "ID IN:" << Unique_ID << ":" << '\n';
	std::string ID = this->base64_decode(Unique_ID);
	//std::cout << "ID  OUT length:" << ID.length() << '\n';
	if(ID.length() == 16)
	{
		this->Unique_ID_1 = (uint32_t)((ID[0] << 24) + (ID[1] << 16) + (ID[2] << 8) + ID[3]);
		this->Unique_ID_2 = (uint32_t)((ID[4] << 24) + (ID[5] << 16) + (ID[6] << 8) + ID[7]);
		this->Unique_ID_3 = (uint32_t)((ID[8] << 24) + (ID[9] << 16) + (ID[10] << 8) + ID[11]);
		this->Unique_ID_4 = (uint32_t)((ID[12] << 24) + (ID[13] << 16) + (ID[14] << 8) + ID[15]);
	}		
	
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
 
void Telegram::setup(ProtocolMSG_t _MSG_ID,  uint32_t _Unique_ID_1, uint32_t _Unique_ID_2, uint32_t _Unique_ID_3, uint32_t _Unique_ID_4){
 	// header:
 	this->MSG_ID = _MSG_ID;
 	this->Unique_ID_1 = _Unique_ID_1;
 	this->Unique_ID_2 = _Unique_ID_2;
 	this->Unique_ID_3 = _Unique_ID_3;
 	this->Unique_ID_4 = _Unique_ID_4;

 	memset(&TelegramData.payload, 0x00, MAX_PAYLOAD_LENGTH); // Zero fills the buffer

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

std::string Telegram::GetUniqueID()
{
	uint8_t id[16];
	
	for(int a = 0; a<16;a++)
	{
		id[a]=TelegramData.payload[a+1];
	}
	return base64_encode(id,16);
}


std::string Telegram::base64_encode(uint8_t bytes_to_encode[], int in_len)
{
	std::string ret = "";
	int i = 0;
	int j = 0;
	uint8_t char_array_3[3];
	uint8_t char_array_4[4];
	int place = 0;

	while (in_len-- > 0) {
		char_array_3[i++] = bytes_to_encode[place++];
		if (i == 3) {
			char_array_4[0] = (uint8_t)((char_array_3[0] & 0xfc) >> 2);
			char_array_4[1] = (uint8_t)(((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4));
			char_array_4[2] = (uint8_t)(((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6));
			char_array_4[3] = (uint8_t)(char_array_3[2] & 0x3f);

			for(i = 0; (i<4) ; i++)
			ret += base64_chars[char_array_4[i]];
			i = 0;
		}
	}

	if (i > 0) {
		for(j = i; j< 3; j++)
		char_array_3[j] = 0;

		char_array_4[0] = (uint8_t)(( char_array_3[0] & 0xfc) >> 2);
		char_array_4[1] = (uint8_t)(((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4));
		char_array_4[2] = (uint8_t)(((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6));

		for (j = 0; (j<i + 1); j++)
		ret += base64_chars[char_array_4[j]];
				
		while((i++ < 3))
		ret += '=';

	}

	return ret;

}

std::string Telegram::base64_decode(std::string const& encoded_string) 
{
	int in_len = encoded_string.size();
	int i = 0;
	int j = 0;
	int in_ = 0;
	unsigned char char_array_4[4], char_array_3[3];
	std::string ret;

	while (in_len-- && ( encoded_string[in_] != '=') && ((isalnum(encoded_string[in_]) || (encoded_string[in_] == '+') || (encoded_string[in_] == '/'))) ) {
		char_array_4[i++] = encoded_string[in_]; in_++;
		if (i ==4) {
			for (i = 0; i <4; i++)
			char_array_4[i] = base64_chars.find(char_array_4[i]);

			char_array_3[0] = ( char_array_4[0] << 2       ) + ((char_array_4[1] & 0x30) >> 4);
			char_array_3[1] = ((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);
			char_array_3[2] = ((char_array_4[2] & 0x3) << 6) +   char_array_4[3];

			for (i = 0; (i < 3); i++)
			ret += char_array_3[i];
			i = 0;
		}
	}

	if (i) {
		for (j = 0; j < i; j++)
		char_array_4[j] = base64_chars.find(char_array_4[j]);

		char_array_3[0] = (char_array_4[0] << 2) + ((char_array_4[1] & 0x30) >> 4);
		char_array_3[1] = ((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);

		for (j = 0; (j < i - 1); j++) ret += char_array_3[j];
	}

	return ret;
}