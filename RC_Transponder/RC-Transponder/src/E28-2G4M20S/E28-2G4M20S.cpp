/*
	E28-2G4M20S.cpp

	Copyright (c) 2019 Lagoni
	Not for commercial use
 */ 

#include "E28-2G4M20S.h"

E28_2G4M20S::E28_2G4M20S(int chipSelectPin, int resetPin, int busyPin, int dio1Pin, int dio2Pin, int dio3Pin, int txEnablePin, int rxEnablePin, int ledPin)
{
	_chipSelectPin = chipSelectPin;
	_resetPin      = resetPin;
	_busyPin       = busyPin;
	_dio1Pin       = dio1Pin;
	_dio2Pin       = dio2Pin;
	_dio3Pin       = dio3Pin;
	_txEnablePin   = txEnablePin;
	_rxEnablePin   = rxEnablePin;
	_ledPin		   = ledPin;
		
	Radio = new SX1280Hal(_chipSelectPin, _busyPin, _dio1Pin, _dio2Pin, _dio3Pin, _resetPin, _txEnablePin, _rxEnablePin, _ledPin);	
		
	RadioStatus.rxDone=false;
	RadioStatus.txDone=false;
	RadioStatus.rxTimeout=false;
	RadioStatus.txTimeout=false;
}

void E28_2G4M20S::SetTxModeActive( void )
{
	Radio->SetRxEnablePin(LOW);
	Radio->SetTxEnablePin(HIGH);
}

void E28_2G4M20S::SetRxModeActive( void )
{
	Radio->SetTxEnablePin(LOW);
	Radio->SetRxEnablePin(HIGH);
}

void E28_2G4M20S::Init()
{
	 Radio->Init( );	 
	 Radio->SetRegulatorMode( USE_DCDC ); // Can also be set in LDO mode but consume more power
	 memset( &RadioData.payload, 0x00, MAX_PAYLOAD_LENGTH ); // Zero fills the buffer
	 
	#if defined( MODE_BLE )

//    SerialUSB.print( "\nRunning in BLE mode\n\r");
    modulationParams.PacketType                   = PACKET_TYPE_BLE;
    modulationParams.Params.Ble.BitrateBandwidth  = GEN_BLE_BR_0_125_BW_0_3;
    modulationParams.Params.Ble.ModulationIndex   = GEN_BLE_MOD_IND_1_00;
    modulationParams.Params.Ble.ModulationShaping = RADIO_MOD_SHAPING_BT_1_0;

    packetParams.PacketType                 = PACKET_TYPE_BLE;
    packetParams.Params.Ble.BlePacketType   = BLE_EYELONG_1_0;
    packetParams.Params.Ble.ConnectionState = BLE_MASTER_SLAVE;
    packetParams.Params.Ble.CrcField        = BLE_CRC_3B;
    packetParams.Params.Ble.Whitening       = RADIO_WHITENING_OFF;

	#elif defined( MODE_GENERIC )

//    SerialUSB.print( "\nRunning in GENERIC mode\n\r");
    modulationParams.PacketType                       = PACKET_TYPE_GENERIC;
    modulationParams.Params.Generic.BitrateBandwidth  = GEN_BLE_BR_0_125_BW_0_3;
    modulationParams.Params.Generic.ModulationIndex   = GEN_BLE_MOD_IND_1_00;
    modulationParams.Params.Generic.ModulationShaping = RADIO_MOD_SHAPING_BT_1_0;

    packetParams.PacketType                    = PACKET_TYPE_GENERIC;
    packetParams.Params.Generic.PreambleLength = PREAMBLE_LENGTH_32_BITS;
    packetParams.Params.Generic.SyncWordLength = GEN_SYNCWORD_LENGTH_5_BYTE;
    packetParams.Params.Generic.SyncWordMatch  = RADIO_RX_MATCH_SYNCWORD_1;
    packetParams.Params.Generic.HeaderType     = RADIO_PACKET_VARIABLE_LENGTH;
    packetParams.Params.Generic.PayloadLength  = MAX_PAYLOAD_LENGTH;
    packetParams.Params.Generic.CrcLength      = RADIO_CRC_3_BYTES;
    packetParams.Params.Generic.Whitening      = RADIO_WHITENING_ON;

	#elif defined( MODE_LORA )
	
//    SerialAUX->println( "Running in LORA mode");
//	SerialAUX->println( "Spreading factor set to 7");
//	SerialAUX->println( "Bandwidth set to 400kHz");
//	SerialAUX->println( "CR encoding set to 4/5");
    modulationParams.PacketType                  = PACKET_TYPE_LORA;
    modulationParams.Params.LoRa.SpreadingFactor = LORA_SF7;
    modulationParams.Params.LoRa.Bandwidth       = LORA_BW_0400;
    modulationParams.Params.LoRa.CodingRate      = LORA_CR_4_5;

    PacketParams.PacketType                 = PACKET_TYPE_LORA;
    PacketParams.Params.LoRa.PreambleLength = 0x08;
    PacketParams.Params.LoRa.HeaderType     = LORA_PACKET_VARIABLE_LENGTH;
    PacketParams.Params.LoRa.PayloadLength  = MAX_PAYLOAD_LENGTH;
    PacketParams.Params.LoRa.CrcMode        = LORA_CRC_ON;
    PacketParams.Params.LoRa.InvertIQ       = LORA_IQ_INVERTED;

	#elif defined( MODE_FLRC )

//    SerialUSB.print( "\nRunning in FLRC mode\n\r");
    modulationParams.PacketType                    = PACKET_TYPE_FLRC;
    modulationParams.Params.Flrc.BitrateBandwidth  = FLRC_BR_0_260_BW_0_3;
    modulationParams.Params.Flrc.CodingRate        = FLRC_CR_1_2;
    modulationParams.Params.Flrc.ModulationShaping = RADIO_MOD_SHAPING_BT_1_0;

    packetParams.PacketType                 = PACKET_TYPE_FLRC;
    packetParams.Params.Flrc.PreambleLength = PREAMBLE_LENGTH_32_BITS;
    packetParams.Params.Flrc.SyncWordLength = FLRC_SYNCWORD_LENGTH_4_BYTE;
    packetParams.Params.Flrc.SyncWordMatch  = RADIO_RX_MATCH_SYNCWORD_1;
    packetParams.Params.Flrc.HeaderType     = RADIO_PACKET_VARIABLE_LENGTH;
    packetParams.Params.Flrc.PayloadLength  = MAX_PAYLOAD_LENGTH;
    packetParams.Params.Flrc.CrcLength      = RADIO_CRC_3_BYTES;
    packetParams.Params.Flrc.Whitening      = RADIO_WHITENING_OFF;

	#else
	#error "Please select the mode of operation for the RC Transponder"
	#endif

	Radio->SetStandby( STDBY_RC );
    Radio->SetPacketType( modulationParams.PacketType );
    Radio->SetModulationParams( &modulationParams );
    Radio->SetPacketParams( &PacketParams );

    Radio->SetRfFrequency( rf_frequency );
    Radio->SetBufferBaseAddresses( 0x00, 0x00 );
    Radio->SetTxParams( tx_power, RADIO_RAMP_20_US ); 
//	SerialAUX->println( "Frequency set to " + String(rf_frequency/1000000) + "MHz");
//	SerialAUX->println( "TX Power set to " + String(tx_power) + "dBm");
}


void E28_2G4M20S::OnTxDone( void )
{
	// switch PA to RX.
	SetRxModeActive();
	SetRXMode(false); // Set to RX with no timeout.
	Radio->SetLed(LOW);
}

void E28_2G4M20S::OnRxDone( void )
{
	memset(&RadioData.payload, 0x00, MAX_PAYLOAD_LENGTH);
	RadioData.payloadLength=0;
	if(Radio->GetPayload(&RadioData.payload[0], &RadioData.payloadLength, MAX_PAYLOAD_LENGTH)){
		// If return 1, then package size is larger than MAX_PAYLOAD_LENGTH
//		SerialAUX->println("Oops! - New package is to big. Length="+String(RadioData.payloadLength)+". Max Size="+String(MAX_PAYLOAD_LENGTH-2));
	}else{
		//Extract and test CRC (last two bytes):
		uint16_t temp_crc = (uint16_t)((RadioData.payload[RadioData.payloadLength-2] << 8) + RadioData.payload[RadioData.payloadLength-1]);
		RadioData.payload[RadioData.payloadLength-2]=0;
		RadioData.payload[RadioData.payloadLength-1]=0;
		RadioData.payloadLength -= 2; // we have removed CRC.
//		SerialAUX->print("\n\n Checking CRC...");			
		if(CalculateCRC(&RadioData.payload[0],RadioData.payloadLength) == temp_crc){
//			SerialAUX->println("OK!");			
			// New data has been copied to buffer.
			Radio->GetPacketStatus(&PacketStatus);
			switch( PacketStatus.packetType )
			{
				case PACKET_TYPE_GFSK:
					RadioData.rssi	 = PacketStatus.Gfsk.RssiSync;
				break;

				case PACKET_TYPE_LORA:
				case PACKET_TYPE_RANGING:
					RadioData.rssi = PacketStatus.LoRa.RssiPkt;
					RadioData.snr = PacketStatus.LoRa.SnrPkt;
				break;

				case PACKET_TYPE_FLRC:
					RadioData.rssi = PacketStatus.Flrc.RssiSync;
				break;

				case PACKET_TYPE_BLE:
				RadioData.rssi = PacketStatus.Ble.RssiSync;
				break;

				case PACKET_TYPE_NONE:
				RadioData.rssi = 0;
				break;
			}
			RadioStatus.rxDone=true;
			RadioDataReady = true;
		}else{
//			SerialAUX->println("ERROR!");
			RadioStatus.rxDone=false;
		}
	}
	SetRXMode(false); // Set to RX with no timeout.
}

// Only call this when interrupt has occurred.
void E28_2G4M20S::IRQHandler( void )
{
	Radio->ProcessIrqs();	
	
	// reset all flags.
//	RadioStatus.rxDone=Radio->RadioPacketStatus.rxDone;  // Set in RXDone, if CRC is ok.
	RadioStatus.txDone=Radio->RadioPacketStatus.txDone;
	RadioStatus.rxTimeout=Radio->RadioPacketStatus.rxTimeout;
	RadioStatus.txTimeout=Radio->RadioPacketStatus.txTimeout;
	//SerialAUX->println("E28 Radio Interrupt!");
	
	if(Radio->RadioPacketStatus.txDone == true){
//		SerialAUX->println("TX Done!");
		this->OnTxDone();
	}    
	if(Radio->RadioPacketStatus.rxDone == true){
//		SerialAUX->println("RX Done!");
		this->OnRxDone();
		// read the message, check CRC if ok, make available.
	}
	if(Radio->RadioPacketStatus.rxSyncWordDone == true){
//		SerialAUX->println("rxSyncWordDone!");
	}	
	if(Radio->RadioPacketStatus.rxHeaderDone == true){
//		SerialAUX->println("rxHeaderDone!");
	}
	if(Radio->RadioPacketStatus.txTimeout == true){
//		SerialAUX->println("TX Timeout!");
	}
	if(Radio->RadioPacketStatus.rxTimeout == true){
//		SerialAUX->println("RX Timeout!");
	}	
	if(Radio->RadioPacketStatus.rxError > 0x00){
//		SerialAUX->println("IRQ Error! " + String(Radio->RadioPacketStatus.rxError));
	}
	if(Radio->RadioPacketStatus.rangingDone > 0x00){
//		SerialAUX->println("IRQ Ranging Code! " + String(Radio->RadioPacketStatus.rangingDone));
	}
	if(Radio->RadioPacketStatus.cadDone == true){
//		SerialAUX->println("CAD Done!");
	}
	
}

void E28_2G4M20S::SendRadioData(RadioData_t *data)
{
	// lets build data fro transmission, including adding 16bit CRC.
	RadioData.payloadLength=data->payloadLength;
	memcpy(&RadioData.payload, data->payload, RadioData.payloadLength);	// copy the data.

	// Prepare Package with CRC
	uint16_t temp_crc = CalculateCRC(&RadioData.payload[0], RadioData.payloadLength);
	RadioData.payload[RadioData.payloadLength++] = (uint8_t)((temp_crc >>  8) & 0xFF);  // CRC
	RadioData.payload[RadioData.payloadLength++] = (uint8_t)(temp_crc & 0xFF);		    // CRC	
			
	this->SetTxModeActive(); // Switch the hardware amplifier to TX mode.
	Radio->SetDioIrqParams( TxIrqMask, TxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE ); // Set module to interrupt on TX complete on DI01.
	this->PacketParams.Params.LoRa.PayloadLength = RadioData.payloadLength;
	Radio->SetPacketParams( &PacketParams );
	Radio->SendPayload(&RadioData.payload[0], RadioData.payloadLength, ( TickTime_t ){ RX_TIMEOUT_TICK_SIZE, TX_TIMEOUT_VALUE } );
	
	//Debug:
	Radio->SetLed(HIGH);
}

void E28_2G4M20S::SetRXMode(bool useTimeout)
{
   this->SetRxModeActive(); // Switch the hardware amplifier to RX mode.
   Radio->SetDioIrqParams( RxIrqMask, RxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE ); // Set module to interrupt on RX complete on DI01 or Timeout.
   if(useTimeout){
		Radio->SetRx( ( TickTime_t ) { RX_TIMEOUT_TICK_SIZE, RX_TIMEOUT_VALUE } );
   }else{
		Radio->SetRx(RX_TX_CONTINUOUS);
   }
}

void E28_2G4M20S::Sleep(void){
	SleepParams_t SleepParameters;
	
	SleepParameters.WakeUpRTC=0;
	SleepParameters.InstructionRamRetention=1;
	SleepParameters.DataBufferRetention=1;
	SleepParameters.DataRamRetention=1;
	
//	SerialAUX->println("Radio Firmware version: " + String(Radio->GetFirmwareVersion()));
	
	Radio->SetTxEnablePin(LOW);
	Radio->SetRxEnablePin(LOW);
		
	Radio->SetSaveContext();
	Radio->SetSleep(SleepParameters);	

//	digitalWrite(_chipSelectPin, HIGH);
}

void E28_2G4M20S::WakeUp(void){
	Radio->SetWakeup();
}


RadioData_t * E28_2G4M20S::GetRadioData()
{
	if(RadioDataReady){
		RadioDataReady = false;
		return &RadioData;	
	}else{
		return nullptr;
	}
}

RadioIRQStatus_t E28_2G4M20S::GetRadioStatus(){
	return this->RadioStatus;
}

void E28_2G4M20S::ClearRadioStatus(){
	RadioStatus.rxDone=false;
	RadioStatus.txDone=false;
	RadioStatus.rxTimeout=false;
	RadioStatus.txTimeout=false;	
}

uint16_t E28_2G4M20S::CalculateCRC(uint8_t *data, uint8_t length){
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

uint16_t  E28_2G4M20S::GetFirmwareVersion( void ){
	return Radio->GetFirmwareVersion();
}