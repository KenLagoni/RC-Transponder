/*
  FrSky DIY sensor class for Passthrough protocol
  (c) Lagoni
  Not for commercial use
*/

#include "FrSkySportSensorPassthrough.h" 

FrSkySportSensorPassthrough::FrSkySportSensorPassthrough(SensorId id) : FrSkySportSensor(id) { }

void FrSkySportSensorPassthrough::setDataTextMSG(char* text, uint8_t severity) // Severity 0-4 (buzzer sound) 5-7 (ping sound). 0-3=Text Red, 4=Warning text yellow, 5-7=text white.
{
  	temp_payload=0;
  	char chr=0;
//	Serial.println("Frsky PassThrough setData adding text \"" + String(text) + "\" with data: ");
	do{
  		for(int a=0;a<4;a++){
	  		chr = *text;
//			SerialPrintHEX(chr);
//			Serial.print(" ");	
	  		temp_payload |= (uint32_t)chr;
	  		if(a<3){
		  		temp_payload = temp_payload << 8;
	  		}
			  
	  		if(chr != 0){
		  		text++; // move to next char.
	  		}
  		}
		
		if(chr != 0){
	//		Serial.print(" Pushing payload: ");
	//		SerialPrintHEX(payload);
	//		Serial.println("");
			if(!this->textFIFO.isFull()){
				this->textFIFO.push(temp_payload);
			}else{
				Serial.println("Frsky PassThrough STATTUS TEXT buffer full!");
			}
			temp_payload=0;
		}
	}while(chr != 0);

//	Serial.print(" last payload: ");
//	SerialPrintHEX(payload);
//	Serial.print(" Severity(" + String(severity) + ") pushing modified payload: ");

	temp_payload |= (severity & 0x4)<<21;
	temp_payload |= (severity & 0x2)<<14;
	temp_payload |= (severity & 0x1)<<7;
//	SerialPrintHEX(payload);
//	Serial.println("");

	if(!this->textFIFO.isFull()){
		this->textFIFO.push(temp_payload);
	}else{
		Serial.println("Frsky PassThrough STATTUS TEXT buffer full!");
	}
}


void FrSkySportSensorPassthrough::setDataAPStatus(uint8_t flightMode, uint8_t simple, uint8_t landComplete, uint8_t armed){
	apStatusPayload = 0;
	apStatusPayload |= flightMode & 0x1F; // Flight mode on first 5 bits.
	apStatusPayload |= (uint32_t)((simple & 0x03) << 5); // Simple on next 2 bits.
	apStatusPayload |= (uint32_t)((landComplete & 0x01) << 7); // land complete on next bit.
	apStatusPayload |= (uint32_t)((armed & 0x01) << 8); // land complete on next bit.
	apStatusDataReady = true;
}
/*
void FrSkySportSensorPassthrough::setDataAPStatus_Part2(uint8_t imu_temp){

	apStatusPayload |= (uint32_t)((imu_temp & 0x3F) << 26); // land complete on next bit.
	apStatusDataReady = true;
}
*/

void FrSkySportSensorPassthrough::setDataGPSStatus(uint8_t numberOfSat, uint8_t fix, int32_t lattidue, int32_t longitude, uint16_t hdop, uint8_t advanced, int32_t altitudeMSL){
	gpsStatusPayload = 0;
	gpsStatusPayload |= this->bit32Pack(numberOfSat, 0, 4);
	gpsStatusPayload |= this->bit32Pack(fix ,4, 2);    
	gpsStatusPayload |= this->bit32Pack(advanced ,14, 2); 
		
	if (hdop > 1270){
		hdop_temp = 1270;   // unknown state 0xFFFF from MAVLink is also relayed as 127 @rotorman 2021/01/25		
	}else{
		hdop_temp = hdop / 10;
	}

	gpsStatusPayload |= this->bit32Pack(prep_number(hdop_temp,2,1) ,6, 8);
	
	amsl_temp = altitudeMSL / 100;  // dm
	if (amsl_temp > 127000){ // upper bound +12.7 km
		amsl_temp = 127000;
	}
	if (amsl_temp < -127000){ // lower bound -12.7 km
		amsl_temp = -127000;
	}

	amsl = prep_number(amsl_temp,2,2);
	gpsStatusPayload |= this->bit32Pack(amsl,22, 9);
	
	if (amsl < 0) {
		gpsStatusPayload |= this->bit32Pack(0, 31,1);   // 1=negative     // @rotorman  2021/01/18
	} else {
		gpsStatusPayload |= this->bit32Pack(0, 31,0);
	}
	gpsStatusDataReady=true;
}

void FrSkySportSensorPassthrough::setDataBattery1Status(float voltage, float current, float totalCurrentSinceStart){
	bat1StatusPayload = this->packBatteryPayload(voltage, current, totalCurrentSinceStart);
	bat1StatusDataReady=true;
}

void FrSkySportSensorPassthrough::setDataBattery2Status(float voltage, float current, float totalCurrentSinceStart){
	bat2StatusPayload = this->packBatteryPayload(voltage, current, totalCurrentSinceStart);
	bat2StatusDataReady=true;
}

// Input
// home latitude           [degE7]  From MSG #242
// home longitude          [degE7]  From MSG #242
// current latitude        [degE7]  From MSG #33
// current longitude       [degE7]  From MSG #33
// Altitude above ground      [mm]  From MSG #33
void FrSkySportSensorPassthrough::setDataHomeStatus_FromMavlink(int32_t homeLatitude, int32_t homeLongitude, int32_t currentLatitude, int32_t currentLongitude, int32_t altitudeAboveGround){
	
	cur_lat = (float)currentLatitude / 1E7;
	cur_lon = (float)currentLongitude / 1E7;
	
	hom_lat = (float)homeLatitude / 1E7;
	hom_lon = (float)homeLongitude / 1E7;
	
	lon1=hom_lon/180*PI;  // degrees to radians
	lat1=hom_lat/180*PI;
	lon2=cur_lon/180*PI;
	lat2=cur_lat/180*PI;

	//Calculate azimuth bearing of craft from home
	a=atan2(sin(lon2-lon1)*cos(lat2), cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(lon2-lon1));
	az=a*180/PI;  // radians to degrees
	if (az<0){
	 az=360+az;
	}

	home_angle = this->Add360(az, -180);                  // Is now the angle from the craft to home in degrees
	
	// Calculate the distance from home to craft
	dLat = (lat2-lat1);
	dLon = (lon2-lon1);
	a = sin(dLat/2) * sin(dLat/2) + sin(dLon/2) * sin(dLon/2) * cos(lat1) * cos(lat2);
	c = 2* asin(sqrt(a));    // proportion of Earth's radius
	dis = 6371000 * c;       // radius of the Earth is 6371km
	
	alt = (float)altitudeAboveGround / 1000; // [meters]

	this->setDataHomeStatus(dis, alt, home_angle);
} 



// Input:
// distanceToHome	  [meters]
// altitudeDiffToHome [meters]
// angleToHome		  [degrees]

void FrSkySportSensorPassthrough::setDataHomeStatus(float distanceToHome, float altitudeDiffToHome, float angleToHome){
	
	homeStatusPayload = 0;
	home_dist = prep_number(roundf(distanceToHome), 3, 2);
	homeStatusPayload |= bit32Pack(home_dist ,0, 12);
	home_alt = prep_number(roundf(altitudeDiffToHome*10), 3, 2); // dm
	homeStatusPayload |= bit32Pack(home_alt ,12, 12);
	if (home_alt < 0){
		homeStatusPayload |= bit32Pack(1,24, 1);
	}else{
		homeStatusPayload |= bit32Pack(0,24, 1);
	}
	home_arrow = (int16_t)(angleToHome * 0.33333); // 0 = heading pointing to home, unit = 3 degrees
	homeStatusPayload |= bit32Pack(home_arrow,25, 7);
	homeStatusDataReady=true;
}


// Input:
// Vertical climb	   [m/s]
// ground speed        [m/s]
// heading 		       [degrees] 0-360

void FrSkySportSensorPassthrough::setDataVYAWStatus(float vclimb, float groundSpeed, float heading){
	
	 vyawStatusPayload = 0;
	 
	 pt_vy = vclimb * 10;       // from #74   m/s to dm/s;
	 pt_vx = groundSpeed * 10;  // from #74  m/s to dm/s
	 pt_yaw = heading * 10;     // degrees -> (degrees*10) or use ap_yaw (-rad, +rad)
 
	 if (pt_vy<0){
		vyawStatusPayload |= bit32Pack(1, 8, 1);	 
	 }else{
		vyawStatusPayload |= bit32Pack(0, 8, 1);
	 }
	 
	 pt_vy = prep_number(roundf(pt_vy), 2, 1);  // Vertical velocity
	 vyawStatusPayload |= bit32Pack(pt_vy, 0, 8);

	 pt_vx = prep_number(roundf(pt_vx), 2, 1);  // Horizontal velocity
	 vyawStatusPayload |= bit32Pack(pt_vx, 9, 8);
	 pt_yaw = pt_yaw * 0.5f;                   // Unit = 0.2 deg
	 vyawStatusPayload |= bit32Pack(pt_yaw ,17, 11);	
	 
	 vyawStatusDataReady=true;
}

void FrSkySportSensorPassthrough::setDataAttitude(float roll, float pitch){
	// input roll is in rad.
	attitudePayload = 0;
	rollDegrees = (roll * 18000)/3.1415;   // Rad to centidegrees
	pitchDegrees = (pitch * 18000)/3.1415; // Rad to centidegrees
	
	//Serial.println("Roll " + String(rollDegrees) + " Pitch " + String(pitchDegrees));
	
	attitudePayload = (uint32_t)((rollDegrees + 18000) * 0.05f) & 0x7FF;
	attitudePayload |= ((uint32_t)((pitchDegrees + 9000) * 0.05f) & 0x7FF) << 11;
	
	
	
	attitudeDataReady = true;
	// Ranger finder TBD.
	
	// roll from [-18000;18000] centidegrees to unsigned .2 degree increments [0;1800] (just in case, limit to 2047 (0x7FF) since the value is stored on 11 bits)
	// uint32_t attiandrng = ((uint16_t)roundf((_ahrs.roll_sensor + 18000) * 0.05f) & ATTIANDRNG_ROLL_LIMIT);
	// attiandrng |= ((uint16_t)roundf((_ahrs.pitch_sensor + 9000) * 0.05f) & ATTIANDRNG_PITCH_LIMIT)<<ATTIANDRNG_PITCH_OFFSET;
}



void FrSkySportSensorPassthrough::setDataParameterFrameType(uint32_t frame_Type){
	this->packParameterPayload(frame_Type, FRAME_TYPE);	
}

void FrSkySportSensorPassthrough::setDataParameterBatteryPack1Capacity(uint32_t bat1_cap){
	this->packParameterPayload(bat1_cap, BATT_CAPACITY_1);
}

void FrSkySportSensorPassthrough::setDataParameterBatteryPack2Capacity(uint32_t bat2_cap){
	this->packParameterPayload(bat2_cap, BATT_CAPACITY_2);
}

void FrSkySportSensorPassthrough::setDataParameterNumberOfWaypointInMission(uint32_t num_way){
	this->packParameterPayload(num_way, TELEMETRY_FEATURES);
}

void FrSkySportSensorPassthrough::packParameterPayload(uint32_t data, uint8_t paramID){
	tempPayload = 0;
	tempPayload |= bit32Pack(data, 0, 24);
	tempPayload |= bit32Pack(paramID, 24, 4);
	
	if(!this->paramsFIFO.isFull()){
		this->paramsFIFO.push(tempPayload);
	}else{
		Serial.println("Frsky PassThrough Parameter buffer full!");
	}	
}

uint32_t FrSkySportSensorPassthrough::packBatteryPayload(float voltage, float current, float totalCurrentSinceStart){
	battempPayload = 0;
	battempPayload |= bit32Pack((uint32_t)(voltage * 10),0, 9); // dV
	bat_amps = prep_number(roundf((int32_t)(current*10)),2,1); // A
	battempPayload |= bit32Pack(bat_amps,9, 8);
	battempPayload |= bit32Pack(totalCurrentSinceStart,17, 15);
	
	return battempPayload;
}


// Input (from Mavlink #24 #62 #42):
// Waypoint Number			         0-N 
// Distance to next waypoint     [meters]
// xTrack error				     [meters]
// Course over ground			[degrees]
// Next Waypoint offest bearing [degrees]

void FrSkySportSensorPassthrough::setDataMissionWaypoints(uint16_t wpNumber, uint16_t distanceToNextWaypoint, float xTrackError, float cog, int16_t nextWPbearing){
	missionStatusPayload = 0;
  
    pt_ms_dist = distanceToNextWaypoint;                        // Distance to next WP  
    pt_ms_xtrack = xTrackError;                                 // Cross track error in metres from #62
    pt_ms_target_bearing = nextWPbearing;                       // Direction of next WP
    angle = (int32_t)this->wrap_360(pt_ms_target_bearing - cog);
    arrowStep = 360 / 8; 
    pt_ms_offset = ((angle + (arrowStep/2)) / arrowStep) % 8;       // Next WP bearing offset from COG

      /*
   
    0 - up
    1 - up-right
    2 - right
    3 - down-right
    4 - down
    5 - down - left
    6 - left
    7 - up - left
 
       */
   
     missionStatusPayload |= bit32Pack(wpNumber, 0, 10);    //  WP number
     pt_ms_dist = prep_number(roundf(pt_ms_dist), 3, 2);       //  number, digits, power
     missionStatusPayload |=bit32Pack(pt_ms_dist, 10, 12);    
     pt_ms_xtrack = prep_number(roundf(pt_ms_xtrack), 1, 1);  
     missionStatusPayload |=bit32Pack(pt_ms_xtrack, 22, 6); 
     missionStatusPayload |=bit32Pack(pt_ms_offset, 29, 3);  

	missionStatusDataReady = true;	
}


// Input (from Mavlink #74):
// Airspeed	          [m/s]
// Throttle           0-100
// Altitude(Baro)  [meters]
void FrSkySportSensorPassthrough::setDataVFRHUD(float airspeed, uint16_t throttle, float baroAltitude){
      vfrHudPayload = 0;
      
      pt_air_spd = airspeed * 10;      // from #74  m/s to dm/s
      pt_throt = throttle;             // 0 - 100%
      pt_bar_alt = baroAltitude * 10;  // m to dm

      pt_air_spd = prep_number(roundf(pt_air_spd), 2, 1);
      vfrHudPayload |= this->bit32Pack(pt_air_spd, 0, 8);

      vfrHudPayload |= this->bit32Pack(throttle, 8, 7);

      pt_bar_alt =  prep_number(roundf(pt_bar_alt), 3, 2);
 
	  vfrHudPayload |= this->bit32Pack(pt_bar_alt, 15, 12);
      if (pt_bar_alt < 0){
		vfrHudPayload |= this->bit32Pack(1, 27, 1);		  
	  }else{
		vfrHudPayload |= this->bit32Pack(0, 27, 1);			  
	  }
	 vfrHudDataReady = true;
}




void FrSkySportSensorPassthrough::send(FrSkySportSingleWireSerial& serial, uint8_t id, uint32_t now)
{
  if(sensorId == id)
  {

	bool dataReplied = false; // Keep track 
	uint8_t sensorLoopedID=LowPrioritySensorDataIdx; // in case we come back to this one, then all sensors
	
	do{
		switch(nextSensorType)
		{
			case HIGH: // High priority sensors 
			{
			    switch(sensorDataIdx)
				{
					case 0:
					{
						if( (now > textTime) && (!this->textFIFO.isEmpty()))
						{
							if(msgRetransmit++ >= 3){
								this->textFIFO.pop(textPayload);			
								msgRetransmit=0;				
							}
							textTime = now + PASSTHROUGH_TEXT_DATA_PERIOD;
							serial.sendData(PASSTHROUGH_TEXT_MSG_ID, textPayload);									
							dataReplied=true; // indicate we used the slot. 	

	//						Serial.println("Frsky PassThrough sending Status Text data.");		
		/*					Serial.print("Frsky PassThrough sending payload: ");
							SerialPrintHEX(payload);
							Serial.println("");
		*/				}
					}
					break;

					case 1:
					{
						if( attitudeDataReady && (now > attitudeTime))
						{
							attitudeTime = now + PASSTHROUGH_ATTITUDE_PERIOD;
							attitudeDataReady = false;
							serial.sendData(PASSTHROUGH_ATTITUDE_ID, attitudePayload);		
							dataReplied=true; // indicate we used the slot.	

				//			Serial.println("Frsky PassThrough sending Attitude data.");
						}
					}
					break;
					
					case 2:
					{
						if( (now > paramsTime) && (!this->paramsFIFO.isEmpty()))
						{
							if(paramsRetransmit++ >= 3){
								this->paramsFIFO.pop(paramsPayload);			
								paramsRetransmit=0;				
							}
							paramsTime = now + PASSTHROUGH_PARAMS_PERIOD;
							serial.sendData(PASSTHROUGH_PARAMS_ID, paramsPayload);									
							dataReplied=true; // indicate we used the slot. 	
						}
					}
					break;					
				}
				sensorDataIdx++;
				if(sensorDataIdx >= PASSTHROUGH_HIGH_PRIORITY_DATA_COUNT) sensorDataIdx = 0;			
				nextSensorType=LOW;
			}
			break;
		
			case LOW: // High priority sensors 
			{
				switch(LowPrioritySensorDataIdx)
				{
					case 0:
					{
						if( apStatusDataReady && (now > apStatusTime))
						{
							apStatusTime = now + PASSTHROUGH_AP_STATUS_PERIOD;
							apStatusDataReady=false;
							serial.sendData(PASSTHROUGH_AP_STATUS_ID, apStatusPayload);
							dataReplied=true; // indicate we used the slot.

				//			Serial.println("Frsky PassThrough sending AP Status data.");
						}
					}
					break;		
					
					case 1:
					{
						if( gpsStatusDataReady && (now > gpsStatusTime))
						{
							gpsStatusTime = now + PASSTHROUGH_GPS_STATUS_PERIOD;
							gpsStatusDataReady=false;
							serial.sendData(PASSTHROUGH_GPS_STATUS_ID, gpsStatusPayload);
							dataReplied=true; // indicate we used the slot.

							//			Serial.println("Frsky PassThrough sending GPS status data.");
						}
					}
					break;	
										
					case 2:
					{
						if( bat1StatusDataReady && (now > bat1StatusTime))
						{
							bat1StatusTime = now + PASSTHROUGH_BAT1_STATUS_PERIOD;
							bat1StatusDataReady=false;
							serial.sendData(PASSTHROUGH_BAT1_STATUS_ID, bat1StatusPayload);
							dataReplied=true; // indicate we used the slot.
							Serial.print("Frsky PassThrough sending BAT1 status data: ");
							SerialPrintHEX(bat1StatusPayload);
							Serial.println("");
						}/*else{
							if( (now > bat1StatusTime) && !bat1StatusDataReady ){
								Serial.println("it is time to send bat1Status, but no new data! nextTime:" + String(bat1StatusTime) + " Now:" + String(now));
							}
						}*/
					}
					break;		
					
					case 3:
					{
						if( homeStatusDataReady && (now > homeStatusTime))
						{
							homeStatusTime = now + PASSTHROUGH_HOME_STATUS_PERIOD;
							homeStatusDataReady=false;
							serial.sendData(PASSTHROUGH_HOME_STATUS_ID, homeStatusPayload);
							dataReplied=true; // indicate we used the slot.
				//			Serial.println("Frsky PassThrough sending HOME status data.");
						}
					}
					break;
													
					case 4:
					{
						if( vyawStatusDataReady && (now > vyawStatusTime) )
						{
							vyawStatusTime = now + PASSTHROUGH_VYAW_STATUS_PERIOD;
							vyawStatusDataReady=false;
							serial.sendData(PASSTHROUGH_VYAW_STATUS_ID, vyawStatusPayload);
							dataReplied=true; // indicate we used the slot.
							//			Serial.println("Frsky PassThrough sending VELANDYAW status data.");
						}
					}
					break;
					
					case 5:
					{
						if( bat2StatusDataReady && (now > bat2StatusTime))
						{
							bat2StatusTime = now + PASSTHROUGH_BAT2_STATUS_PERIOD;
							bat2StatusDataReady=false;
							serial.sendData(PASSTHROUGH_BAT2_STATUS_ID, bat2StatusPayload);
							dataReplied=true; // indicate we used the slot.
						}
					}
					break;	

					case 6:
					{
						if( vfrHudDataReady && (now > vfrHudTime))
						{
							vfrHudTime = now + PASSTHROUGH_VFR_HUD_PERIOD;
							vfrHudDataReady=false;
							serial.sendData(PASSTHROUGH_VFR_HUD_ID, vfrHudPayload);
							dataReplied=true; // indicate we used the slot.
						}
					}
					break;
										
					case 7:
					{
						if( missionStatusDataReady && (now >  missionStatusTime))
						{
							missionStatusTime = now + PASSTHROUGH_MISSION_STATUS_PERIOD;
							missionStatusDataReady=false;
							serial.sendData(PASSTHROUGH_MISSION_STATUS_ID,  missionStatusPayload);
							dataReplied=true; // indicate we used the slot.
						}
					}
					break;
				}
			
				LowPrioritySensorDataIdx++;
				if(LowPrioritySensorDataIdx >= PASSTHROUGH_LOW_PRIORITY_DATA_COUNT) LowPrioritySensorDataIdx = 0;	
				if(sensorLoopedID == LowPrioritySensorDataIdx){ // no data ready for any sensor, stop the loop.
					dataReplied=true;
			//		Serial.println("Frsky PassThrough No data ready for reply.");
				}
				nextSensorType=HIGH;
			}
			break;
		} // switch(nextSensorType)	
	}while(!dataReplied);
  } // if(sensorId == id)
}



uint16_t FrSkySportSensorPassthrough::decodeData(uint8_t id, uint16_t appId, uint32_t data)
{
 
  return 0;
}


uint32_t FrSkySportSensorPassthrough::createMask(uint8_t lo, uint8_t hi) {
	uint32_t r = 0;
	for (unsigned i=lo; i<=hi; i++)
	r |= 1 << i;
	return r;
}

// Mask then AND the shifted bits, then return the value to be OR with the payload
uint32_t FrSkySportSensorPassthrough::bit32Pack(uint32_t dword ,uint8_t displ, uint8_t lth) {
	uint32_t dw_and_mask =  (dword<<displ) & (createMask(displ, displ+lth-1));
	return dw_and_mask;
}

// From Arducopter 3.5.5 code
uint16_t FrSkySportSensorPassthrough::prep_number(int32_t number, uint8_t digits, uint8_t power)
{
	uint16_t res = 0;
	uint32_t abs_number = abs(number);

	if ((digits == 1) && (power == 1)) { // number encoded on 5 bits: 4 bits for digits + 1 for 10^power
		if (abs_number < 10) {
			res = abs_number<<1;
			} else if (abs_number < 150) {
			res = ((uint8_t)roundf(abs_number * 0.1f)<<1)|0x1;
			} else { // transmit max possible value (0x0F x 10^1 = 150)
			res = 0x1F;
		}
		if (number < 0) { // if number is negative, add sign bit in front
			res |= 0x1<<5;
		}
		} else if ((digits == 2) && (power == 1)) { // number encoded on 8 bits: 7 bits for digits + 1 for 10^power
		if (abs_number < 100) {
			res = abs_number<<1;
			} else if (abs_number < 1270) {
			res = ((uint8_t)roundf(abs_number * 0.1f)<<1)|0x1;
			} else { // transmit max possible value (0x7F x 10^1 = 1270)
			res = 0xFF;
		}
		if (number < 0) { // if number is negative, add sign bit in front
			res |= 0x1<<8;
		}
		} else if ((digits == 2) && (power == 2)) { // number encoded on 9 bits: 7 bits for digits + 2 for 10^power
		if (abs_number < 100) {
			res = abs_number<<2;
			//   Log.print("abs_number<100  ="); Log.print(abs_number); Log.print(" res="); Log.print(res);
			} else if (abs_number < 1000) {
			res = ((uint8_t)roundf(abs_number * 0.1f)<<2)|0x1;
			//   Log.print("abs_number<1000  ="); Log.print(abs_number); Log.print(" res="); Log.print(res);
			} else if (abs_number < 10000) {
			res = ((uint8_t)roundf(abs_number * 0.01f)<<2)|0x2;
			//  Log.print("abs_number<10000  ="); Log.print(abs_number); Log.print(" res="); Log.print(res);
			} else if (abs_number < 127000) {
			res = ((uint8_t)roundf(abs_number * 0.001f)<<2)|0x3;
			} else { // transmit max possible value (0x7F x 10^3 = 127000)
			res = 0x1FF;
		}
		if (number < 0) { // if number is negative, add sign bit in front
			res |= 0x1<<9;
		}
		} else if ((digits == 3) && (power == 1)) { // number encoded on 11 bits: 10 bits for digits + 1 for 10^power
		if (abs_number < 1000) {
			res = abs_number<<1;
			} else if (abs_number < 10240) {
			res = ((uint16_t)roundf(abs_number * 0.1f)<<1)|0x1;
			} else { // transmit max possible value (0x3FF x 10^1 = 10240)
			res = 0x7FF;
		}
		if (number < 0) { // if number is negative, add sign bit in front
			res |= 0x1<<11;
		}
		} else if ((digits == 3) && (power == 2)) { // number encoded on 12 bits: 10 bits for digits + 2 for 10^power
		if (abs_number < 1000) {
			res = abs_number<<2;
			} else if (abs_number < 10000) {
			res = ((uint16_t)roundf(abs_number * 0.1f)<<2)|0x1;
			} else if (abs_number < 100000) {
			res = ((uint16_t)roundf(abs_number * 0.01f)<<2)|0x2;
			} else if (abs_number < 1024000) {
			res = ((uint16_t)roundf(abs_number * 0.001f)<<2)|0x3;
			} else { // transmit max possible value (0x3FF x 10^3 = 127000)
			res = 0xFFF;
		}
		if (number < 0) { // if number is negative, add sign bit in front
			res |= 0x1<<12;
		}
	}
	return res;
}

//=================================================================================================
//Add two bearing in degrees and correct for 360 boundary
int16_t FrSkySportSensorPassthrough::Add360(int16_t arg1, int16_t arg2) {
	int16_t ret = arg1 + arg2;
	if (ret < 0) ret += 360;
	if (ret > 359) ret -= 360;
	return ret;
}
//=================================================================================================
// Correct for 360 boundary - yaapu
float FrSkySportSensorPassthrough::wrap_360(int16_t angle)
{
	const float ang_360 = 360.f;
	float res = fmodf(static_cast<float>(angle), ang_360);
	if (res < 0) {
		res += ang_360;
	}
	return res;
}