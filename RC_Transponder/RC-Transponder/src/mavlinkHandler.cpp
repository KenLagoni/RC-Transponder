/*
 * MavlinkHandler.cpp
 *
 * Created: 12/01/2022 11.10.44
 *  Author: klo
 */ 

#include "MavlinkHandler.h"

MavlinkHandler::MavlinkHandler(){}

void MavlinkHandler::begin(Uart *ptr1, FrSkySportSensorGps *ptr2, FrSkySportSensorPassthrough *ptr3){
	this->serial = ptr1;
	this->FrskyGPS = ptr2;
	this->FrskyPASS = ptr3;
}

void MavlinkHandler::service(void){
	
	if( (this->serial == NULL) || (this->FrskyGPS == NULL) || (this->FrskyPASS == NULL) ){
		return;	
	}
		
	
	 if(this->serial->available()){ // service serial data from FC.
		 uint8_t rbyte= this->serial->read();
		 if (mavlink_parse_char(MAVLINK_COMM_1, rbyte, &msgInput, &msgStatus))
		 {
//			Serial.println("New Mavlink messages #" + String(msgInput.msgid));
			// New Mavlink message received:		
			 switch(msgInput.msgid)
			 {
				 
				 case MAVLINK_MSG_ID_ATTITUDE: // #30
				 {
					 this->FrskyPASS->setDataAttitude( mavlink_msg_attitude_get_roll(&msgInput), mavlink_msg_attitude_get_pitch(&msgInput) );
					 // attitudeMsgHZ++;
					 // Serial.println("Attitude time: " + String(millis()-lastTime));
					 // lastTime=millis();
				 }
				 break;
				 
				 
				 case MAVLINK_MSG_ID_HEARTBEAT: // #0
				 {
					 uint8_t ap_type = mavlink_msg_heartbeat_get_type(&msgInput);
					 if (ap_type == 5 || ap_type == 6 || ap_type == 18 || ap_type == 26 || ap_type == 27) break;
					 // Ignore heartbeats from GCS (6) or Ant Trackers(5) or Onboard_Controllers(18) or Gremsy Gimbal(26) or ADSB (27))

					 uint32_t ap_custom_mode = mavlink_msg_heartbeat_get_custom_mode(&msgInput);
					 uint8_t ap_base_mode = mavlink_msg_heartbeat_get_base_mode(&msgInput);
					 uint8_t armed = ap_base_mode >> 7;
					  this->FrskyPASS->setDataAPStatus((uint8_t)(ap_custom_mode + 1), 0, armed, armed);

					 if(!frameTypeSent){
						frameTypeSent=true;
						 this->FrskyPASS->setDataParameterFrameType(ap_type);						 
					 }

				 }
				 break;


				 case MAVLINK_MSG_ID_SYS_STATUS: // #1
				 {
					 float voltage_battery1 = (float)Get_Volt_Average1(mavlink_msg_sys_status_get_voltage_battery(&msgInput));        // V  from Get_Volt_Average1()
					 voltage_battery1=voltage_battery1/1000; // now in [V]
					 float current_battery1 = (float)Get_Current_Average1(mavlink_msg_sys_status_get_current_battery(&msgInput));     // dA,  100 = 1A
					 current_battery1 = current_battery1 / 100; // now in [A]
					  this->FrskyPASS->setDataBattery1Status(voltage_battery1,current_battery1,bat1.tot_mAh);
				 }
				 break;
				 
				 case MAVLINK_MSG_ID_PARAM_VALUE: // #22 (I think we need to request this!
				 {
					 uint16_t param_index = mavlink_msg_param_value_get_param_index(&msgInput);
					
					 if(param_index == 356){ // Battery 1 capacity parameter
						 this->FrskyPASS->setDataParameterBatteryPack1Capacity(mavlink_msg_param_value_get_param_value(&msgInput));
					 }
					 
					 if(param_index == 364){ // Battery 2 capacity parameter
						 this->FrskyPASS->setDataParameterBatteryPack2Capacity(mavlink_msg_param_value_get_param_value(&msgInput));
					 }
				 }
				 break;			 
				 
				 case MAVLINK_MSG_ID_STATUSTEXT: //#253
				 {
					 mavlink_statustext_t msg;
					 mavlink_msg_statustext_decode(&msgInput, &msg);
					  this->FrskyPASS->setDataTextMSG(msg.text, msg.severity);
				 }
				 break;


				 case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: // #33
				 {
				    int32_t current_lat = mavlink_msg_global_position_int_get_lat(&msgInput);             // Latitude, expressed as degrees * 1E7
				    int32_t current_lon = mavlink_msg_global_position_int_get_lon(&msgInput);             // Longitude, expressed as degrees * 1E7
					int32_t alt_ag = mavlink_msg_global_position_int_get_relative_alt(&msgInput);         // Altitude above ground (millimeters)
										
					this->FrskyPASS->setDataHomeStatus_FromMavlink(home_latitude,home_longitude,current_lat,current_lon, alt_ag); // Home lat/long from MSG #242.
				 }
				 break;
				 				 
				    
			 	 case MAVLINK_MSG_ID_HOME_POSITION: // #242
			 	 {
				 	 home_latitude = mavlink_msg_home_position_get_latitude(&msgInput);
				 	 home_longitude = mavlink_msg_home_position_get_longitude(&msgInput);
			 	 }
			 	 break;
				  
				 case MAVLINK_MSG_ID_VFR_HUD:      //  #74
			 	 {
					  this->FrskyPASS->setDataVYAWStatus((float)mavlink_msg_vfr_hud_get_climb(&msgInput), (float)mavlink_msg_vfr_hud_get_groundspeed(&msgInput), (float)mavlink_msg_vfr_hud_get_heading(&msgInput));
					  this->FrskyPASS->setDataVFRHUD(mavlink_msg_vfr_hud_get_airspeed(&msgInput), mavlink_msg_vfr_hud_get_throttle(&msgInput), mavlink_msg_vfr_hud_get_alt(&msgInput));
			 	 }
			 	 break;				 
				  
		
		
				 case MAVLINK_MSG_ID_BATTERY_STATUS: // #147 
				 {

					  uint8_t battery_id = mavlink_msg_battery_status_get_id(&msgInput);
					  if (battery_id == 0) {  // Battery 1
						  bat1.tot_mAh = (float)mavlink_msg_battery_status_get_current_consumed(&msgInput);    // mAh
					  }/* 
					  else if (battery_id == 1) {  // Battery 2
						  pt_bat2_mAh = ap_current_consumed;
					  }*/					 
				 }
				 break;
				 
				 
			 	 case MAVLINK_MSG_ID_GPS_RAW_INT: // #24
				 {
					uint8_t fixtype = mavlink_msg_gps_raw_int_get_fix_type(&msgInput);  // 0 = No GPS, 1 =No Fix, 2 = 2D Fix, 3 = 3D Fix
					uint8_t sat_visible =  mavlink_msg_gps_raw_int_get_satellites_visible(&msgInput);    // number of visible satellites
					int32_t lat = mavlink_msg_gps_raw_int_get_lat(&msgInput);
					int32_t lon = mavlink_msg_gps_raw_int_get_lon(&msgInput);
					int32_t amsl = mavlink_msg_gps_raw_int_get_alt(&msgInput);     // 1m =1000
					uint16_t hdop = mavlink_msg_gps_raw_int_get_eph(&msgInput);     // GPS HDOP 
					
					// for Mission Waypoint
					cog=(float)mavlink_msg_gps_raw_int_get_cog(&msgInput); // in [cdeg]
					cog=cog/100; // now in [deg]
					
					this->FrskyPASS->setDataGPSStatus(sat_visible, (fixtype & 0x03),lat,lon, hdop, ((fixtype>>2) & 0x03), amsl);
					FrskyGPS->setData((float)(lat/1E7),(float)(lon/1E7),0,0,0,0,0,0,0,0,0); // only lat/lon data.	
				 }
				 break;

			 	 case MAVLINK_MSG_ID_MISSION_COUNT: // #44   received back after #43 Mission_Request_List sent
				 {
					 this->FrskyPASS->setDataParameterNumberOfWaypointInMission(mavlink_msg_mission_count_get_count(&msgInput));
				 }
				 break;	

			 	 case MAVLINK_MSG_ID_MISSION_CURRENT: // #42 
			 	 {
					currentWP =  mavlink_msg_mission_current_get_seq(&msgInput);  
				  }
			 	 break;
				  
			 	 case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT: // #62
			 	 {
					int16_t target_bearing = mavlink_msg_nav_controller_output_get_target_bearing(&msgInput);   // Bearing to current waypoint/target [degrees]
					uint16_t wp_dist = mavlink_msg_nav_controller_output_get_wp_dist(&msgInput);                // Distance to active waypoint [meters]
					float xtrack_error = mavlink_msg_nav_controller_output_get_xtrack_error(&msgInput);         // Current crosstrack error on x-y plane [meters]
	
					 this->FrskyPASS->setDataMissionWaypoints(currentWP,wp_dist,xtrack_error,cog,target_bearing);								
			 	 }
			 	 break;

				 default:
				 break;
			 }
		 }
	 }
}



//=================================================================================================

uint32_t MavlinkHandler::Get_Current_Average1(uint16_t cA)  {   // in 100*milliamperes (1 = 100 milliampere)
	
	Accum_mAh1(cA);
	
	if (bat1.avg_cA < 1){
		bat1.avg_cA = cA;  // Initialise first time
	}

	bat1.avg_cA = (bat1.avg_cA * 0.6666F) + (cA * 0.333F);  // moving average

	return bat1.avg_cA;
}

uint32_t MavlinkHandler::Get_Volt_Average1(uint16_t mV)  {

	if (bat1.avg_mV < 1) bat1.avg_mV = mV;  // Initialise first time

	// bat1.avg_mV = (bat1.avg_mV * 0.9) + (mV * 0.1);  // moving average
	bat1.avg_mV = (bat1.avg_mV * 0.6666) + (mV * 0.3333);  // moving average
	Accum_Volts1(mV);
	return bat1.avg_mV;
}

void MavlinkHandler::Accum_Volts1(uint32_t mVlt) {    //  mV   milli-Volts
	bat1.tot_volts += (mVlt / 1000);    // Volts
	bat1.samples++;
}


void MavlinkHandler::Accum_mAh1(uint32_t cAs) {        //  cA    100 = 1A
	if (bat1.ft) {
		bat1.prv_millis = millis() -1;   // prevent divide zero
		bat1.ft = false;
	}
	uint32_t period = millis() - bat1.prv_millis;
	bat1.prv_millis = millis();
	
	double hrs = (float)(period / 3600000.0f);  // ms to hours

	bat1.mAh = cAs * hrs;     //  Tiny cAh consumed this tiny period di/dt
	// bat1.mAh *= 100;        //  dA to mA
	bat1.mAh *= 10;           //  cA to mA
	bat1.mAh *= 1.0625;       // Emirical adjustment Markus Greinwald 2019/05/21
	bat1.tot_mAh += bat1.mAh;   //   Add them all in
}

//=================================================================================================