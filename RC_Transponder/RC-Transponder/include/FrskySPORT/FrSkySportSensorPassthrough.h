/*
  FrSky DIY sensor class for Passthrough data.
  (c) Lagoni 
  Not for commercial use
*/

#ifndef _FRSKY_SPORT_SENSOR_PASSTHROUGH_H_
#define _FRSKY_SPORT_SENSOR_PASSTHROUGH_H_

#include "FrSkySportSensor.h"
#include "RingBuf.h" //FIFO
#include <math.h> // For roundf


#define PASSTHROUGH_DEFAULT_ID ID28

#define PASSTHROUGH_HIGH_PRIORITY_DATA_COUNT 3 // Attitude, Status Text and parameters
#define PASSTHROUGH_LOW_PRIORITY_DATA_COUNT  8 // All other messages

#define PASSTHROUGH_TEXT_FIFO_SIZE 256
#define PASSTHROUGH_PARAM_FIFO_SIZE 10

#define PASSTHROUGH_TEXT_MSG_ID			0x5000
#define PASSTHROUGH_AP_STATUS_ID		0x5001
#define PASSTHROUGH_GPS_STATUS_ID		0x5002
#define PASSTHROUGH_BAT1_STATUS_ID      0x5003
#define PASSTHROUGH_HOME_STATUS_ID      0x5004
#define PASSTHROUGH_VYAW_STATUS_ID      0x5005
#define PASSTHROUGH_ATTITUDE_ID		    0x5006
#define PASSTHROUGH_PARAMS_ID		    0x5007
#define PASSTHROUGH_BAT2_STATUS_ID      0x5008
#define PASSTHROUGH_MISSION_STATUS_ID   0x5009 
#define PASSTHROUGH_SERVO_CHANNELS_ID   0x50F1 // Not yet implemented
#define PASSTHROUGH_VFR_HUD_ID			0x50F2 
#define PASSTHROUGH_WIND_ESTIMATE_ID	0x50F3 // Not yet implemented




#define PASSTHROUGH_TEXT_DATA_PERIOD	      10  // Fast as possible.
#define PASSTHROUGH_AP_STATUS_PERIOD		 500
#define PASSTHROUGH_GPS_STATUS_PERIOD		1000
#define PASSTHROUGH_BAT1_STATUS_PERIOD      1000
#define PASSTHROUGH_HOME_STATUS_PERIOD       500
#define PASSTHROUGH_VYAW_STATUS_PERIOD       500
#define PASSTHROUGH_ATTITUDE_PERIOD			  50  // Max 20 Hz,since OpenTX radio screens don't update faster.
#define PASSTHROUGH_PARAMS_PERIOD			  10  // Fast as possible.
#define PASSTHROUGH_BAT2_STATUS_PERIOD      1000
#define PASSTHROUGH_MISSION_STATUS_PERIOD   1000  
#define PASSTHROUGHSERVO_CHANNELS_PERIOD    1000  // Not yet implemented
#define PASSTHROUGH_VFR_HUD_PERIOD			 500  
#define PASSTHROUGH_WIND_ESTIMATE_PERIOD    1000  // Not yet implemented



class FrSkySportSensorPassthrough : public FrSkySportSensor
{
  public:
    FrSkySportSensorPassthrough(SensorId id = PASSTHROUGH_DEFAULT_ID);
    void setDataTextMSG(char* text, uint8_t severity);											   // 0x5000 - Text messages Mavlink MSG #253
	void setDataAPStatus(uint8_t flightMode, uint8_t simple, uint8_t landComplete, uint8_t armed); // 0x5001 - AP STATUS - From Mavlink Heartbeat #0
	void setDataGPSStatus(uint8_t numberOfSat, uint8_t fix, int32_t lattidue, int32_t longitude, uint16_t hdop, uint8_t advanced, int32_t altitudeMSL); // 0x5002 - GPS STATUS
	void setDataBattery1Status(float voltage, float current, float totalCurrentSinceStart);        // 0x5003 - Battery1 status.		
	void setDataHomeStatus(float distanceToHome, float altitudeDiffToHome, float angleToHome);	   // 0x5004 - Home status with parameters as floats.		
	void setDataHomeStatus_FromMavlink(int32_t homeLatitude, int32_t homeLongitude, int32_t currentLatitude, int32_t currentLongitude, int32_t altitudeAboveGround); // 0x5004 - Home status with parameters from Mavlink #24
	void setDataVYAWStatus(float vclimb, float groundSpeed, float heading);	// 0x5005 - Text messages Mavlink MSG #74		
	void setDataAttitude(float roll, float pitch);	                        // 0x5006 - ATTITUDE Mavlink MSG #30 (this needs 10-20Hz to get good ADI on radio).
	
	void setDataParameterFrameType(uint32_t mav_Type);					    // 0x5007 -	Frame Type
	void setDataParameterBatteryPack1Capacity(uint32_t bat1_cap);		    // 0x5007 -	Battery 1 Capacity 
	//void setDataParameterBatteryReserveCapacity(uint32_t bat_res_cap);	// 0x5007 -	Not yet implemented in Arduplane
	//void setDataParameterLowBatteryTriggerLevel(float voltage);			// 0x5007 -	Not yet implemented in Arduplane	
	void setDataParameterBatteryPack2Capacity(uint32_t bat2_cap);		    // 0x5007 -	Battery 2 Capacity
	void setDataParameterNumberOfWaypointInMission(uint32_t num_way);	    // 0x5007 -	Number of waypoints in mission.
	
	void setDataBattery2Status(float voltage, float current, float totalCurrentSinceStart);        // 0x5008 - Battery2 status.		
	void setDataMissionWaypoints(uint16_t wpNumber, uint16_t distanceToNextWaypoint, float xTrackError, float cog, int16_t nextWPbearing);      // 0x5009 - Mission waypoints
	void setDataVFRHUD(float airspeed, uint16_t throttle, float baroAltitude);                     // 0x50F2 -  VFR_HUD
	
	
	
	
    virtual void send(FrSkySportSingleWireSerial& serial, uint8_t id, uint32_t now);
    virtual uint16_t decodeData(uint8_t id, uint16_t appId, uint32_t data);
    
  private:
	bool nextSensorType = HIGH; // Start with High priority sensors first.
	uint8_t LowPrioritySensorDataIdx=0;
  
	// #0x5000 - Status Text data.
	uint32_t textTime;
	uint32_t temp_payload=0;
	RingBuf<uint32_t, PASSTHROUGH_TEXT_FIFO_SIZE> textFIFO;
	uint32_t textPayload=0;
	int8_t msgRetransmit=0;
  
  
	// #0x5001 - AP STATUS
	uint32_t apStatusTime;
	bool apStatusDataReady;
	uint32_t apStatusPayload=0; 

	// #0x5002 - GPS STATUS
	uint32_t gpsStatusTime;
	uint16_t hdop_temp;
	int32_t amsl_temp, amsl;
	bool gpsStatusDataReady;
	uint32_t gpsStatusPayload=0;

	// #0x5003 - Battery 1 Status
	uint32_t bat1StatusTime;
	uint16_t bat_amps;
	bool bat1StatusDataReady;
	uint32_t battempPayload;
	uint32_t bat1StatusPayload=0;

	// #0x5004 - Home Status
	uint32_t homeStatusTime;
	uint16_t home_dist;
	int16_t home_alt, home_arrow;
	float cur_lat, cur_lon, hom_lat, hom_lon, lat1, lon1, lat2, lon2, a, c, az, home_angle, dLat , dLon, dis, alt;
	bool homeStatusDataReady;
	uint32_t homeStatusPayload=0;

	// #0x5005 - VELANDYAW
	uint32_t vyawStatusTime;
	float pt_vy, pt_vx, pt_yaw;
	bool vyawStatusDataReady;
	uint32_t vyawStatusPayload=0;
	
	// #0x5006 - Attitude data.
	uint32_t attitudeTime;
	float rollDegrees, pitchDegrees;
	bool attitudeDataReady;
	uint32_t attitudePayload=0;
	
	// #0x5007 - Parameter data.
	void packParameterPayload(uint32_t data, uint8_t paramID);	
	enum PassthroughParam : uint8_t {
		NONE =                0,
		FRAME_TYPE =          1,
		BATT_FS_VOLTAGE =     2,
		BATT_FS_CAPACITY =    3,
		BATT_CAPACITY_1 =     4,
		BATT_CAPACITY_2 =     5,
		TELEMETRY_FEATURES =  6
	};
	uint32_t paramsTime;
	bool paramsDataReady;
	uint32_t paramsPayload=0;
	uint32_t tempPayload;
	RingBuf<uint32_t, PASSTHROUGH_PARAM_FIFO_SIZE> paramsFIFO;
	int8_t paramsRetransmit=0;

	// #0x5008 - Battery 2 Status
	uint32_t bat2StatusTime;
	bool bat2StatusDataReady;
	uint32_t bat2StatusPayload=0;

	// 0x5009 - Mission Waypoints
	uint32_t missionStatusTime;
	bool missionStatusDataReady;
	uint16_t pt_ms_dist;
	float pt_ms_xtrack,pt_ms_target_bearing;
	int32_t angle, arrowStep;
	int8_t pt_ms_offset;
	uint32_t missionStatusPayload=0;

    // 0x50F1 // Not yet implemented
  
	// #0x50F2 - VFR HUD
	uint32_t vfrHudTime;
	bool vfrHudDataReady;
	float    pt_air_spd, pt_bar_alt;       // dm/s
	uint16_t pt_throt;					   // 0 to 100%
	uint32_t vfrHudPayload=0;	

    // #0x50F3 // Not yet implemented
	
    // Private helper functions.
	uint32_t packBatteryPayload(float voltage, float current, float totalCurrentSinceStart); // can be used by bat1 or bat2 to pack payload.
	uint32_t createMask(uint8_t lo, uint8_t hi); // need for bit32Pack.
	uint32_t bit32Pack(uint32_t dword ,uint8_t displ, uint8_t lth); // from zs6bu MavlinkToPassthru
	uint16_t prep_number(int32_t number, uint8_t digits, uint8_t power); // From Arducopter 3.5.5 code
	int16_t Add360(int16_t arg1, int16_t arg2);
	float wrap_360(int16_t angle);
};
#endif // _FRSKY_SPORT_SENSOR_PASSTHROUGH_H_