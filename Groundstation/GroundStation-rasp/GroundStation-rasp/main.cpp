/*
 * GroundStation-rasp.cpp
 *
 * Created: 10-04-2019 14:22:06
 * Author : klo
 */ 
#include "E28-2G4M20S.h"
#include "bcm2835.h"
#include <stdio.h>
#include <string>
#include "RFService.h"

// for UDP
#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>

// For File io
#include <iostream>
#include <fstream>

//time
#include <time.h>
#include <inttypes.h>


using namespace std;

#define NPACK 1
#define PORT 9930

E28_2G4M20S *Radio = nullptr;
RFService *RadioService = nullptr;
char *dest_ip;
int sendUDPData(char *buf, size_t len );
void sendUDPDataProtocol(RadioData_t *data);

void TimerService();

long int last_heartbeat;
long int heartbeat_difference;
struct timespec gettime_now;

FILE * SystemLogFile;
string logPath;

enum PayloadID_t{
	RADAR_APPLICATION_ID=0x00,
	RADIO_DATA_TO_RF,
	RADIO_DATA_TO_PC
}PayloadID;

enum ApplicationCMD_t{
	HW_REPLY=0x00   // Application sends this to test if HW is on this USB port, HW must reply with same message.
}ApplicationCMD;


/* Replace with your library code */
int main(int argc, char *argv[])
{
	printf("Number of input argument(s):%d\n",argc);	
	for(int a=0;a<argc;a++)
	{	
		printf("Input argument1:%s:\n",argv[a]);
	}
	
	if (argc != 3)
	{
		printf("Error, wrong number of arguments entered\n");
		return EXIT_FAILURE;
	}

	dest_ip = argv[1];
	string s(argv[2]);
	logPath = s;
	string systemLog = logPath + "/" +  "aTextFile.txt"; 
	
	if (!bcm2835_init())
	{
		printf("bcm2835_init failed. Are you running as root??\n");
		return 1;
	}

	if (!bcm2835_spi_begin())
	{
	    printf("bcm2835_spi_begin failed. Are you running as root??\n");
	    return 1;
	}

	printf("Starting Groundstation\n");
	//			E28_2G4M20S(      chipSelectPin  ,       resetPin      ,       busyPin       ,      dio1Pin        , dio2Pin, dio3Pin,     txEnablePin     ,     rxEnablePin     ,        ledPin)
	Radio = new E28_2G4M20S( RPI_BPLUS_GPIO_J8_24, RPI_BPLUS_GPIO_J8_12, RPI_BPLUS_GPIO_J8_13, RPI_BPLUS_GPIO_J8_16,   0    ,    0   , RPI_BPLUS_GPIO_J8_11, RPI_BPLUS_GPIO_J8_07, RPI_BPLUS_GPIO_J8_37);
	printf("New E28 module...\n");
	Radio->Init();
	printf("Init of E28 module complete!\n");
	Radio->SetRXMode(false); // no timeout
	uint16_t test = Radio->GetFirmwareVersion();
	printf("Firmware Version %d\n", test);
	

	
	
	
	// Prepare logging:	
	SystemLogFile = fopen(systemLog.c_str(),"a");
	fprintf(SystemLogFile,"Test!\n");                   // String to file
//	fprintf (pFileXML,"%.2x",character);              // Printing hex value, 0x31 if character= 1
	fflush(SystemLogFile);
	//fclose (pFileTXT); // must close after opening

	RadioService = new RFService(Radio, logPath);

	//SETUP HEARTBEAT TIMER
	clock_gettime(CLOCK_REALTIME, &gettime_now);
	last_heartbeat = gettime_now.tv_nsec;

	
	do{
		RadioService->ServiceHandel();
		if(RadioService->Available() > 0)  
		{
			RadioData_t *newmsg = RadioService->GetDataIncludingRSSI();
			
			//printf("\nUTC Time: %d\n",((Telegram_MSG_1 *)msg)->GetUTCTime());
			//printf("Battery Voltage %.2f V\n",((Telegram_MSG_1 *)msg)->GetBatteryVoltage());
			
			sendUDPDataProtocol(newmsg);
		}
		TimerService();
		
//		bcm2835_delay(1000);
//		printf("sending...");
//		char buf[] = "I'm a UDP god 2";
//		send(buf, strlen(buf));
//		printf("done\n");
		
		//RFService();
/*		bcm2835_gpio_write(RPI_BPLUS_GPIO_J8_37, HIGH);
		bcm2835_delay(500);
		bcm2835_gpio_write(RPI_BPLUS_GPIO_J8_37, LOW);
		bcm2835_delay(500);*/
	}while(1);
		
	return 0;
}

void OneSecondService()
{
	RadioData_t *beacon = new RadioData_t;
	beacon->payload[0]=RADAR_APPLICATION_ID; //
	beacon->payload[1]=HW_REPLY; //
	beacon->payloadLength=2;
	sendUDPDataProtocol(beacon);
	
	RadioService->PingActivePlanes();
}

void TimerService(){
		//---------------------------
		//---------------------------
		//----- HEARTBEAT TIMER -----
		//---------------------------
		//---------------------------
		clock_gettime(CLOCK_REALTIME, &gettime_now);
		heartbeat_difference = gettime_now.tv_nsec - last_heartbeat;		//Get nS value
		if (heartbeat_difference < 0)
		{
			heartbeat_difference += 1000000000;				//(Rolls over every 1 second)
		}
		
		if (heartbeat_difference > 1000000)					//<<< Heartbeat every 1mS
		{
			//-------------------------
			//----- 1mS HEARTBEAT -----
			//-------------------------
			last_heartbeat += 1000000;						//<<< Heartbeat every 1mS
			if (last_heartbeat > 1000000000)				//(Rolls over every 1 second)
			{
				last_heartbeat -= 1000000000;
				OneSecondService();
			}
		} //if (heartbeat_difference > 1000000)
}


/*
int receive(void)
{
	struct sockaddr_in si_me, si_other;
	int s, i, slen=sizeof(si_other);
	char buf[BUFLEN];
	
	if ((s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1)
	diep("socket");
	
	memset((char *) &si_me, 0, sizeof(si_me));
	si_me.sin_family = AF_INET;
	si_me.sin_port = htons(PORT);
	si_me.sin_addr.s_addr = htonl(INADDR_ANY);
	if (bind(s, &si_me, sizeof(si_me))==-1)
		diep("bind");
	
	for (i=0; i<NPACK; i++)
	{
		if (recvfrom(s, buf, BUFLEN, 0, &si_other, &slen)==-1)
			diep("recvfrom()");
			
		printf("Received packet from %s:%d\nData: %s\n\n",
		inet_ntoa(si_other.sin_addr), ntohs(si_other.sin_port), buf);
	}
	
	close(s);
}*/
/* diep(), #includes and #defines like in the server */
     
void sendUDPDataProtocol(RadioData_t *data)
{
		uint8_t output[data->payloadLength+4];
	
		output[0]=0x1E; // Start byte
		output[1]=(char)data->payloadLength+2; // Length (output.payloadlength +2 byte CRC;
		memcpy(&output[2], &data->payload[0], data->payloadLength);

		uint16_t temp_crc = Radio->CalculateCRC(&output[2], data->payloadLength);

		output[data->payloadLength+2] = (uint8_t)((temp_crc >>  8) & 0xFF);  // CRC
		output[data->payloadLength+3] = (uint8_t)(temp_crc & 0xFF);		  // CRC

		sendUDPData((char *)output, data->payloadLength+4);		
}	 
	 
	 
int sendUDPData(char *buf, size_t len)
{
	struct sockaddr_in si_other;
	int s, i, slen=sizeof(si_other);
//	char buf[BUFLEN];
	
	if ((s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1){
		perror("socket");
		exit(1);
	}
	
	memset((char *) &si_other, 0, sizeof(si_other));
	si_other.sin_family = AF_INET;
	si_other.sin_port = htons(PORT);
	if (inet_aton(dest_ip, &si_other.sin_addr)==0) {
		fprintf(stderr, "inet_aton() failed\n");
		exit(1);
	}
	else{
	//	printf("Destination IP:%s",)
		
	}
	
	for (i=0; i<NPACK; i++) {
//		printf("Sending packet %d\n", i);
		
//		sprintf(buf, "I'm a UDP GOD! \n");	
		if (sendto(s, buf, len, 0, (struct sockaddr*)&si_other, slen)==-1)
		{
			perror("sendto()");
			exit(1);
		}
	
	}
	
	close(s);
	return 0;
}