# RC Transponder (Locator Beacon)
After spending hours/days/weeks searching for lost RC Planes, I decided to design a single piece of hardware which should be able to survive a crash and keep transmitting the GPS location.

## Design
The latest version is version 11, but I'm pending the new hardware.

### Goals
The hardware must:

1. Get GPS coordinates from on-board GPS sensor.
2. Keep operating if power from plane is lost. (Have its own battery)
3. Continue to operate after a crash for minimum of 24h with a beacon transmitted every 30 second.
4. Recharge onboard battery from planes power supply.
5. Transmit GPS position on license free band up to 2km range.
6. Be firmware upgradeable from USB.
7. Operate as Frsky GPS sensor on SMART Port when powered from plane.
8. Relay other transponder beacons to ground via Frsky SMART Port. (Using other planes to search/listen for downed planes beacon.)
9. Be of smallest/lightest size as possible.
10. Have embedded antenna in PCB if possible.
11. Plug into PC via USB to display other transponders. 

### CPU (ATSAMD21 - ARM Cortex-M0+):
I started out using the standard Arduino ATMEGA-328P 8-bit microprocessor, however after working with the Arduino MKRZERO (32-bit ARM Cortex-M0+) I quickly changed to this platform. It all-ready had a USB bootloader. 

### Radio module (EBYTE E28-2G4M20S 100mW Lora):
I started out using a 433MHz GFSK 100mW module, but the size of the antenna and the complexity of the hardware made me look for other modules.

In order to reduce size of the antenna, I tired the 2.4GHz modules from EBYTE.
I tested the EBYTE E28-2G4M20S (100mW) found that when using Lora modulation with SF7, BW= 400KHz and CR=4/5, I could get a range of 2km.
 
### GPS Sensor selection (Sierra Wireless PA6H):
The PA6H from Global Tech (now Sierra Wireless) was the smallest GPS I could find with a decent chip antenna. (UBLOX CAM-M8 was not tested but could also be relevant).
The PA6H only transmit NMEA strings, so the CPU has some extra Work here.


## Hardware Layout and manufacturing
Schematic and layout is done in Cadence/Allegro. The version 10 has a size of (44mmx20mm) and Version 11 is (43.5mmx18mm).

10 prototypes PCB was ordered with stencil and then it's "just" pick and place the components (I only made two).

Board after pick and place but before reflow (see [RocketScream for DIY reflow own](http://www.rocketscream.com/blog/product/tiny-reflow-controller/)).

![alt text](http://lagoni.org/Github/RCtransponder-pictures/RCtransponderV10-solderpaste-with-components.png)

After reflow:

![alt text](http://lagoni.org/Github/RCtransponder-pictures/RCtransponderV10-after-relow.png)

Then the top side is soldered (GPS/Radio/battery/connector and one LED) (MicroSD Card for scale):
![alt text](http://lagoni.org/Github/RCtransponder-pictures/RCtransponderV10-top.png)

Finished unit with heat shrink (alongside the first 433MHz prototype):
![alt text](http://lagoni.org/Github/RCtransponder-pictures/RCtransponderV10-top-vs-433.png)

![alt text](http://lagoni.org/Github/RCtransponder-pictures/RCtransponderV10-back.png)

## Range Testing
I did some tests at my local flying club. One transponder was mounted on a chair with a laptop on the field and the other was mounted in a plane.
The transponder in the plane was set to send every second and the PC on the ground logged all the data.

![alt text](http://lagoni.org/Github/RCtransponder-pictures/RCtransponderV10-second-flight-test.png)

It worked very well, and there were no problem with the range.
To push the system to the limits, I then mounted the transponder from the plane in my car, and drove away. I found a point ~2km away which (almost line of sight) where the transponder still worked, but the RSSI was very low and some of the incomming packages had CRC errors:

![alt text](http://lagoni.org/Github/RCtransponder-pictures/RCtransponderV10-car-test.png)

## Low Power Test
TBD (Pending hardware version 11)

## Low Temperature Test
TBD (Pending hardware version 11)

# Software
The Software reflects that I started out using Arduino MKRZERO and the Arduino program and then imported it to Atmel Studio.

## Radio driver (EBYTE E28-2G4M20S) (Semtech SX1280)
I was not able to find any Arduino libraries for the SX1280 chip, and thus I started out using the demo code from Semtech, which I had to make some changes to until I got something which worked.

## GPS NMEA decoder (GSP80Lite)
Was a library I once made for the Arduino ATMEGA328P, so i reused it and added some more features. It still only decode GPGGA strings, which means that Date, COG and speed is not read yet. (Used in the FrSky GPS sensor.)

## Frsky SPORT library
I used this great library from [pawelsky](https://www.rcgroups.com/forums/showthread.php?2245978-FrSky-S-Port-telemetry-library-easy-to-use-and-configurable) However the ATSAMD21 chip is not supported and furthermore, I don't use single wire serial.
I did some dirty changes to his library to get it to work with my hardware. One of which is the single wire serial, where I just disable the RX input when transmitting, to avoid loopback.
