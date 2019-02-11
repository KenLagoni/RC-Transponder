# RC Transponder (Beacon Locator)
After spending hours/days/weeks searching for downed RC Planes, I decided to design a single piece of hardware which should be able to survive a crash and keep transmitting the GPS location.

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
The transponder in the plane was set to sent every second and the PC on the ground logged all the data.

![alt text](http://lagoni.org/Github/RCtransponder-pictures/RCtransponderV10-second-flight-test.png)

It worked very well, and there where no problem with the range.
To push the system to the limits, I then mounted the transponder from the plane in my car, and drove away. I found a point ~2km away which (almost line of sight) where the transponder still worked, but the RSSI was very low and some of the incomming packages had CRC errors:

![alt text](http://lagoni.org/Github/RCtransponder-pictures/RCtransponderV10-car-test.png)

## Low Power Test
TBD (Pending hardware version 11)

## Low Temperature Test
TBD (Pending hardware version 11)

