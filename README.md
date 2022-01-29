### Introduction
Continuous technological progress means that devices that were once unavailable to the average consumer are becoming cheaper and more common.
Such devices are microcontrollers, also known as microcomputers. Microcontrollers have become so common that it is hard to find a device that does not have even a single chip. The number of different versions of microcontrollers simplifies the selection for a specific need.
 There are chips that differ in architecture, number of cores, computing power, word size (8, 16, 32-bit), clock speed, memory size, number of I / O ports. They can be equipped with circuits such as comparators and converters
A / C and C / A, RTC systems, PWM systems, RTC systems, Watchdog, or transmission controllers (UART, SPI, I2C, USB, CAN, etc.).
Today, thanks to, among others, a company such as Arduino, they are available to every hobbyist at an affordable price.
Arduino is an open-source platform , thanks to which members of the community can constantly improve various aspects related to the functioning of this environment.
The use of the Arduino platform for activities related to astronomy is a kind of demonstration of some of the possibilities that lie dormant in open-source software and using popular sensors dedicated to it, you can create devices for various purposes.
On the Internet you can come across a multitude of projects, from 3D printers to projects related to the control of LED strips. However, the subject of astronomy is not often brought up in the developer and developer community, and the number of projects available is small.
Therefore, it was decided to use Arduino. As part of this work, it was undertaken to build a device to facilitate the search for stars in the sky.

### Objective of the work
The aim of this work is to design and program a device capable of calculating the position of a star in the sky and then facing it. The platform chosen to control the device and its functions is Arduino.
The basic function of the device will be to indicate in the sky a place suitable for observation of the star entered by the user. 
In order to be able to use the device correctly, the user will have to enter a series of information related to the star he wants to observe. A person interested in observing the star will have to enter data describing the coordinates of the star, after which it is possible to calculate its position in relation to the observer.
These data are right ascension and declination of the celestial body. The result of the calculation will be displayed on the TFT display and the laser will face the star after turning.
The device will be powered by lithium-ion cells. The device is equipped with two DC motors that ensure rotation in the horizontal and vertical planes. Encoders will be used to read the shaft position. Both motors will be controlled by the same controller that is able to operate them simultaneously. Interaction with the user will be possible using the remote control.For this purpose, an infrared receiver is mounted on the device. After turning on the device, a menu will appear on the TFT display where the user will be able to select further options, such as:
  1. calibration,
  1. introduction of right ascension and star declination,
  1. introduction of offsets,
  1. the "find a star" option,
  1. information about the previous search result.

The user will be able to search for a star once without tracking it or use the follow option. This option allows the star to be tracked continuously on the basis of data from the real time clock.
It will be possible to charge the device with a standard charger with a rated voltage of 9 or 12 V.
