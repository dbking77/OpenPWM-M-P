OpenPWM-M-P
===========

A single channel motor driver for dc brushed motors utiliizing a attiny and a ZXBM5210 motor driver

At this time, this project is distributed under the TAPR Open Hardware License: http://www.tapr.org/ohl.html

These drivers use a resistor to control the max trip current. 

We assume that the desired trip current is right at the max for the driver, ow, 2 amps. 

Using the examples supplied in the data sheet, we would need a 0.25 ohm Rs resistor. Though, that resistor would need to be a 1 watt resistor. 

We would like to make the resistor smaller than a 1 watt surface mount resistor. So, we made the pad a 0805 pad. The max dissipation you can get out of a 0805 resistor is 1/3 watt. 

By altering VRef, we can alter the power disapated by the resistor. 

The resistor will see connect upto VRfef and ground.

If we select a lower cost resistor of 0.047 ohms ( http://www.digikey.com/scripts/DkSearch/dksus.dll?Detail&itemSeq=165686396 )

The equation for what the trip current(assuming that the trip current is 2) is:

ItripMax(2) = Vref / (Rs * 10)
2 = Vref / 0.47
Vref = 0.94V

The resistor should only see 0.04 watts if Vref is kept to 0.94.

Project owned by Michael Gregg(mgregg@-at-@michaelgregg.com) and Derr(dbking77@-at@gmail.com)

