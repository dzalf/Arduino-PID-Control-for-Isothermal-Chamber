# PID-Control-Chamber
Control code and electronics for the isothermic chamber of my PhD

**Welcome!**

I'm glad to share this PID control project, including the electronic design from the PCB and the related Arduino code.

This cooler was usewd to cool down a piezoelectric sensor to a minimum of 5 Celsius. The power delivery of the Peltier used is not high enough for cooling further, however for the purposes of this particular project that was more than enough. In onrder to increase the cooling capabilities use a higher power cooler BUT keep in mind that it must be rated to < 5A at 12V.

The board and code are broadly described as follows. Please check the code and schematic and if you have any questions leave a comment.

**ELECTRONIC BOARD**

This PID control is for the control of a relay rather than a MOSFET or Bipolar transistor. The relay is rated to 12V and 5A, is a miniature relay (give me some time to retrieve an exact model number). The PCB was originally drawn on Proteus 8.4/8.6. Despite its simplicity is a robust design that incorporates an I2C LCD, a rotary encoder, an NTC as the temperature sensor and a custom-made keyboard.

The PCB has a form factor as small as 40mm x 60 mm

![alt txt](https://github.com/dzalf/Arduino-PID-Control-for-Isothermal-Chamber/blob/master/PCB-files/PCB.png)


**ARDUINO CODE AND INSTRUMENTATION**

An Arduino Pro Micro Leonardo was powerful enough for this project.

Libraries used and general description:

a. The core of this project is the PID library <PID_V1.h> by [Brett Beauregard](https://github.com/br3ttb/Arduino-PID-Library). Further technical information about this library can be found [here](http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/).

I must say that his library works like a charm but it needs a lot of fiddling to tune up.

b. In addition my code uses interrupts to read the changes from the rotary encoder. I **literally** used a [salvaged](https://www.instagram.com/p/BYetgmXHe7p/) encoder from I microwave I burnt :fire: a bag of popcorn in and it was totalled afterwards (I almost evacuated the entire building after burning it :no_mouth:)

c. The instrumentation was developed with the GINOURMOUS help from the of [blog](https://www.allaboutcircuits.com/projects/measuring-temperature-with-an-ntc-thermistor/) of Joseph Corletto. 

d. The NTC is a 10kOhm thermistor from [Vishay](https://www.vishay.com/thermistors/list/product-29051/)

e. The Peltier cooler originally used can be found [here](https://uk.farnell.com/adaptive-thermal-management/ar-ar-019-12/peltier-cooler-air-to-air-15v/dp/2507250)

f. The classic DHT22 humidity sensor was used. I don't add a link becuase you knoiw where to find them. They are E V E R Y W H E R E!

**#TODO**

1. Upload a clear schematic for the keyboard. However the pinout can be extracted from the code
2. Port the schematic to kiCAD
3. Cleanup and comment the code
4. Buy more beer...


Cheers! :beer:

dzalf :sunglasses:


