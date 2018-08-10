# Arduino-remote-control
Arduino based multi channel remote control for aeromodelism with telemetry and PS2 controller.
Based on arduino pro mini and NRF20L01 modules. Uses a PS2 remote controller as input.
Features multiple channels and memory for saving multiple models. Dual rates, throttle cut, on-fly default position set up (in case of signal lost), telemetry for measuring remote battery voltage and some other things.
If you have any questions email me to simonb2230@gmail.com

Based on a few nice projects from:
http://www.billporter.info/2010/06/05/playstation-2-controller-arduino-library-v1-0/
https://www.flitetest.com/articles/custom-arduino-2-4ghz-rc-transmitter
http://www.electronoobs.com/eng_robotica_tut5_2_1.php

and probably some other examples that I can't remember.

The power for the remote is from a single LiIon battery from a portable charger (using the same charging circuit)
The power for the receiver comes from the BEC of the ESC. The resistive divider on the receiver is calculated for a 3S Lipo battery.

The NRF modules need a 1uf ceramic SMD cap soldered directly on the PCB of the module itself

The range tested on ground is about 800m
