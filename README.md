# Welcome to Star Pointer

Star pointer is a project developed on Arduino Uno R3, using
- MPU 9250 IMU
- BN-220 GPS
- 2004 LCD

To detect the arduino's orientation in Azimutal coordinates (Azimuth and Elevation), then converted to Gallactical coordinates (namely, Right Ascension and Declination). The purpose is to assist one's navigation through the sky using a dobsonian telescope.

The project at this time is in a development stage:

- Calibration of MPU 9250 is done manually through different main files to be uploaded on the Arduino, doing only necessary Serial print for calibration datasets to be written. Calibration itself is done through ipython notebooks, as well as using Magneto V2 tool for the magnetometer. Calibration parameters are to be written back in arduino source file.

- Wiring is still on a breadboard ! Arduino shield is coming to move forward, then a 3D model of the final box which will contain the system will be built (and available, once done) and 3D printed.

![Picture of the breadboard assembly](.images/off_breadboard.jpg?raw=true "Current development stage")

- Gallactical coordinates, that was challenging on an Arduino only managing 4 bits float precision. A specific python notebook aimed at comparing the arduino implementation (written back in python) with the reference python library Astropy.

As for now, the code is not going to move much : on an arduino uno R3, it currently uses :

- 99% of the storage space available, knowing that the bootloader is still there
- 61% of the dynamic memory for the global variables.

Knowing that, you may now understand why different main files are there for the calibration :)

This work is using a lot of external sources which are all mentionned where they are used.

