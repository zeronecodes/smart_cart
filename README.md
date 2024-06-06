# Smart Cart Navigation System

## Overview

This project, developed during my second year at university (2021), was an ambitious attempt to create a smart cart capable of navigating to users using GPS. The cart could be called using a Bluetooth connection from an Android device, initiating the navigation process. The system was designed with an Arduino Mega 2560, which controlled the motor driver to receive PWM commands for movement.

## Key Components

### Hardware
- **Arduino Mega 2560**: The central microcontroller unit.
- **Motor Driver**: Receives PWM commands from the Arduino to control the cart's motors.
- **GPS Module**: Provides the geographical location of the cart.
- **Bluetooth Module (HC-06)**: Allows communication with an Android device.
- **Compass (Adafruit HMC5883)**: Helps in determining the cart's heading.
- **Ultrasonic Sensors**: Used for obstacle detection and distance measurement.
- **Motors**: Control the movement of the cart.

### Software Libraries
- `SoftwareSerial.h`: Enables serial communication on other digital pins of the Arduino.
- `TinyGPS.h`: Handles GPS data parsing.
- `Wire.h`: Used for I2C communication.
- `Adafruit_Sensor.h` and `Adafruit_HMC5883_U.h`: Used for compass functionality.

## Important Definitions

- **Declination Angle**: The error of the magnetic field in your location. For accurate compass readings, you must add this angle. Mine is 0.23 radians.
- **GPS Update Interval**: Set to 1000 milliseconds to ensure regular updates.

## Data Structures

- **GeoLoc**: Stores GPS coordinates (latitude and longitude).
- **AppLoc**: Stores coordinates received from the Bluetooth connection and a flag indicating if the data is new.
- **MyCompass**: Stores compass data (X, Y, Z, and H values).

## Motor and Sensor Setup

Pins for motors and ultrasonic sensors are defined and set up. Functions to control the cart's direction (left, right, front, back, stop) and speed are implemented.

## GPS and Bluetooth

The cart's location is continuously updated using the GPS module. Bluetooth communication is handled to receive the user's location, which is then parsed and stored.

## Compass Calibration

Calibration of the compass is necessary to get accurate heading data. This involves setting the declination angle and correcting the compass readings.

## Navigation Logic

The cart navigates to the user's location by:
1. Retrieving the current GPS coordinates.
2. Calculating the bearing and distance to the target location.
3. Adjusting the motor speeds and directions to steer the cart.

## Libraries

```cpp
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
```

## Future Development
With years of experience, I would now opt to enhance the system's accuracy by replacing the GPS module with a UWB module assisted by an RTK GPS system. This change would provide centimeter-level accuracy, significantly improving the navigation capabilities of the smart cart.
