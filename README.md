# GpsClock
Clock library for Ardunio that provides time from GPS, Traditional Japanese time, and sun position.

# GpsClock Library

The GpsClock library provides an easy-to-use interface for calculating the Traditional Japanese hour using GPS data. This library is compatible with Arduino-compatible boards, including the ESP32 and ESP8266.

## Features

- Easy-to-use interface for obtaining GPS data
- Calculates the Traditional Japanese hour based on GPS time, date, and location
- Compatible with various Arduino-compatible boards, including ESP32 and ESP8266

## Installation

1. Download the library as a ZIP file.
2. In the Arduino IDE, go to Sketch > Include Library > Add .ZIP Library, and select the downloaded ZIP file.
3. The GpsClock library should now be available in the Arduino IDE under Sketch > Include Library.

## Usage

1. Include the GpsClock library at the beginning of your sketch:

```cpp
#include "GpsClock.h"

    Define the pin connections for the GPS module:

cpp

#define TX_PIN 17
#define RX_PIN 16

    Create a GpsClock object with the defined pin connections:

cpp

GpsClock clock(TX_PIN, RX_PIN);

    In the setup() function, initialize the GpsClock:

cpp

clock.begin();

    In the loop() function, update the GpsClock and get the Traditional Japanese hour:

cpp

clock.update();
int traditionalJapaneseHour = clock.getTraditionalJapaneseHour();

    Use the Traditional Japanese hour as needed (e.g., display it, speak it using another library, etc.).

Contributing

We welcome contributions to improve the GpsClock library. If you find any bugs or have suggestions for enhancements, please open an issue on the project's GitHub page.
License

