#include <TinyGPS++.h>
#include <math.h>

const double J2000 = 2451545.0; // Julian date for Jan 1, 2000 at 12:00:00 UTC


// GPS
static const int RXPin = 16, TXPin = 17;
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;

// SP0256A data bus pins
const int dataPins[] = {2, 4, 12, 13, 14, 15, 25, 26};

// SP0256A control pins
const int LRQPin = 27;
const int ALDPin = 32;

// 74HC573 latch control pins
const int latchClockPin = 33;
const int latchEnablePin = 34;

// Allophone data for numbers 0-9
const uint8_t numberAllophones[][2] = {
  {0x3C, 0x29}, // 0
  {0x30, 0x37}, // 1
  {0x32, 0x00}, // 2
  {0x33, 0x00}, // 3
  {0x34, 0x00}, // 4
  {0x35, 0x00}, // 5
  {0x36, 0x00}, // 6
  {0x37, 0x00}, // 7
  {0x38, 0x00}, // 8
  {0x39, 0x00}  // 9
};

void setup() {
  Serial.begin(115200);
  Serial1.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);

  // Initialize SP0256A data bus pins
  for (int i = 0; i < 8; i++) {
    pinMode(dataPins[i], OUTPUT);
  }

  // Initialize SP0256A control pins
  pinMode(LRQPin, INPUT);
  pinMode(ALDPin, OUTPUT);

  // Initialize 74HC573 latch control pins
  pinMode(latchClockPin, OUTPUT);
  pinMode(latchEnablePin, OUTPUT);

  digitalWrite(latchEnablePin, HIGH);
  digitalWrite(latchClockPin, LOW);
  digitalWrite(ALDPin, HIGH);
}

void sendAllophone(uint8_t allophone) {
  // Write allophone data to data bus pins
  for (int i = 0; i < 8; i++) {
    digitalWrite(dataPins[i], (allophone >> i) & 1);
  }

  // Load allophone data into latch
  digitalWrite(latchEnablePin, LOW);
  digitalWrite(latchClockPin, HIGH);
  delayMicroseconds(1);
  digitalWrite(latchClockPin, LOW);
  digitalWrite(latchEnablePin, HIGH);

  // Load allophone data into SP0256A
  digitalWrite(ALDPin, LOW);
  while (digitalRead(LRQPin) == LOW) {} // Wait for LRQ to go HIGH
  digitalWrite(ALDPin, HIGH);
  while (digitalRead(LRQPin) == HIGH) {} // Wait for LRQ to go LOW
}

void speakNumber(int number) {
  int tens = number / 10;
  int ones = number % 10;

  if (tens > 0) {
    sendAllophone(numberAllophones[tens][0]);
    if (numberAllophones[tens][1] != 0x00) {
      sendAllophone(numberAllophones[tens][1]);
    }
  }

  sendAllophone(numberAllophones[ones][0]);
  if (numberAllophones[ones][1] != 0x00) {
    sendAllophone(numberAllophones[ones][1]);
  }
}

void loop() {
  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
    if (gps.time.isUpdated()) {
      int hour = gps.time.hour();
      int minute = gps.time.minute();
      int second = gps.time.second();

      // Adjust the time for the desired timezone and daylight saving time
      hour += 2; // Add 2 hours for example
      if (hour >= 24) {
        hour -= 24;
      }

      // Speak the hour
      speakNumber(hour);
      delay(500);

      // Speak the minute
      speakNumber(minute);
      delay(500);

      // Speak the second
      speakNumber(second);
      delay(500);

      // Wait for a minute before announcing the time again
      delay(60000);
    }
  }
}





double toJulianDate(int year, int month, int day, int hour, int minute, int second) {
  double a = floor((14 - month) / 12.0);
  double y = year + 4800 - a;
  double m = month + 12 * a - 3;
  double jd = day + floor((153 * m + 2) / 5.0) + 365 * y + floor(y / 4.0) - floor(y / 100.0) + floor(y / 400.0) - 32045;
  jd += (hour - 12) / 24.0 + minute / 1440.0 + second / 86400.0;
  return jd;
}

double toJulianCentury(double jd) {
  return (jd - J2000) / 36525.0;
}

double meanSolarLongitude(double t) {
  double L0 = fmod(280.46646 + t * (36000.76983 + 0.0003032 * t), 360.0);
  return L0;
}

double meanSolarAnomaly(double t) {
  double M = fmod(357.52911 + t * (35999.05029 - 0.0001537 * t), 360.0);
  return M;
}

double equationOfCenter(double t, double M) {
  double C = 1.914602 - t * (0.004817 + 0.000014 * t) + sin(M * M_PI / 180.0) * (2.000469 + 0.020064 * t);
  return C;
}

double eclipticLongitude(double L0, double C) {
  double L = fmod(L0 + C, 360.0);
  return L;
}

double obliquityCorrection(double t) {
  double e0 = 23.0 + 26.0 / 60.0 + 21.448 / 3600.0 - t * (46.815 + t * (0.00059 - t * 0.001813));
  double omega = 125.04 - 1934.136 * t;
  double e = e0 + 0.00256 * cos(omega * M_PI / 180.0);
  return e;
}

double solarZenithAngle(double latitude, double L, double e) {
  double delta = asin(sin(e * M_PI / 180.0) * sin(L * M_PI / 180.0)) * 180.0 / M_PI;
  double H = -0.83 - 2.076 * sqrt(10.0) / 60.0; // Altitude at sunrise and sunset
  double cosH = (sin(H * M_PI / 180.0) - sin(latitude * M_PI / 180.0) * sin(delta * M_PI / 180.0)) / (cos(latitude * M_PI / 180.0) * cos(delta * M_PI / 180.0));
  double h = acos(cosH) * 180.0 / M_PI;
  return h;
}

double solarTransit(double t, double L, double M) {
  double n = t - (M - L) / 360.0;
  double J_transit = J2000 + n;
  return J_transit;
}

void sunriseAndSunset(double J_transit, double h, double longitude, double &J_rise, double &J_set) {
  J_set = J_transit + h / 360.0 - longitude / 360.0;
  J_rise = J_transit - h / 360.0 - longitude / 360.0;
}


int japaneseTimeSystem(double latitude, double longitude, int year, int month, int day, int hour, int minute, int second) {
  double jd = toJulianDate(year, month, day, hour, minute, second);
  double t = toJulianCentury(jd);
  double L0 = meanSolarLongitude(t);
  double M = meanSolarAnomaly(t);
  double C = equationOfCenter(t, M);
  double L = eclipticLongitude(L0, C);
  double e = obliquityCorrection(t);

  double h = solarZenithAngle(latitude, L, e);
  double J_transit = solarTransit(t, L, M);
  double J_rise, J_set;
  sunriseAndSunset(J_transit, h, longitude, J_rise, J_set);

  int sunriseHour = int((J_rise - floor(J_rise)) * 24.0);
  int sunsetHour = int((J_set - floor(J_set)) * 24.0);

  double dayLength = (J_set - J_transit) * 24.0; // In hours
  double nightLength = 24.0 - dayLength;

  int daylightHours = int(dayLength / 6.0);
  int nightHours = int(nightLength / 6.0);

  // Calculate the traditional Japanese time
  int traditionalHour;
  if (hour >= sunriseHour && hour < sunsetHour) {
    traditionalHour = (hour - sunriseHour) / daylightHours;
  } else {
    if (hour >= sunsetHour) {
      traditionalHour = (hour - sunsetHour) / nightHours + 6;
    } else {
      traditionalHour = (hour + 24 - sunsetHour) / nightHours + 6;
    }
  }

  return traditionalHour;
}

