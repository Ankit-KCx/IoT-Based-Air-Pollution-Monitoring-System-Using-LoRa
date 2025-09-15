// --- Libraries ---
#include <SoftwareSerial.h>
#include <DHT.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <TinyGPSPlus.h>

// --- Pins ---
#define SDS_RX_PIN 4
#define SDS_TX_PIN 3
#define GPS_RX_PIN 7
#define GPS_TX_PIN 6
#define DHT_PIN 5
#define MQ135_PIN A0
#define DHT_TYPE DHT11
#define RFM95_CS 10
#define RFM95_RST 9
#define RFM95_INT 2
#define RFM95_FREQ 433.0

// --- Objects ---
SoftwareSerial sdsSerial(SDS_TX_PIN, SDS_RX_PIN); // SDS011: TX, RX
SoftwareSerial gpsSerial(GPS_TX_PIN, GPS_RX_PIN); // GPS: TX, RX
TinyGPSPlus gps;
DHT dht(DHT_PIN, DHT_TYPE);
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// --- Timing ---
unsigned long lastReadTime = 0;
const unsigned long readInterval = 20000; // ⏱️ 20 seconds

// --- Encryption Key ---
const byte ENCRYPTION_KEY = 0xAB;

// --- SDS011 Data Structure ---
struct SDS011Data {
  float pm25;
  float pm10;
};

// --- Encrypt Function ---
String encryptData(const String& data) {
  String encrypted = "";
  for (int i = 0; i < data.length(); i++) {
    encrypted += (char)(data[i] ^ ENCRYPTION_KEY);
  }
  return encrypted;
}

// --- SDS011 Reader Function ---
bool readSDS011(SDS011Data &data) {
  unsigned long startMillis = millis();
  while (sdsSerial.available() < 10) {
    if (millis() - startMillis > 2000) {
      Serial.println("SDS011: No data or timeout.");
      return false;
    }
    delay(10);
  }

  byte buffer[10];
  sdsSerial.readBytes(buffer, 10);

  if (buffer[0] == 0xAA && buffer[1] == 0xC0 && buffer[9] == 0xAB) {
    int pm25_low = buffer[2];
    int pm25_high = buffer[3];
    int pm10_low = buffer[4];
    int pm10_high = buffer[5];

    data.pm25 = (pm25_high * 256 + pm25_low) / 10.0;
    data.pm10 = (pm10_high * 256 + pm10_low) / 10.0;
    return true;
  } else {
    Serial.println("SDS011: Invalid data frame.");
    return false;
  }
}

// --- Setup ---
void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("System Starting...");

  gpsSerial.begin(9600);
  dht.begin();

  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH); delay(100);
  digitalWrite(RFM95_RST, LOW);  delay(100);
  digitalWrite(RFM95_RST, HIGH); delay(100);

  if (!rf95.init()) {
    Serial.println("LoRa init failed!");
    while (1);
  }

  if (!rf95.setFrequency(RFM95_FREQ)) {
    Serial.println("Frequency set failed!");
    while (1);
  }

  rf95.setTxPower(13, false);
  Serial.println("Setup complete.");
}

// --- GPS Helper ---
void processGPS(unsigned long timeoutMs, float &lat, float &lon, bool &validFix) {
  lat = -1.0;
  lon = -1.0;
  validFix = false;
  unsigned long start = millis();

  while (millis() - start < timeoutMs) {
    while (gpsSerial.available()) {
      char c = gpsSerial.read();
      gps.encode(c);
    }

    if (gps.location.isUpdated() && gps.location.isValid()) {
      lat = gps.location.lat();
      lon = gps.location.lng();
      validFix = true;
      break;
    }
  }

  if (validFix) {
    Serial.print("Lat: "); Serial.print(lat, 6);
    Serial.print(" | Lon: "); Serial.println(lon, 6);
  } else {
    Serial.println("GPS: No valid location.");
  }
}

// --- Loop ---
void loop() {
  // Keep feeding GPS parser
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  if (millis() - lastReadTime >= readInterval) {
    lastReadTime = millis();
    Serial.println("\n--- Reading Sensors ---");

    // --- DHT11 ---
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    if (isnan(h) || isnan(t)) {
      Serial.println("DHT11: Failed.");
      h = -99.9; t = -99.9;
    } else {
      Serial.print("Humidity: "); Serial.print(h);
      Serial.print(" %, Temp: "); Serial.println(t);
    }

    // --- MQ-135 ---
    int mq135_raw = analogRead(MQ135_PIN);
    Serial.print("MQ135 Raw: "); Serial.println(mq135_raw);

    // --- SDS011 ---
    gpsSerial.end(); delay(100);
    sdsSerial.begin(9600); delay(100);
    SDS011Data sdsData;
    if (!readSDS011(sdsData)) {
      sdsData.pm25 = -99.9;
      sdsData.pm10 = -99.9;
    } else {
      Serial.print("PM2.5: "); Serial.print(sdsData.pm25);
      Serial.print(" ug/m3, PM10: "); Serial.println(sdsData.pm10);
    }
    sdsSerial.end(); delay(100);
    gpsSerial.begin(9600); delay(100);

    // --- GPS ---
    Serial.println("Reading GPS...");
    float latitude, longitude;
    bool gpsFixAvailable = false;
    processGPS(8000, latitude, longitude, gpsFixAvailable);

    // --- Prepare Data String ---
    String latStr = gpsFixAvailable ? String(latitude, 6) : "NA";
    String lonStr = gpsFixAvailable ? String(longitude, 6) : "NA";

    String dataString = String(h, 1) + "," + String(t, 1) + "," +
                        String(mq135_raw) + "," +
                        String(sdsData.pm25, 1) + "," + String(sdsData.pm10, 1) + "," +
                        latStr + "," + lonStr;

    // --- Encrypt ---
    String encrypted = encryptData(dataString);

    // --- Debug Output ---
    Serial.print("Plaintext: "); Serial.println(dataString);
    Serial.print("Encrypted (Hex): ");
    for (int i = 0; i < encrypted.length(); i++) {
      byte b = encrypted[i];
      if (b < 16) Serial.print("0");
      Serial.print(b, HEX);
      Serial.print(" ");
    }
    Serial.println();

    // --- Send Encrypted Packet via LoRa ---
    rf95.send((uint8_t*)encrypted.c_str(), encrypted.length());
    rf95.waitPacketSent();
    Serial.println("LoRa encrypted packet sent!");
  }

  delay(100);
}
