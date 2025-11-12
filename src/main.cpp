#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>



bool timer1 = false;
bool timer2 = false;
int wifiLock = 0;  //TODO pass from wifi
int wifiUnlock = 0;//TODO passed from wifi
unsigned long previousMillis = 0;
const unsigned long interval = 30000; // 30 seconds
bool clientConnected = false;
bool timerA = true;  // Start with timer A active

int nowMIlli = 0;
int elaspeMilli = 0;
int wifiPowerInput = -80;  // TODO get this value from wifi esp32
int currentLockValue = 0;
bool lockStateChange = false;


bool onbardPinOn = false;
int scanTime = 5; // seconds
bool lock = false;
BLEScan* pBLEScan;
bool state = true;
int LOCK = 5;
int UNLOCK = 18;
bool seeKey = false;
int power = -100;
String inputString = "";
//String displayValue = "Not set yet";  // This is read-only



void isActionNeeded();


void pauseWiFiRunBluetooth();
void doBluetooth();
void pauseBluetoothDoWiFi();
void unlockit();
void lockit();


void setup() {
  Serial.begin(115200);
  delay(1000);
    // Reset Wi-Fi
    BLEDevice::init("");
  pBLEScan = BLEDevice::getScan(); // create new scan
  //pBLEScan->setActiveScan(true);   // active scan = request more data
  //pBLEScan->setActiveScan(false);
  pinMode(2,OUTPUT);
  digitalWrite(2,HIGH);

  pinMode(LOCK, OUTPUT); 
  digitalWrite(LOCK, LOW);
  pinMode(UNLOCK, OUTPUT);
  digitalWrite(UNLOCK, LOW);
  Serial2.begin(9600, SERIAL_8N1, 16, 17); // RX=16, TX=17
}

void loop() {


if(!clientConnected){
  //Serial.println("Scan done!\n");
   unsigned long currentMillis = millis();

  // Check if 30 seconds have passed
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;  // reset the timer

    if (timerA) {
      Serial.println("⏱ Timer A finished - doing A task");
      timer1 = true;
      // --- do whatever you need here for Timer A ---
      // e.g. turn Wi-Fi ON
    } else {
      Serial.println("⏱ Timer B finished - doing B task");
      timer2 = true;
      // --- do whatever you need here for Timer B ---
      // e.g. turn Wi-Fi OFF
    }

    timerA = !timerA; // switch between A and B
  }
}
  // You can still do other stuff here while waiting...

    // 
static String inputString = "";

while (Serial2.available()) {
  char c = Serial2.read();
  if (c == '\n') {
    //Serial.println("Raw data: " + inputString);

    int firstComma = inputString.indexOf(',');
    int secondComma = inputString.indexOf(',', firstComma + 1);

    if (firstComma > 0 && secondComma > 0) {
      wifiPowerInput = inputString.substring(0, firstComma).toInt();
      wifiLock = inputString.substring(firstComma + 1, secondComma).toInt();
      wifiUnlock = inputString.substring(secondComma + 1).toInt();
      //TODO need to only run this when voalue changes
if (wifiUnlock == 1 && state == true) {
  Serial.println("Unlock signal detected");
  unlockit();
  state = false;
  wifiUnlock = 0;  // reset
}

if (wifiLock == 1 && state == false) {
  Serial.println("Lock signal detected");
  lockit();
  state = true;
  wifiLock = 0;  // reset
}
      //Serial.print("Power: ");
      //Serial.print(wifiPowerInput);
      //Serial.print(" °C, Lock: ");
      //Serial.print(wifiLock);
      //Serial.print(" %, Unlock: ");
      //Serial.println(wifiUnlock);*/
    }

    inputString = ""; // clear for next packet
  } else {
    inputString += c;
  }
}
  
  delay(20);
  doBluetooth();

}

void isActionNeeded(){
  //was door open
  //if door was not open do nothing
  //if door was open run proximaty
     //only do when key is far away
         //lock once
         //toot horn
    //if see key again 
        //unlock oncc
    //wait for door to open and close
      //now look for door open
    //Or another solution

    //loop until see key
    //if see key unlock once
  
    Serial.print("Power is ");
    //Serial.println(power);
    //wifiPowerInput = power;
    //int inValue = wifiPowerInput.toInt();
    Serial.print("Input value and threshold is ");
    //Serial.println(inValue);
    if(power > wifiPowerInput && state == true){
      unlockit();
        state = false;
        Serial.println("state is false");
    }
    if(power <= -80 && state == false){
      lockit();
      state = true;
      Serial.println("state is true");
    }
    Serial.println(state);
    //loop until don't see key
    //if don't see key lock once
}





/*void pauseWiFiRunBluetooth(){
  //Serial.println("Bluetooth Running");
        // Turn Wi-Fi OFF
      WiFi.softAPdisconnect(true);
      WiFi.disconnect(true, true);
      WiFi.mode(WIFI_OFF);
      Serial.println("Wi-Fi turned OFF");

      pBLEScan->setActiveScan(true);
      doBluetooth();

}*/

void doBluetooth(){
  BLEScanResults foundDevices = pBLEScan->start(scanTime, false);
  Serial.print("Devices found: ");
  Serial.println(foundDevices.getCount());

  for (int i = 0; i < foundDevices.getCount(); i++) {
    BLEAdvertisedDevice device = foundDevices.getDevice(i);

    // Blue Charm iBeacons use manufacturer data in Apple's format
    std::string md = device.getManufacturerData();

    if (md.length() >= 25) { // iBeacon format is 25 bytes after Apple header
      const uint8_t* data = (const uint8_t*)md.data();

      // Check Apple company ID (0x004C)
      if (data[0] == 0x4C && data[1] == 0x00 && data[2] == 0x02 && data[3] == 0x15) {
        Serial.println("----- iBeacon Found -----");

        char uuid[37];
        sprintf(uuid,
          "%02X%02X%02X%02X-%02X%02X-%02X%02X-%02X%02X-%02X%02X%02X%02X%02X%02X",
          data[4], data[5], data[6], data[7],
          data[8], data[9],
          data[10], data[11],
          data[12], data[13],
          data[14], data[15], data[16], data[17], data[18], data[19]);

        uint16_t major = (data[20] << 8) | data[21];
        uint16_t minor = (data[22] << 8) | data[23];
        int8_t txPower = (int8_t)data[24];

        Serial.print("UUID: "); Serial.println(uuid);
        Serial.print("Major: "); Serial.println(major);
        Serial.print("Minor: "); Serial.println(minor);
        Serial.print("TX Power: "); Serial.println(txPower);
        Serial.print("RSSI: "); Serial.println(device.getRSSI());
        String newUuid = String(uuid);
        //String strPower = String(txPower);
        power = device.getRSSI();
        if(newUuid.equals("426C7565-4368-6172-6D42-6561636F6E73")){
          Serial.println("Bingo");   
          isActionNeeded();
        }

      }
    }
  }
}

void unlockit(){
          digitalWrite(UNLOCK,HIGH);
        digitalWrite(LOCK, LOW);
        delay(500);
        digitalWrite(UNLOCK,LOW);
        digitalWrite(LOCK, LOW);
        Serial.println("UNLOCKING");
}

void lockit(){
      digitalWrite(UNLOCK, LOW);
      digitalWrite(LOCK, HIGH);
      delay(500);
      digitalWrite(UNLOCK, LOW);
      digitalWrite(LOCK, LOW);
      Serial.println("LOCKING");
}