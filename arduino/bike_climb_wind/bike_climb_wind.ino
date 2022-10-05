#include "Arduino.h"

#include "MovingAverageFilter.h"

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//***** DEBUG *****//
bool debugSerialEnabled = true;                 // Enable serial debug - default value, may be overridden

//***** BLE *****//
int powerMvgAvgLen = 6; // orig = 8;
// 8 = 2 second power average at 4 samples per sec
// 6 = 2 second power average at 3 samples per sec
// 4 = 2 second power average at 2 samples per sec
// 2 = 2 second power average at 1 samples per sec
MovingAverageFilter mafPower(powerMvgAvgLen);
int speedMvgAvgLen = 2; // orig = 2
MovingAverageFilter mafSpeed(speedMvgAvgLen);

int bleScanTime = 5; //In seconds
BLEScan *bleScan;

BLEUUID bleServiceHeartRate = BLEUUID((uint16_t) 0x180D);
BLEUUID bleCharHeartRate = BLEUUID((uint16_t) 0x2A37);
BLEAdvertisedDevice *bleAdvDevHeartRate = nullptr;
BLEClient *bleClntHeartRate = nullptr;
BLERemoteCharacteristic *bleRemoteCharHeartRate = nullptr;
int bleHeartRateBpm = 0;

BLEUUID bleServiceFitnessMachine = BLEUUID((uint16_t) 0x1826);
BLEUUID bleCharPower = BLEUUID((uint16_t) 0x2A63);
BLEUUID bleCharSpeed = BLEUUID((uint16_t) 0x2A5B);
BLEAdvertisedDevice *bleAdvDevFitnessMachine = nullptr;
BLEClient *bleClntFitnessMachine = nullptr;
BLERemoteCharacteristic *bleRemoteCharPower = nullptr;
BLERemoteCharacteristic *bleRemoteCharSpeed = nullptr;
int blePowerWatt = 0;             // variable for the power (W) read from bluetooth

//***** OLED *****//
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 oledDisplay(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//*****************************************************************************************************************//void setup() {
void setup() {
    debugSetup();

    oledSetup();

    bleSetup();
}

//*****************************************************************************************************************//
void loop() {
    bool bleIsAvailable = bleLoop();

    if (!bleIsAvailable) {
        oledStartDisplay();
    }
    else {
        oledValuesDisplay();
    }

    delayMicroseconds(100);
}

//*****************************************************************************************************************//

//***** DEBUG Functions ****//

void debugSetup() {
    // Setup serial for debug output
    Serial.begin(115200);
    Serial.println();
}

void debugPrintln(String debugText) {
    if (!debugSerialEnabled) {
        // No debug enabled so do nothing
        return;
    }

    if (debugSerialEnabled) {
        String debugTimeText = "[+" + String(float(millis()) / 1000, 3) + "s] " + debugText;

        if (debugSerialEnabled) {
            Serial.println(debugTimeText);
            Serial.flush();
        }
    }
}

//***** BLE Functions *****//
class BleAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
        //debugPrintln(String(F("Advertised Device: ")) + advertisedDevice.toString().c_str());

        // heart rate monitor found
        if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(bleServiceHeartRate)) {
            debugPrintln(String(F("Advertised device for heart rate monitor: ")) + advertisedDevice.toString().c_str());
            bleAdvDevHeartRate = new BLEAdvertisedDevice(advertisedDevice);
        }

        // fitness machine found
        if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(bleServiceFitnessMachine)) {
            debugPrintln(String(F("Advertised device for fitness machine: ")) + advertisedDevice.toString().c_str());
            bleAdvDevFitnessMachine = new BLEAdvertisedDevice(advertisedDevice);
        }
    }
};

class BleClientCallbacks: public BLEClientCallbacks {
    void onConnect(BLEClient *bleClient) {

    }

    void onDisconnect(BLEClient *bleClient) {
        if (bleClient == bleClntHeartRate) {
            bleHeartRateBpm = 0;
        }

        if (bleClient == bleClntFitnessMachine) {
            blePowerWatt = 0;
        }

        // delete pointer in case of disconnection
        bleClient = nullptr;
    }
};

void bleSetup() {
    BLEDevice::init("");
    bleScan = BLEDevice::getScan(); //create new scan
    bleScan->setAdvertisedDeviceCallbacks(new BleAdvertisedDeviceCallbacks());
    bleScan->setActiveScan(true); //active scan uses more power, but get results faster
    bleScan->setInterval(100);
    bleScan->setWindow(99); // less or equal setInterval value
}

bool bleLoop() {
    bool isConnected = (bleConnectHeartRate() && bleConnectFitnessMachine());

    if (!isConnected) {
        // scan for devices
        debugPrintln(String(F("Starting BLE Scan for ")) + String(bleScanTime) + String(F("seconds ...")));
        BLEScanResults foundDevices = bleScan->start(bleScanTime, true);
        bleScan->clearResults(); // delete results fromBLEScan buffer to release memory
    }

    return isConnected;
}

static void bleHeartRateNotifyCallback(BLERemoteCharacteristic *bleRemoteCharacteristic, uint8_t *data, size_t length, bool isNotify) {
    bleHeartRateBpm = data[1];
    debugPrintln(String(F("Notification of Heart Rate: ")) + bleHeartRateBpm + String(F("bpm")));
}

bool bleConnectHeartRate() {
    if (bleAdvDevHeartRate != nullptr && (bleClntHeartRate == nullptr || !bleClntHeartRate->isConnected())) {
        // connect heart rate monitor if address is set
        debugPrintln(String(F("Forming a connection to heart rate monitor... ")) + bleAdvDevHeartRate->toString().c_str());

        bleClntHeartRate = BLEDevice::createClient();
        bleClntHeartRate->setClientCallbacks(new BleClientCallbacks());
        debugPrintln(String(F(" * Created client for connection")));

        if (bleClntHeartRate->connect(bleAdvDevHeartRate)) {  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
            debugPrintln(String(F(" * Connected to BLE device (server)")));

            // Obtain a reference to the heart rate service we are after in the remote BLE server.
            BLERemoteService *pRemoteService = bleClntHeartRate->getService(bleServiceHeartRate);
            if (pRemoteService == nullptr) {
                debugPrintln(String(F(" * Failed to find our service UUID :  ")) + bleServiceHeartRate.toString().c_str());
                bleClntHeartRate->disconnect();
            }
            Serial.println(" * Found our service");

            // Obtain a reference to the heart rate characteristic in the service of the remote BLE server.
            bleRemoteCharHeartRate = pRemoteService->getCharacteristic(bleCharHeartRate);
            if (bleRemoteCharHeartRate == nullptr) {
                debugPrintln(String(F(" * Failed to find our characteristic UUID: ")) + bleCharHeartRate.toString().c_str());
                bleClntHeartRate->disconnect();
            }
            Serial.println(" * Found our characteristic");

            // Register the notification callback
            if (bleRemoteCharHeartRate->canNotify()) {
                bleRemoteCharHeartRate->registerForNotify(bleHeartRateNotifyCallback);
            }
            else {
                debugPrintln(String(F(" * The heart rate characteristic cannot notify.")));
                bleClntHeartRate->disconnect();
            }
        }
        else {
            debugPrintln(String(F(" * Connection to BLE device (server) failed")));
        }
    }

    return (bleClntHeartRate != nullptr && bleClntHeartRate->isConnected());
}

static void blePowerNotifyCallback(BLERemoteCharacteristic *bleRemoteCharacteristic, uint8_t *data, size_t length, bool isNotify) {
    // see specs at https://www.bluetooth.com/specifications/specs/cycling-power-service-1-1/
    // CSCMeasurement characteristic are notified of updates approximately once per second.

    // Power is returned as watts in location 2 and 3 (loc 0 and 1 is 8 bit flags)
    byte rawpowerValue2 = data[2];       // power least sig byte in HEX
    byte rawpowerValue3 = data[3];       // power most sig byte in HEX

    long rawPowerTotal = (rawpowerValue2 + (rawpowerValue3 * 256));

    // Use moving average filter to give '3s power'
    blePowerWatt = mafPower.process(rawPowerTotal);
    debugPrintln(String(F("Notification of Power: ")) + blePowerWatt + String(F("watt")));
}

static void bleSpeedNotifyCallback(BLERemoteCharacteristic *bleRemoteCharacteristic, uint8_t *data, size_t length, bool isNotify) {

}

bool bleConnectFitnessMachine() {
    if (bleAdvDevFitnessMachine != nullptr && (bleClntFitnessMachine == nullptr || !bleClntFitnessMachine->isConnected())) {
        // connect fitness machine if address is set
        debugPrintln(String(F("Forming a connection to fitness machine... ")) + bleAdvDevFitnessMachine->toString().c_str());

        bleClntFitnessMachine = BLEDevice::createClient();
        bleClntFitnessMachine->setClientCallbacks(new BleClientCallbacks());
        debugPrintln(String(F(" * Created client for connection")));

        if (bleClntFitnessMachine->connect(bleAdvDevFitnessMachine)) {  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
            debugPrintln(String(F(" * Connected to BLE device (server)")));

            // Obtain a reference to the fitness machine service we are after in the remote BLE server.
            BLERemoteService *pRemoteService = bleClntFitnessMachine->getService(bleServiceFitnessMachine);
            if (pRemoteService == nullptr) {
                debugPrintln(String(F(" * Failed to find our service UUID :  ")) + bleServiceFitnessMachine.toString().c_str());
                bleClntFitnessMachine->disconnect();
            }
            Serial.println(" * Found our service");

            // Obtain a reference to the power characteristic in the service of the remote BLE server.
            bleRemoteCharPower = pRemoteService->getCharacteristic(bleCharPower);
            if (bleRemoteCharPower == nullptr) {
                debugPrintln(String(F(" * Failed to find our power characteristic UUID: ")) + bleCharPower.toString().c_str());
                bleClntFitnessMachine->disconnect();
            }
            Serial.println(" * Found our characteristic for power");

            // Register the notification callback for power
            if (bleRemoteCharPower->canNotify()) {
                bleRemoteCharPower->registerForNotify(blePowerNotifyCallback);
            }
            else {
                debugPrintln(String(F(" * The power characteristic cannot notify.")));
                bleClntFitnessMachine->disconnect();
            }

            // Obtain a reference to the speed characteristic in the service of the remote BLE server.
            bleRemoteCharSpeed = pRemoteService->getCharacteristic(bleCharSpeed);
            if (bleRemoteCharSpeed == nullptr) {
                debugPrintln(String(F(" * Failed to find our speed characteristic UUID: ")) + bleCharSpeed.toString().c_str());
                bleClntFitnessMachine->disconnect();
            }
            Serial.println(" * Found our characteristic for speed");

            // Register the notification callback for power
            if (bleRemoteCharSpeed->canNotify()) {
                bleRemoteCharSpeed->registerForNotify(bleSpeedNotifyCallback);
            }
            else {
                debugPrintln(String(F(" * The speed characteristic cannot notify.")));
                bleClntFitnessMachine->disconnect();
            }
        }
        else {
            debugPrintln(String(F(" * Connection to BLE device (server) failed")));
        }
    }

    return (bleClntFitnessMachine != nullptr && bleClntFitnessMachine->isConnected());
}

//***** OLED Functions *****//

void oledSetup() {
    if (oledDisplay.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        oledStartDisplay();
    }
    else {
        debugPrintln(String(F("Display SSD1306 allocation failed")));
    }
}

void oledStartDisplay() {
    oledDisplay.clearDisplay();
    oledDisplay.setTextSize(2);
    oledDisplay.setTextColor(WHITE);
    oledDisplay.setCursor(0, 0);
    oledDisplay.println(String(F("  - BIKE -")));
    oledDisplay.setTextSize(1);
    oledDisplay.println(String(F("      CLIMB & BLOW    ")));
    oledDisplay.println();
    oledDisplay.println(String(F(" Waiting for devices ")));
    oledDisplay.display();
}

String oledHeartRateString() {
    if (bleHeartRateBpm == 0) {
        return String(F("--- bpm"));
    }

    if (bleHeartRateBpm >= 0 && bleHeartRateBpm < 10) {
        return String(F("00")) + String(bleHeartRateBpm) + String(F(" bpm"));
    }

    if (bleHeartRateBpm >= 10 && bleHeartRateBpm < 99) {
        return String(F("0")) + String(bleHeartRateBpm) + String(F(" bpm"));
    }

    return String(bleHeartRateBpm) + String(F(" bpm"));
}

void oledValuesDisplay() {
    oledDisplay.clearDisplay();
    oledDisplay.setCursor(0, 0);
    oledDisplay.setTextSize(2);
    oledDisplay.println(oledHeartRateString());
    oledDisplay.display();
}
