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

//***** SETTINGS *****//
double settingRiderWeight = 78.00;

//***** CALULCATION *****//
int powerMvgAvgLen = 6; // orig = 8;
// 8 = 2 second power average at 4 samples per sec
// 6 = 2 second power average at 3 samples per sec
// 4 = 2 second power average at 2 samples per sec
// 2 = 2 second power average at 1 samples per sec
MovingAverageFilter mafPower(powerMvgAvgLen);
int speedMvgAvgLen = 2; // orig = 2
MovingAverageFilter mafSpeed(speedMvgAvgLen);

// For power and speed declare some variables and set some default values
long calcPrevWheelRevs;                  // For speed data set 1
long calcPrevWheelEvntTime;                      // For speed data set 1
long calcCurrWheelRevs;                  // For speed data set 2
long calcCurrWheelEvntTime;                      // For speed data set 2

int calcSpeedKmh;                     // Calculated speed in KM per Hr
int calcPrevSpeedKmh;

//***** BLE *****//
bool bleScanEnabled = true;
int bleScanTime = 5; //In seconds
BLEScan *bleScan;

BLEUUID bleServiceHeartRate = BLEUUID((uint16_t) 0x180D);
BLEUUID bleCharHeartRate = BLEUUID((uint16_t) 0x2A37);
BLEAdvertisedDevice *bleAdvDevHeartRate = nullptr;
BLEClient *bleClntHeartRate = nullptr;
BLERemoteCharacteristic *bleRemoteCharHeartRate = nullptr;
int bleHeartRateBpm = 0;

BLEUUID bleServiceFitnessMachine = BLEUUID((uint16_t) 0x1826);
BLEUUID bleCharIndoorBikeData = BLEUUID((uint16_t) 0x2AD2);
BLEAdvertisedDevice *bleAdvDevFitnessMachine = nullptr;
BLEClient *bleClntFitnessMachine = nullptr;
BLERemoteCharacteristic *bleRemoteIndoorBikeData = nullptr;
float bleSpeedKmh = 0;              // variable for the speed (kmh) read from bluetooth
int blePowerWatt = 0;               // variable for the power (W) read from bluetooth

//***** LIFT *****//
float liftSpeedMpersec = 0;           // for calculation
float liftResistanceWatts = 0;        // for calculation
float liftPowerMinusResistance = 0;   // for calculation
double liftTargetGrade = 0;

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

    liftLoop();

    if (!bleIsAvailable) {
        oledStartDisplay();
    }
    else {
        liftCalcTargetGrade();
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
        // heart rate monitor found
        if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(bleServiceHeartRate) && bleAdvDevHeartRate == nullptr) {
            bleAdvDevHeartRate = new BLEAdvertisedDevice(advertisedDevice);
            debugPrintln(String(F("Advertised device for heart rate monitor: ")) + bleAdvDevHeartRate->toString().c_str());
        }

        // fitness machine found
        if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(bleServiceFitnessMachine) && bleAdvDevFitnessMachine == nullptr) {
            bleAdvDevFitnessMachine = new BLEAdvertisedDevice(advertisedDevice);
            debugPrintln(String(F("Advertised device for fitness machine: ")) + bleAdvDevFitnessMachine->toString().c_str());
        }

        bleScanEnabled = ((bleAdvDevHeartRate == nullptr) || (bleAdvDevFitnessMachine == nullptr));
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

    if (!isConnected && bleScanEnabled) {
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
                return false;
            }
            debugPrintln(" * Found our service...");
            debugPrintln(String(F("   ** ")) + pRemoteService->toString().c_str());

            // Obtain a reference to the heart rate characteristic in the service of the remote BLE server.
            bleRemoteCharHeartRate = pRemoteService->getCharacteristic(bleCharHeartRate);
            if (bleRemoteCharHeartRate == nullptr) {
                debugPrintln(String(F(" * Failed to find our characteristic UUID: ")) + bleCharHeartRate.toString().c_str());
                bleClntHeartRate->disconnect();
                return false;
            }
            debugPrintln(" * Found our characteristic");

            // Register the notification callback
            if (bleRemoteCharHeartRate->canNotify()) {
                bleRemoteCharHeartRate->registerForNotify(bleHeartRateNotifyCallback);
            }
            else {
                debugPrintln(String(F(" * The heart rate characteristic cannot notify.")));
                bleClntHeartRate->disconnect();
                return false;
            }
        }
        else {
            debugPrintln(String(F(" * Connection to BLE device (server) failed")));
        }
    }

    return (bleClntHeartRate != nullptr && bleClntHeartRate->isConnected());
}

static void bleIndoorBikeDataNotifyCallback(BLERemoteCharacteristic *bleRemoteCharacteristic, uint8_t *data, size_t length, bool isNotify) {
    //https://developer.huawei.com/consumer/en/doc/development/HMSCore-Guides/ibd-0000001051005923

    debugPrintln(String(F("Indoor Bike Data received [")) + String(length) + String(F("]...")));
    int index = 0;

    // flags of indoor bike data
    uint16_t flags = (uint16_t) ((data[index + 1] << 8) | data[index]);
    String flagString = " * Flags =";
    for (int i = 0; i < 16; i++) {
        flagString += String(F(" [")) + String(i) + String(F("] ")) + String(bitRead(flags, i));
    }
    debugPrintln(flagString);

    index += 2;
    if ((flags & 0) == 0) {         // instantaneous speed
        uint16_t value = ((uint16_t) ((data[index + 1] << 8) | data[index]));
        bleSpeedKmh = mafSpeed.process((value * 1.0f) / 100.0f);
        debugPrintln(String(F(" * Instantaneous Speed (kmh/avgfilter) = ")) + String(bleSpeedKmh));
        index += 2;
    }
    if ((flags & 2) > 0) {          //average speed
//        uint16_t value = ((uint16_t) ((data[index + 1] << 8) | data[index]));
//        float averageSpeedKmh = (value * 1.0f) / 100.0f;
//        debugPrintln(String(F(" * Average Speed (kmh) = ")) + String(averageSpeedKmh));
        index += 2;
    }
    if ((flags & 4) > 0) {          //instantaneous cadence
        uint16_t value = ((uint16_t) ((data[index + 1] << 8) | data[index]));
        float instantaneousCadenceRevMin = (value * 1.0f) / 2.0f;
        debugPrintln(String(F(" * Instantaneous Cadence (rev/min) = ")) + String(instantaneousCadenceRevMin));
        index += 2;
    }
    if ((flags & 8) > 0) {          //average cadence
//        uint16_t value = ((uint16_t) ((data[index + 1] << 8) | data[index]));
//        float averageCadenceRevMin = (value * 1.0f) / 2.0f;
//        debugPrintln(String(F(" * Average Cadence (rev/min) = ")) + String(averageCadenceRevMin));
        index += 2;
    }
    if ((flags & 16) > 0) {          //total distance
//        uint32_t distance = ((uint32_t) (data[index + 2] << 16) | (uint32_t) (data[index + 1] << 8) | data[index]);
//        debugPrintln(String(F(" * Total distance (m) = ")) + String(distance));
        index += 3;
    }
    if ((flags & 32) > 0) {         //resistance level
//        int16_t resistance = ((int16_t) ((data[index + 1] << 8) | data[index]));
//        debugPrintln(String(F(" * Resistance Level = ")) + String(resistance));
        index += 2;
    }
    if ((flags & 64) > 0) {          //instantaneous power
        int16_t power = ((int16_t) ((data[index + 1] << 8) | data[index]));
        blePowerWatt = mafPower.process(power);
        debugPrintln(String(F(" * Instantaneous Power (watt/avgfilter) = ")) + String(blePowerWatt));
        index += 2;
    }
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
                return false;
            }
            debugPrintln(" * Found our service...");
            debugPrintln(String(F("   ** ")) + pRemoteService->toString().c_str());

            for (auto &myPair : *pRemoteService->getCharacteristics()) {
                debugPrintln(String(F("     ** ")) + myPair.second->toString().c_str());
            }

            // Obtain a reference to the power characteristic in the service of the remote BLE server.
            bleRemoteIndoorBikeData = pRemoteService->getCharacteristic(bleCharIndoorBikeData);
            if (bleRemoteIndoorBikeData == nullptr) {
                debugPrintln(String(F(" * Failed to find our power characteristic UUID: ")) + bleCharIndoorBikeData.toString().c_str());
                bleClntFitnessMachine->disconnect();
                return false;
            }
            debugPrintln(" * Found our characteristic for power");

            // Register the notification callback for power
            if (bleRemoteIndoorBikeData->canNotify()) {
                bleRemoteIndoorBikeData->registerForNotify(bleIndoorBikeDataNotifyCallback);
            }
            else {
                debugPrintln(String(F(" * The power characteristic cannot notify.")));
                bleClntFitnessMachine->disconnect();
                return false;
            }
        }
        else {
            debugPrintln(String(F(" * Connection to BLE device (server) failed")));
        }
    }

    return (bleClntFitnessMachine != nullptr && bleClntFitnessMachine->isConnected());
}
//***** LIFT Functions *****//
void liftLoop() {
    //liftCalcTargetGrade();
}

void liftCalcTargetGrade(void) {
    debugPrintln(String(F("Speed as base kmh = ")) + String(bleSpeedKmh));
    float speed28 = pow(bleSpeedKmh, 2.8);                                              // pow() needed to raise y^x where x is decimal
    debugPrintln(String(F("Speed pow 28 = ")) + String(speed28));
    liftResistanceWatts = (0.0102 * speed28) + 9.428;                                   // calculate power from rolling / wind resistance
    debugPrintln(String(F("Power from rolling / wind resistance = ")) + String(liftResistanceWatts));
    liftPowerMinusResistance = blePowerWatt - liftResistanceWatts;                      // find power from climbing
    debugPrintln(String(F("Power from climbing = ")) + String(liftPowerMinusResistance));

    liftSpeedMpersec = bleSpeedKmh / 3.6;                                                  // find speed in SI units. 1 meter / second (m/s) is equal 3.6 kilometers / hour (km/h)
    if (liftSpeedMpersec == 0) {
        liftTargetGrade = 0;
    }
    else {
        liftTargetGrade = ((liftPowerMinusResistance / (settingRiderWeight * 9.8)) / liftSpeedMpersec) * 100; // calculate grade of climb in %
    }

    //  // Limit upper and lower grades
    //  if (inputGrade < -10) {
    //    inputGrade = -10;
    //  }
    //  if (inputGrade > 20) {
    //    inputGrade = 20;
    //  }

    debugPrintln(String(F("Calculated target grade = ")) + String(liftTargetGrade));
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

String oledString03d(int value, String unit) {
    if (bleHeartRateBpm == 0) {
        return String(F("--- "));
    }

    if (bleHeartRateBpm >= 0 && bleHeartRateBpm < 10) {
        return String(F("00")) + String(bleHeartRateBpm) + String(F(" ")) + unit;
    }

    if (bleHeartRateBpm >= 10 && bleHeartRateBpm < 99) {
        return String(F("0")) + String(bleHeartRateBpm) + String(F(" ")) + unit;
    }

    return String(bleHeartRateBpm) + String(F(" ")) + unit;
}

void oledValuesDisplay() {
    oledDisplay.clearDisplay();
    oledDisplay.setCursor(0, 0);
    oledDisplay.setTextSize(2);
    oledDisplay.println(String(F("Heart")) + oledString03d(bleHeartRateBpm, "bpm"));
    oledDisplay.println(String(F("Power")) + oledString03d(blePowerWatt, "watt"));
    oledDisplay.println(String(F("Grade")) + String(liftTargetGrade) + String(F(" %")));
    oledDisplay.display();
}
