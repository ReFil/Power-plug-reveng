#include <Arduino.h>
#include "BL0937.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <RTClib.h>
#include "FS.h"
#include "SPIFFS.h"
#include "ESPFlash.h"

#define SERVICE_UUID           "7f087ad6-ba82-4b45-ae10-99b57c0dd188" 
#define CHARACTERISTIC_UUID_RELAY "f2ec7a09-c4a9-4c22-a4ed-25a37ef9bbd3"
#define CHARACTERISTIC_UUID_DATETIME "f2ec7a09-c4a9-4c22-a4ed-25a37ef9bbd4"
#define CHARACTERISTIC_UUID_DL "f2ec7a09-c4a9-4c22-a4ed-25a37ef9bbd5"
#define CHARACTERISTIC_UUID_LASTTIME "f2ec7a09-c4a9-4c22-a4ed-25a37ef9bbd6"
#define CHARACTERISTIC_UUID_DATA "f2ec7a09-c4a9-4c22-a4ed-25a37ef9bbd7"
#define CHARACTERISTIC_UUID_CALIB "f2ec7a09-c4a9-4c22-a4ed-25a37ef9bbd8"

#define Serial_BAUDRATE                 115200

#define FORMAT_SPIFFS_IF_FAILED true

// GPIOs
#define RELAY_PIN                       D10
#define SEL_PIN                         D0
#define CF1_PIN                         D2
#define CF_PIN                          D1

#define LED1_PIN                        D9
#define LED2_PIN                        D8
#define SW_PIN                          D7

// Set SEL_PIN to HIGH to sample current
// This is the case for Itead's Sonoff POW, where a
// the SEL_PIN drives a transistor that pulls down
// the SEL pin in the BL0937 when closed
#define CURRENT_MODE                    HIGH

// These are the nominal values for the resistors in the circuit
#define CURRENT_RESISTOR                0.001
#define VOLTAGE_RESISTOR_UPSTREAM       ( 2 * 1200000 ) // Real: 2280k
#define VOLTAGE_RESISTOR_DOWNSTREAM     ( 1000 ) // Real 1.009k

//Debugging and alt mode defines

//#define NO_RTC
#define SERIAL_EN

//define energy meter ic device
BL0937 bl0937;

//Define RTC
RTC_DS1307 rtc;

//Init bluetooth jibber jabber
BLEServer* pServer = NULL;
//Recieve characteristics for setting relay, date and time and requesting data download
BLECharacteristic* pRelayCharacteristic = NULL;
BLECharacteristic* pDateTimeCharacteristic = NULL;
BLECharacteristic* pDlCharacteristic = NULL;
BLECharacteristic* pCalibCharacteristic = NULL;
//Tx characteristics for sending last time of data download and main data download characteristic
BLECharacteristic* pLastTimeCharacteristic = NULL;
BLECharacteristic* pDataCharacteristic = NULL;

bool deviceConnected = false;
bool oldDeviceConnected = false;

struct dataPacket {
    double current;
    uint16_t yr;
    uint8_t mnth;
    uint8_t day;
    uint8_t hr;
    uint8_t min;
};

ESPFlash <dataPacket> dataStore("/dataStore");
ESPFlash <DateTime> lastTime("/lastTime");
ESPFlash <double> currentCal("/currentCal");

//Init timers for flashing
hw_timer_t *timer1 = NULL;
//Init sample timer
hw_timer_t *timer2 = NULL;


// When using interrupts we have to call the library entry point
// whenever an interrupt is triggered
void IRAM_ATTR bl0937_cf1_interrupt() {
    bl0937.cf1_interrupt();
}
void IRAM_ATTR bl0937_cf_interrupt() {
    bl0937.cf_interrupt();
}

// Library expects an interrupt on both edges
void setInterrupts() {
    attachInterrupt(digitalPinToInterrupt(CF1_PIN), bl0937_cf1_interrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(CF_PIN), bl0937_cf_interrupt, CHANGE);
}

void setLastTime() {
    String strs[5];
    strs[0] = String(lastTime.get().year());
    strs[1] = String(lastTime.get().month());
    strs[2] = String(lastTime.get().day());
    strs[3] = String(lastTime.get().hour());
    strs[4] = String(lastTime.get().minute());

    pLastTimeCharacteristic->setValue((strs[0] + ":" + strs[1] + ":" + strs[2] + ":" + strs[3] + ":" + strs[4] + ":" + strs[5]).c_str());
}


//init bluetooth callbacks
class RelayCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        std::string rxValue = pCharacteristic->getValue();
        switch (std::stoi(rxValue))
        {
        case 1:
            digitalWrite(RELAY_PIN, HIGH);
            break;
        case 2:
            digitalWrite(RELAY_PIN, HIGH);
            break;
        
        default:
            break;
        }
    }
};

class calibCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        String rxValue = (pCharacteristic->getValue()).c_str();
        //current calib mode
        unsigned long timeout = millis();
        while ((millis() - timeout) < 10000) {
            delay(1);
        }
        bl0937.expectedCurrent(rxValue.toDouble());
        pCharacteristic->setValue(String(bl0937.getCurrentMultiplier()).c_str());
        currentCal.set(bl0937.getCurrentMultiplier());
    }
};

class TimeCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        String rxValue = (pCharacteristic->getValue()).c_str();
    

        #ifndef NO_RTC
        //Actual date/time set code
        //date/time is et in format YYYY:MM:DD:HH:MM:SS
        String strs[6];
        uint8_t StringCount = 0;
        while (rxValue.length() > 0)
        {
            int8_t index = rxValue.indexOf(':');
            if (index == -1) // No space found
            {
            strs[StringCount++] = rxValue;
            break;
            }
            else
            {
            strs[StringCount++] = rxValue.substring(0, index);
            rxValue = rxValue.substring(index+1);
            }
        }
        rtc.adjust(DateTime(strs[0].toInt(), strs[1].toInt(), strs[2].toInt(), strs[3].toInt(), strs[4].toInt(), strs[5].toInt()));

        delay(10000);
        strs[0] = String(rtc.now().year());
        strs[1] = String(rtc.now().month());
        strs[2] = String(rtc.now().day());
        strs[3] = String(rtc.now().hour());
        strs[4] = String(rtc.now().minute());
        strs[5] = String(rtc.now().second());

        pCharacteristic->setValue((strs[0] + ":" + strs[1] + ":" + strs[2] + ":" + strs[3] + ":" + strs[4] + ":" + strs[5]).c_str());

        if(pLastTimeCharacteristic->getValue() == "Set Date/Time")
            pLastTimeCharacteristic->setValue("Time Set!");
        #else
            pCharacteristic->setValue("No RTC!!!");
        #endif        
    }
    void onRead(BLECharacteristic *pCharacteristic) {
        String strs[6];
        strs[0] = String(rtc.now().year());
        strs[1] = String(rtc.now().month());
        strs[2] = String(rtc.now().day());
        strs[3] = String(rtc.now().hour());
        strs[4] = String(rtc.now().minute());
        strs[5] = String(rtc.now().second());

        pCharacteristic->setValue((strs[0] + ":" + strs[1] + ":" + strs[2] + ":" + strs[3] + ":" + strs[4] + ":" + strs[5]).c_str());
    }
};

volatile bool btdown = false;

class DlCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        std::string rxValue = pCharacteristic->getValue();
        switch (std::stoi(rxValue))
        {
        case 1:
            btdown = true;

            break;
        case 2:
            //Delete Data
            #ifdef SERIAL_EN
                Serial.println("Delete request");
            #endif
            //Clear data, set last time, start sampling
            dataStore.clear();
            lastTime.set(rtc.now());
            setLastTime();
            timerAlarmEnable(timer2);
            break;
        
        default:
            break;
        }
        
    }
};

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      timerAlarmDisable(timer1);
      digitalWrite(LED1_PIN, LOW);
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      digitalWrite(LED1_PIN, HIGH);
      BLEDevice::stopAdvertising();
    }
};

//Init misc interrupts

void IRAM_ATTR onTimer1(){
    digitalWrite(LED1_PIN, !digitalRead(LED1_PIN));
}
#ifdef NO_RTC
volatile uint16_t increment;
#endif
volatile bool sample;
//Sampling code
void IRAM_ATTR onTimer2(){
    digitalWrite(LED2_PIN, !digitalRead(LED2_PIN));
    sample = true;
 }

volatile bool print = false;

void IRAM_ATTR isr() {
    #ifdef SERIAL_EN
        Serial.println("Start Adv");
        print = true;
    #endif
    pServer->startAdvertising();
    timerAlarmEnable(timer1);
}

void setup() {

    #ifdef SERIAL_EN
    // Init Serial port and clean garbage
    Serial.begin(115200);
    Serial.println("Hello worls");
    Serial.println();
    #endif

    //configure BLE
    BLEDevice::init("UoS Power");
    BLEServer *pServer = BLEDevice::createServer();

    pServer->setCallbacks(new MyServerCallbacks());

    //Initialise all characteristics
    BLEService *pService = pServer->createService(SERVICE_UUID);
    pRelayCharacteristic = pService->createCharacteristic(
                                           CHARACTERISTIC_UUID_RELAY,
                                           BLECharacteristic::PROPERTY_WRITE |
                                           BLECharacteristic::PROPERTY_READ
                                         );
    pDateTimeCharacteristic = pService->createCharacteristic(
                                           CHARACTERISTIC_UUID_DATETIME,
                                           BLECharacteristic::PROPERTY_WRITE |
                                           BLECharacteristic::PROPERTY_READ
                                         );
    pDlCharacteristic = pService->createCharacteristic(
                                           CHARACTERISTIC_UUID_DL,
                                           BLECharacteristic::PROPERTY_WRITE
                                         );
    pLastTimeCharacteristic = pService->createCharacteristic(
                                           CHARACTERISTIC_UUID_LASTTIME,
                                           BLECharacteristic::PROPERTY_READ
                                         );
    pDataCharacteristic = pService->createCharacteristic(
                                           CHARACTERISTIC_UUID_DATA,
                                           BLECharacteristic::PROPERTY_READ|
                                           BLECharacteristic::PROPERTY_NOTIFY
                                         );    
    pCalibCharacteristic = pService->createCharacteristic(
                                           CHARACTERISTIC_UUID_CALIB,
                                           BLECharacteristic::PROPERTY_WRITE |
                                           BLECharacteristic::PROPERTY_READ
                                         );                                 

    //Set callbacks
    pRelayCharacteristic->setCallbacks(new RelayCallbacks());
    pDateTimeCharacteristic->setCallbacks(new TimeCallbacks());
    pDlCharacteristic->setCallbacks(new DlCallbacks());
    pCalibCharacteristic->setCallbacks(new calibCallbacks());
    pDataCharacteristic->addDescriptor(new BLE2902());
    
    pService->start();

    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(false);
    pAdvertising->setMinPreferred(0x00);  // functions that help with iPhone connections issue
    BLEDevice::stopAdvertising();
    

    #ifdef SERIAL_EN
    Serial.println("Characteristic defined! Now you can read it in your phone!");
    #endif

    //Init filesystem for datastorage
    if(!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)){
        #ifdef SERIAL_EN
            Serial.println("SPIFFS Mount Failed");
        #endif
      return;
    }

    #ifndef NO_RTC
    if (! rtc.begin()) {
        #ifdef SERIAL_EN
        Serial.println("Couldn't find RTC");
        #endif
        while (1);
    }
    #endif
    //If last time saved in flash
    if(lastTime.length() > 0)
        setLastTime();
    //if RTC not set
    #ifndef NO_RTC
    if(rtc.now().year() < 2010)
        pLastTimeCharacteristic->setValue("Set Date/Time");
    #else
        pLastTimeCharacteristic->setValue("NO RTC!!!");
    #endif

    // Close the relay to switch on the load
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, HIGH);

    //Configure i/o
    pinMode(LED1_PIN, OUTPUT);
    pinMode(LED2_PIN, OUTPUT);
    digitalWrite(LED1_PIN, HIGH);
    digitalWrite(LED2_PIN, HIGH);

    pinMode(SW_PIN, INPUT_PULLUP);
    
    //Configure timers for LED flashing
    timer1 = timerBegin(0, 80, true);

    timerAttachInterrupt(timer1, &onTimer1, true);

    timerAlarmWrite(timer1, 500000, true);

    //Configure sample timer
    timer2 = timerBegin(1, 4800, true);
    timerAttachInterrupt(timer2, &onTimer2, true);
    timerAlarmWrite(timer2, 200000, true);
    // Initialize BL0937
    // void begin(unsigned char cf_pin, unsigned char cf1_pin, unsigned char sel_pin, unsigned char currentWhen = HIGH, bool use_interrupts = false, unsigned long pulse_timeout = PULSE_TIMEOUT);
    // * cf_pin, cf1_pin and sel_pin are GPIOs to the BL0937 IC
    // * currentWhen is the value in sel_pin to select current sampling
    // * set use_interrupts to true to use interrupts to monitor pulse widths
    // * leave pulse_timeout to the default value, recommended when using interrupts
    bl0937.begin(CF_PIN, CF1_PIN, SEL_PIN, CURRENT_MODE, true);

    // These values are used to calculate current, voltage and power factors as per datasheet formula
    // These are the nominal values for the Sonoff POW resistors:
    // * The CURRENT_RESISTOR is the 1milliOhm copper-manganese resistor in series with the main line
    // * The VOLTAGE_RESISTOR_UPSTREAM are the 5 470kOhm resistors in the voltage divider that feeds the V2P pin in the BL0937
    // * The VOLTAGE_RESISTOR_DOWNSTREAM is the 1kOhm resistor in the voltage divider that feeds the V2P pin in the BL0937
    bl0937.setResistors(CURRENT_RESISTOR, VOLTAGE_RESISTOR_UPSTREAM, VOLTAGE_RESISTOR_DOWNSTREAM);

    // Show default (as per datasheet) multipliers
    #ifdef SERIAL_EN

    Serial.print("[HLW] Default current multiplier : "); Serial.println(bl0937.getCurrentMultiplier());
    Serial.print("[HLW] Default voltage multiplier : "); Serial.println(bl0937.getVoltageMultiplier());
    Serial.print("[HLW] Default power multiplier   : "); Serial.println(bl0937.getPowerMultiplier());
    Serial.println();
    #endif
    //Calibrate current sensor
    if(currentCal.length() > 0) {
        bl0937.setCurrentMultiplier(currentCal.get());
        pCalibCharacteristic->setValue(String(currentCal.get()).c_str());
    }
    else
        pCalibCharacteristic->setValue("Req Calib!!");

    //Connection pin interrupt
    attachInterrupt(SW_PIN, isr, FALLING);
    setInterrupts();
    timerAlarmEnable(timer2);
    #ifdef SERIAL_EN
        Serial.println("Entering prog");
    #endif
}

void loop() {
    if(sample) {
       dataPacket data;
    #ifdef SERIAL_EN
        Serial.println("Taking sammple");
    #endif
    
    
    #ifdef NO_RTC
        data.current = bl0937.getCurrent();
        data.yr = 2023;
        data.mnth = 1;
        data.day = 27;
        data.hr = (increment/60) % 60;
        data.min = increment % 60;
        increment++;
        dataStore.append(data);
    #else
    double current = bl0937.getActivePower();
    pRelayCharacteristic->setValue(String(current).c_str());
    if(true) {
        data.current = current;
        DateTime now = rtc.now();
        data.yr = now.year();
        data.mnth = now.month();
        data.day = now.day();
        data.hr = now.hour();
        data.min = now.minute();
        dataStore.append(data);
    }
    #endif
    sample = false;
    }
    if(print) {
        timerAlarmDisable(timer2);
        for(uint32_t i = 0; i < dataStore.length(); i++) {
                dataPacket data = dataStore.getElementAt(i);
                #ifdef SERIAL_EN
                    Serial.print(data.current); 
                    Serial.print(", "); 
                    Serial.print(data.yr); 
                    Serial.print(", "); 
                    Serial.print(data.mnth); 
                    Serial.print(", "); 
                    Serial.print(data.day); 
                    Serial.print(", "); 
                    Serial.print(data.hr); 
                    Serial.print(", "); 
                    Serial.print(data.min); 
                    Serial.println(" "); 

                #endif
                delay(5);
            }
        timerAlarmEnable(timer2);
        print = false;
    }
    if(btdown) {
        //Send  data
            //Stop sampling
            timerAlarmDisable(timer2);
            #ifdef SERIAL_EN
                Serial.print("Download request of data. length: ");
                Serial.println(dataStore.length());
            #endif
            digitalWrite(LED2_PIN, LOW);
            for(uint32_t i = 0; i < dataStore.length(); i++) {
                dataPacket data = dataStore.getElementAt(i);
                pDataCharacteristic->setValue((uint8_t*)&data, sizeof(data));
                pDataCharacteristic->notify();
                /*#ifdef SERIAL_EN
                    Serial.print(data.current); 
                    Serial.print(": "); 
                    Serial.print(data.yr); 
                    Serial.print(", "); 
                    Serial.print(data.mnth); 
                    Serial.print(", "); 
                    Serial.print(data.day); 
                    Serial.print(", "); 
                    Serial.print(data.hr); 
                    Serial.print(", "); 
                    Serial.print(data.min); 
                    Serial.println(" "); 

                #endif*/
                delay(10);
            }
            digitalWrite(LED2_PIN, HIGH);
            btdown = false;
    }
}
