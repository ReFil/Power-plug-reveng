#include <Arduino.h>
#include "HLW8012.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define SERVICE_UUID           "7f087ad6-ba82-4b45-ae10-99b57c0dd188" 
#define CHARACTERISTIC_UUID_RX "f2ec7a09-c4a9-4c22-a4ed-25a37ef9bbd3"
#define CHARACTERISTIC_UUID_TX "f2ec7a09-c4a9-4c22-a4ed-25a37ef9bbd4"

BLEServer* pServer = NULL;
BLECharacteristic* pRxCharacteristic = NULL;
BLECharacteristic* pTxCharacteristic = NULL;

bool deviceConnected = false;


class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

#define Serial_BAUDRATE                 115200

// GPIOs
#define RELAY_PIN                       D3
#define SEL_PIN                         D0
#define CF1_PIN                         D2
#define CF_PIN                          D1

#define LED1_PIN                        D4
#define LED2_PIN                        D5
#define SW_PIN                          D6

// Check values every 10 seconds
#define UPDATE_TIME                     5000

// Set SEL_PIN to HIGH to sample current
// This is the case for Itead's Sonoff POW, where a
// the SEL_PIN drives a transistor that pulls down
// the SEL pin in the HLW8012 when closed
#define CURRENT_MODE                    LOW

// These are the nominal values for the resistors in the circuit
#define CURRENT_RESISTOR                0.001
#define VOLTAGE_RESISTOR_UPSTREAM       ( 2 * 12000000 ) // Real: 2280k
#define VOLTAGE_RESISTOR_DOWNSTREAM     ( 1000 ) // Real 1.009k

#define SERIAL_EN

HLW8012 hlw8012;

// When using interrupts we have to call the library entry point
// whenever an interrupt is triggered
void ICACHE_RAM_ATTR hlw8012_cf1_interrupt() {
    hlw8012.cf1_interrupt();
}
void ICACHE_RAM_ATTR hlw8012_cf_interrupt() {
    hlw8012.cf_interrupt();
}

// Library expects an interrupt on both edges
void setInterrupts() {
    attachInterrupt(digitalPinToInterrupt(CF1_PIN), hlw8012_cf1_interrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(CF_PIN), hlw8012_cf_interrupt, CHANGE);
}

void calibrate() {

    // Let some time to register values
    unsigned long timeout = millis();
    while ((millis() - timeout) < 10000) {
        delay(1);
    }

    // Calibrate using a 60W bulb (pure resistive) on a 230V line
    hlw8012.expectedActivePower(60.0);
    hlw8012.expectedVoltage(230.0);
    hlw8012.expectedCurrent(60.0 / 230.0);

    // Show corrected factors
#ifdef SERIAL_EN
    Serial.print("[HLW] New current multiplier : "); Serial.println(hlw8012.getCurrentMultiplier());
    Serial.print("[HLW] New voltage multiplier : "); Serial.println(hlw8012.getVoltageMultiplier());
    Serial.print("[HLW] New power multiplier   : "); Serial.println(hlw8012.getPowerMultiplier());
#endif
}

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();

  
    }
};

void setup() {
#ifdef SERIAL_EN

    // Init Serial port and clean garbage
    Serial.begin(115200);
    Serial.println("Hello worls");
    Serial.println();
#endif

    BLEDevice::init("UoS Power");
    BLEServer *pServer = BLEDevice::createServer();

    pServer->setCallbacks(new MyServerCallbacks());

    
    BLEService *pService = pServer->createService(SERVICE_UUID);
    pRxCharacteristic = pService->createCharacteristic(
                                           CHARACTERISTIC_UUID_RX,
                                           BLECharacteristic::PROPERTY_WRITE
                                         );
    pTxCharacteristic = pService->createCharacteristic(
                                           CHARACTERISTIC_UUID_TX,
                                           BLECharacteristic::PROPERTY_READ|
                                           BLECharacteristic::PROPERTY_NOTIFY
                                         );

    pRxCharacteristic->setCallbacks(new MyCallbacks());
    pTxCharacteristic->addDescriptor(new BLE2902());
    pTxCharacteristic->setValue("Hello World says Neil");
    pService->start();

    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(false);
    pAdvertising->setMinPreferred(0x00);  // functions that help with iPhone connections issue
    BLEDevice::startAdvertising();

    #ifdef SERIAL_EN

    Serial.println("Characteristic defined! Now you can read it in your phone!");
#endif

    // Close the relay to switch on the load
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, HIGH);

    // Initialize HLW8012
    // void begin(unsigned char cf_pin, unsigned char cf1_pin, unsigned char sel_pin, unsigned char currentWhen = HIGH, bool use_interrupts = false, unsigned long pulse_timeout = PULSE_TIMEOUT);
    // * cf_pin, cf1_pin and sel_pin are GPIOs to the HLW8012 IC
    // * currentWhen is the value in sel_pin to select current sampling
    // * set use_interrupts to true to use interrupts to monitor pulse widths
    // * leave pulse_timeout to the default value, recommended when using interrupts
    hlw8012.begin(CF_PIN, CF1_PIN, SEL_PIN, CURRENT_MODE, true);

    // These values are used to calculate current, voltage and power factors as per datasheet formula
    // These are the nominal values for the Sonoff POW resistors:
    // * The CURRENT_RESISTOR is the 1milliOhm copper-manganese resistor in series with the main line
    // * The VOLTAGE_RESISTOR_UPSTREAM are the 5 470kOhm resistors in the voltage divider that feeds the V2P pin in the HLW8012
    // * The VOLTAGE_RESISTOR_DOWNSTREAM is the 1kOhm resistor in the voltage divider that feeds the V2P pin in the HLW8012
    hlw8012.setResistors(CURRENT_RESISTOR, VOLTAGE_RESISTOR_UPSTREAM, VOLTAGE_RESISTOR_DOWNSTREAM);

    // Show default (as per datasheet) multipliers
    #ifdef SERIAL_EN

    Serial.print("[HLW] Default current multiplier : "); Serial.println(hlw8012.getCurrentMultiplier());
    Serial.print("[HLW] Default voltage multiplier : "); Serial.println(hlw8012.getVoltageMultiplier());
    Serial.print("[HLW] Default power multiplier   : "); Serial.println(hlw8012.getPowerMultiplier());
    Serial.println();
    #endif


    setInterrupts();
    //calibrate();

}

char buffer[50];
unsigned int power;
uint16_t voltage;
double current;
unsigned int apppwr;
int pfactor;
unsigned long energy;



void loop() {

    static unsigned long last = millis();

    // This UPDATE_TIME should be at least twice the interrupt timeout (2 second by default)
    if ((millis() - last) > UPDATE_TIME) {

        last = millis();

        power = hlw8012.getActivePower();
        voltage = hlw8012.getVoltage();
        current = hlw8012.getCurrent();
        apppwr = hlw8012.getApparentPower();
        pfactor = (100 * hlw8012.getPowerFactor());
        energy = hlw8012.getEnergy();

        pTxCharacteristic->setValue((uint8_t*)&voltage, 2);
        
        #ifdef SERIAL_EN
        
        Serial.print("[HLW] Active Power (W)    : "); Serial.println(power);
        Serial.print("[HLW] Voltage (V)         : "); Serial.println(voltage);
        Serial.print("[HLW] Current (A)         : "); Serial.println(current);
        Serial.print("[HLW] Apparent Power (VA) : "); Serial.println(apppwr);
        Serial.print("[HLW] Power Factor (%)    : "); Serial.println(pfactor);
        Serial.print("[HLW] Agg. energy (Ws)    : "); Serial.println(energy);
        Serial.println();
        #endif

    }

}
