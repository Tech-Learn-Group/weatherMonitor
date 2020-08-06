/*
 Name:    Weather Monitor
 Created: April 1, 2020 
 Sensor :   DHT22 
 Author: Newton Kelvin
*/ 
 // Libraries
#include <Arduino.h>
#include <ArduinoLowPower.h> 
#include <SigFox.h>
#include <DHT.h>


// Constants
#define DHTPIN 2; // The Pin we connect to.
#define DHTTYPE DHT22; // DHT 22 (AM2302)
DHT dht(DHTPIN, DHTTYPE);; // Initialize DHT sensor for normal 16mhz arduino


// Connect pin 1 (on the left) of the sensor to +5V
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 3 (on the right) of the sensor to GROUND

#define DHTPIN        2                // What digital pin we're connected to
#define DHTTYPE       DHT22
#define DEBUG         true             // Set DEBUG to false to disable serial prints
#define SLEEPTIME     5*60*1000   // Set the delay to 5 minutes (5 min x 60 seconds x 1000 milliseconds)

#define UINT16_t_MAX  65536
#define INT16_t_MAX   UINT16_t_MAX/2

typedef struct __attribute__ ((packed)) sigfox_message {
        int16_t moduleTemperature;
        int16_t dhtTemperature;
        uint16_t dhtHumidity;
        uint8_t lastMessageStatus;
} SigfoxMessage;

// Stub for message which will be sent
SigfoxMessage msg;

//DHT dht(DHTPIN, DHTTYPE);

void setup() {
        if (DEBUG) {
                Serial.begin(9600);
                while (!Serial) {}
        }

        if (!SigFox.begin()) {
                // Something is really wrong, try rebooting
                // Reboot is useful if we are powering the board using an unreliable power source
                // (eg. solar panels or other energy harvesting methods)
                reboot();
        }

        // Send module to standby until we need to send a message
        SigFox.end();

        if (DEBUG) {
                // Enable DEBUG prints and LED indication if we are testing
                SigFox.debug();
        }

        dht.begin();
}

void loop() {
        // Reading temperature or humidity takes about 250 milliseconds!
        // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
        float h = dht.readHumidity();
        // Read temperature as Celsius (the default)
        float t = dht.readTemperature();
        // Check if any reads failed and exit early (to try again).
        if (isnan(h) || isnan(t)) {
                Serial.println("Failed to read from DHT sensor!");
                return;
        }

        msg.dhtTemperature = convertoFloatToInt16(t, 60, -60);
        msg.dhtHumidity = convertoFloatToUInt16(h, 110);

        if(DEBUG) {
                Serial.print("Humidity: ");
                Serial.print(h);
                Serial.print(" %\t");
                Serial.print("Temperature: ");
                Serial.print(t);
                Serial.print(" *C ");
        }
        // Start the module
        SigFox.begin()
        // Wait at least 30ms after first configuration (100ms before)
        delay(100)

        // We can only read the module temperature before SigFox.end()
        t = SigFox.internalTemperature()
        msg.moduleTemperature = convertoFloatToInt16(t, 60, -60)

        // Clears all pending interrupts
        SigFox.status()
        delay(1);

        SigFox.beginPacket();
        SigFox.write((uint8_t*)&msg, 12)

        msg.lastMessageStatus = SigFox.endPacket()

        SigFox.end();
        //Sleep for 5 minutes
        LowPower.sleep(SLEEPTIME)
}

void reboot() {
        NVIC_SystemReset()
        while (1) ;
}

int16_t convertoFloatToInt16(float value, long max, long min) {
        float conversionFactor = (float) (INT16_t_MAX) / (float)(max - min);
        return (int16_t)(value * conversionFactor)
}

uint16_t convertoFloatToUInt16(float value, long max) {
        float conversionFactor = (float) (UINT16_t_MAX) / (float)(max);
        return (uint16_t)(value * conversionFactor)
}
