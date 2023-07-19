#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <elapsedMillis.h>

#include "can.h"

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

elapsedMillis step;
elapsedMillis temp;

#define SHUTDOWN_PIN 28
#define CH_SAFETY_PIN 27
#define LATCHING_ERROR_PIN 38

#define MAX_VOLTAGE 456000
#define MAX_CURRENT 180000

PARAMETERS param;

bool request = 0;         // BMS request
bool charge = 0;          // Charger status
bool shutdownStatus = 0;  // Charging shutdown status
bool latchingError = 0;   // Latching error status

enum status {  // state machine status
    idle,
    charging,
    shutdown
};

status CH_Status;     // current state machine status
status NX_CH_Status;  // next state machine status

String CH_Status_Strings[] = {"idle", "charging", "shutdown"};  // print array

void setup() {
    // put your setup code here, to run once:
    Serial.begin(115200);
    Serial.println("startup");

    pinMode(CH_SAFETY_PIN, INPUT);
    pinMode(SHUTDOWN_PIN, INPUT);
    pinMode(LATCHING_ERROR_PIN, INPUT);

    can1.begin();
    can1.setBaudRate(125000);
    can1.enableFIFO();
    can1.enableFIFOInterrupt();
    can1.setFIFOFilter(REJECT_ALL);
    can1.setFIFOFilter(0, CH_ID, STD);
    can1.setFIFOFilter(1, BMS_ID_CCL, STD);
    can1.setFIFOFilter(2, BMS_ID_ERR, STD);
    can1.setFIFOFilter(3, TA_ID, STD);
    can1.onReceive(canint);

    param.setVoltage = MAX_VOLTAGE;
    charge = 0;
}

void canint(const CAN_message_t& message) {
    parseMessage(message);
}

void parseChargerMessage(uint8_t data[]) {
    if (data[0] == 0x01)
        switch (data[1]) {
            case 0x02:  // set voltage response
            {
                param.setVoltage = 0;
                param.setVoltage |= data[4] << 24;
                param.setVoltage |= data[5] << 16;
                param.setVoltage |= data[6] << 8;
                param.setVoltage |= data[7];

                Serial.print("Voltage Set= ");
                Serial.print(param.setVoltage);

                break;
            }

            case 0x03:  // set current response
            {
                param.setCurrent = 0;
                param.setCurrent |= data[4] << 24;
                param.setCurrent |= data[5] << 16;
                param.setCurrent |= data[6] << 8;
                param.setCurrent |= data[7];

                Serial.print("Current Set= ");
                Serial.print(param.setCurrent);
                break;
            }

            default:
                break;
        }

    if (data[0] == 0x03)
        switch (data[1]) {
            case 0x00:  // current voltage response
            {
                param.currVoltage = 0;
                param.currVoltage |= data[4] << 24;
                param.currVoltage |= data[5] << 16;
                param.currVoltage |= data[6] << 8;
                param.currVoltage |= data[7];

                Serial.print("Current voltage= ");
                Serial.print(param.currVoltage);

                break;
            }

            case 0x2f:  // current current response
            {
                param.currCurrent = 0;
                param.currCurrent |= data[4] << 24;
                param.currCurrent |= data[5] << 16;
                param.currCurrent |= data[6] << 8;
                param.currCurrent |= data[7];

                Serial.print("Current Current= ");
                Serial.print(param.currCurrent);
                break;
            }

            default:
                break;
        }
}

void parseTAMessage(uint8_t data[]) {
    switch (data[0]) {
        case 0x00:
            for (int i = 0; i < 7; i++)  // temp 0 - 6
            {
                param.temp[i + 0] = data[i + 1];
            }

            break;

        case 0x1: {
            for (int i = 0; i < 7; i++)  // temp 7 - 13
            {
                param.temp[i + 7] = data[i + 1];
            }

            break;
        }

        case 0x2: {
            for (int i = 0; i < 7; i++)  // temp 14 - 20
            {
                param.temp[i + 14] = data[i + 1];
            }

            break;
        }

        case 0x3: {
            for (int i = 0; i < 7; i++)  // temp 21 - 27
            {
                param.temp[i + 21] = data[i + 1];
            }

            break;
        }

        case 0x4: {
            for (int i = 0; i < 7; i++)  // temp 28 - 34
            {
                param.temp[i + 28] = data[i + 1];
            }

            break;
        }

        case 0x5: {
            for (int i = 0; i < 7; i++)  // temp 35 - 41
            {
                param.temp[i + 35] = data[i + 1];
            }

            break;
        }

        case 0x6: {
            for (int i = 0; i < 7; i++)  // temp 42 - 48
            {
                param.temp[i + 42] = data[i + 1];
            }

            break;
        }

        case 0x7: {
            for (int i = 0; i < 7; i++)  // temp 49 - 55
            {
                param.temp[i + 49] = data[i + 1];
            }

            break;
        }

        case 0x8: {
            for (int i = 0; i < 4; i++)  // temp 56 - 59
            {
                param.temp[i + 56] = data[i + 1];
            }

            break;
        }
    }
}

void parseMessage(CAN_message_t message) {
    switch (message.id) {
        case CH_ID:
            parseChargerMessage(message.buf);
            break;

        case BMS_ID_CCL: {
            param.ccl = message.buf[0] * 1000;
            Serial.print("ccl = ");
            Serial.println(param.ccl);
            break;
        }

        case TA_ID: {
            parseTAMessage(message.buf);
            break;
        }
    }
}
void chargerMachine() {
    switch (CH_Status) {
        case idle: {
            if (shutdownStatus) {
                NX_CH_Status = shutdown;
            } else if (request == 1) {
                NX_CH_Status = charging;
            }
            break;
        }
        case charging: {
            if (shutdownStatus) {
                NX_CH_Status = shutdown;
            } else if (request == 0 or latchingError) {
                NX_CH_Status = idle;
            }
            break;
        }
        case shutdown:
            break;

        default: {
            Serial.println("invalid charger state");
            break;
        }
    }
}
void printStates() {
    if (CH_Status != NX_CH_Status) {
        Serial.println(CH_Status_Strings[NX_CH_Status]);
    }
}

void readInputs() {
    request = digitalRead(CH_SAFETY_PIN);
    shutdownStatus = digitalRead(SHUTDOWN_PIN);
    latchingError = digitalRead(LATCHING_ERROR_PIN);
}

void powerOnModule(bool OnOff) {
    CAN_message_t powerMsg;

    powerMsg.id = CH_ID;
    powerMsg.flags.extended = 1;
    powerMsg.len = 8;
    powerMsg.buf[0] = 0x10;
    powerMsg.buf[1] = 0x04;
    powerMsg.buf[2] = 0x00;
    powerMsg.buf[3] = 0x00;
    powerMsg.buf[4] = 0x00;
    powerMsg.buf[5] = 0x00;
    powerMsg.buf[6] = 0x00;
    powerMsg.buf[7] = 0x00;

    if (!OnOff)
        powerMsg.buf[7] = 0x01;  // turn off command

    can1.write(powerMsg);  // send message
}

void setVoltage(uint32_t voltage) {
    CAN_message_t voltageMsg;

    voltageMsg.id = CH_ID;
    voltageMsg.flags.extended = 1;
    voltageMsg.len = 8;
    voltageMsg.buf[0] = 0x10;
    voltageMsg.buf[1] = 0x02;
    voltageMsg.buf[2] = 0x00;
    voltageMsg.buf[3] = 0x00;
    voltageMsg.buf[4] = voltage >> 24 & 0xff;
    voltageMsg.buf[5] = voltage >> 16 & 0xff;
    voltageMsg.buf[6] = voltage >> 8 & 0xff;
    voltageMsg.buf[7] = voltage & 0xff;

    can1.write(voltageMsg);  // send message
}

void setCurrent(uint32_t current) {
    CAN_message_t currentMsg;

    currentMsg.id = CH_ID;
    currentMsg.flags.extended = 1;
    currentMsg.len = 8;
    currentMsg.buf[0] = 0x10;
    currentMsg.buf[1] = 0x03;
    currentMsg.buf[2] = 0x00;
    currentMsg.buf[3] = 0x00;
    currentMsg.buf[4] = current >> 24 & 0xff;
    currentMsg.buf[5] = current >> 16 & 0xff;
    currentMsg.buf[6] = current >> 8 & 0xff;
    currentMsg.buf[7] = current & 0xff;

    can1.write(currentMsg);  // send message
}

void setLOW() {
    CAN_message_t LowMsg;

    LowMsg.id = CH_ID;
    LowMsg.flags.extended = 1;
    LowMsg.len = 8;
    LowMsg.buf[0] = 0x10;
    LowMsg.buf[1] = 0x5f;
    LowMsg.buf[2] = 0x00;
    LowMsg.buf[3] = 0x00;
    LowMsg.buf[4] = 0x00;
    LowMsg.buf[5] = 0x00;
    LowMsg.buf[6] = 0x00;
    LowMsg.buf[7] = 0x00;

    can1.write(LowMsg);  // send message
}

void updateCharger(status CH_Status) {
    setCurrent(param.allowedCurrent);
    setVoltage(MAX_VOLTAGE);
    powerOnModule(CH_Status == charging);
}

void loop() {
    // Serial.println("aa");

    if (step < 1000)
        return;

    step = 0;

    readInputs();
    chargerMachine();
    printStates();
    CH_Status = NX_CH_Status;

    if (param.ccl < MAX_CURRENT) {
        param.allowedCurrent = param.ccl;
    } else {
        param.allowedCurrent = MAX_CURRENT;
    }

    for (int i = 0; i < 60; i++) {
        Serial.print("temp");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(param.temp[i]);

        param.temp[i] = 0;
    }

    updateCharger(CH_Status);
}