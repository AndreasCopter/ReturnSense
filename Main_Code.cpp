/*
   ESP32 <-> INAV 8.x failsafe supervisor

   Aircraft: X4 multirotor
   FC:       INAV 8.0.x
   FC port:  UART2 configured for MSP

   RC input: CRSF / ExpressLRS
   FC link:  MSP

   IMPORTANT:
   ---------
   This software does NOT drive motors directly.
   INAV remains responsible for stabilization and motor mixing.

   DEVELOPMENT VERSION:
   - Test WITHOUT PROPELLERS first.
   - Autonomous RTH is intentionally gated.
   - Verify every channel and mode assignment on the bench.
*/

#include <Arduino.h>
#include <math.h>

// ============================================================
// USER CONFIGURATION
// ============================================================

// ---------- ESP32 UARTS ----------

// CRSF receiver -> ESP32
HardwareSerial RCSerial(1);

// ESP32 -> INAV UART2
HardwareSerial INAVSerial(2);

// Change these to your actual ESP32 GPIOs.
static const int RC_RX_PIN = 16;
static const int RC_TX_PIN = 17;

// ESP32 UART connected to INAV UART2
static const int INAV_RX_PIN = 26;
static const int INAV_TX_PIN = 27;

// CRSF baud
static const uint32_t RC_BAUD = 420000;

// MSP baud.
// Configure INAV UART2 for the same value.
static const uint32_t MSP_BAUD = 115200;


// ---------- RC watchdog ----------

static const uint32_t RC_TIMEOUT_MS = 5000;

// Require RC to remain healthy before giving control back.
static const uint32_t RC_RECOVER_MS = 3000;


// ---------- GPS requirements ----------

static const uint8_t MIN_SATS = 8;

// GPS must have been valid for this long before we trust it.
static const uint32_t GPS_VALID_TIME_MS = 3000;


// ---------- Recovery ----------

// We ask INAV to execute RTH.
// INAV's RTH altitude configuration should be set in INAV.
//
// Do NOT use a fixed 10 m motor command here.
//
// Recommended INAV configuration:
// nav_rth_alt_mode = 1
// nav_rth_altitude = 1000       // 10 m extra
//
// Verify these values in your particular INAV configuration.

static const bool ENABLE_AUTONOMOUS_RTH = false;

// Keep FALSE during initial testing.
// Change to TRUE only after bench/SITL testing.


// ---------- Drift learning ----------

// Ignore measurements while almost stationary.
static const float MIN_GROUND_SPEED_MS = 2.0f;

// Low-pass coefficient.
static const float DRIFT_ALPHA = 0.02f;


// ============================================================
// MSP COMMANDS
// ============================================================

#define MSP_RAW_GPS       106
#define MSP_COMP_GPS      107
#define MSP_ATTITUDE      108
#define MSP_ALTITUDE      109
#define MSP_SET_RAW_RC    200
#define MSP_WP            118
#define MSP_NAV_STATUS    121

// INAV mode permanent IDs
#define BOXNAVALTHOLD     3
#define BOXNAVRTH         8
#define BOXNAVPOSHOLD     9
#define BOXNAVCRUISE      44


// ============================================================
// RC CHANNEL STORAGE
// ============================================================

uint16_t rcChannels[16];

bool rcValid = false;

uint32_t lastRcPacket = 0;
uint32_t rcRecoveredSince = 0;


// ============================================================
// GPS STATE
// ============================================================

struct GPSData
{
    bool valid = false;

    uint8_t fix = 0;
    uint8_t sats = 0;

    int32_t latitude = 0;
    int32_t longitude = 0;

    int32_t altitudeCm = 0;

    uint16_t speedCms = 0;
    uint16_t courseDeg = 0;
};

GPSData gps;

uint32_t gpsValidSince = 0;


// ============================================================
// HOME STATE
// ============================================================

struct HomeData
{
    bool valid = false;

    uint16_t distanceMeters = 0;
    int16_t directionDeg = 0;
};

HomeData home;


// ============================================================
// ATTITUDE
// ============================================================

struct AttitudeData
{
    int16_t roll = 0;
    int16_t pitch = 0;
    int16_t heading = 0;
};

AttitudeData attitude;


// ============================================================
// DRIFT ESTIMATION
// ============================================================

float driftDeg = 0.0f;

bool driftInitialized = false;


// ============================================================
// FAILSAFE STATE
// ============================================================

enum RecoveryState
{
    STATE_NORMAL,
    STATE_RC_LOST,
    STATE_VALIDATE,
    STATE_RTH,
    STATE_FALLBACK
};

RecoveryState recoveryState = STATE_NORMAL;


// ============================================================
// MSP UTILITIES
// ============================================================

uint8_t checksum8(const uint8_t *data, uint8_t len)
{
    uint8_t c = 0;

    for (uint8_t i = 0; i < len; i++)
        c ^= data[i];

    return c;
}


// ------------------------------------------------------------
// Send legacy MSP frame
// ------------------------------------------------------------

void sendMSP(uint8_t command, const uint8_t *payload, uint8_t length)
{
    uint8_t checksum = length ^ command;

    for (uint8_t i = 0; i < length; i++)
        checksum ^= payload[i];

    INAVSerial.write('$');
    INAVSerial.write('M');
    INAVSerial.write('<');

    INAVSerial.write(length);
    INAVSerial.write(command);

    for (uint8_t i = 0; i < length; i++)
        INAVSerial.write(payload[i]);

    INAVSerial.write(checksum);
}


// ============================================================
// MSP REQUEST
// ============================================================

void requestMSP(uint8_t command)
{
    sendMSP(command, nullptr, 0);
}


// ============================================================
// LITTLE-ENDIAN HELPERS
// ============================================================

uint16_t readU16(const uint8_t *p)
{
    return ((uint16_t)p[0]) |
           ((uint16_t)p[1] << 8);
}

int16_t readI16(const uint8_t *p)
{
    return (int16_t)readU16(p);
}

uint32_t readU32(const uint8_t *p)
{
    return ((uint32_t)p[0]) |
           ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

int32_t readI32(const uint8_t *p)
{
    return (int32_t)readU32(p);
}


// ============================================================
// MSP RESPONSE PARSER
// ============================================================

void processMSP(uint8_t command,
                uint8_t *payload,
                uint8_t length)
{
    switch (command)
    {
        // ----------------------------------------------------
        // GPS
        //
        // INAV MSP_RAW_GPS:
        // fix
        // sats
        // latitude
        // longitude
        // altitude
        // speed
        // course
        // ----------------------------------------------------

        case MSP_RAW_GPS:

            if (length >= 16)
            {
                gps.fix =
                    payload[0];

                gps.sats =
                    payload[1];

                gps.latitude =
                    readI32(&payload[2]);

                gps.longitude =
                    readI32(&payload[6]);

                gps.altitudeCm =
                    readI16(&payload[10]) * 100;

                gps.speedCms =
                    readU16(&payload[12]);

                gps.courseDeg =
                    readU16(&payload[14]);

                bool good =
                    gps.fix >= 2 &&
                    gps.sats >= MIN_SATS;

                if (good)
                {
                    if (gpsValidSince == 0)
                        gpsValidSince = millis();

                    if (millis() - gpsValidSince >=
                        GPS_VALID_TIME_MS)
                    {
                        gps.valid = true;
                    }
                }
                else
                {
                    gpsValidSince = 0;
                    gps.valid = false;
                }
            }

            break;


        // ----------------------------------------------------
        // HOME VECTOR
        // ----------------------------------------------------

        case MSP_COMP_GPS:

            if (length >= 5)
            {
                home.distanceMeters =
                    readU16(&payload[0]);

                home.directionDeg =
                    readI16(&payload[2]);

                // Direction 0 is meaningful, so distance is
                // used as part of the validity decision.
                home.valid =
                    gps.valid &&
                    home.distanceMeters > 1;
            }

            break;


        // ----------------------------------------------------
        // ATTITUDE
        // ----------------------------------------------------

        case MSP_ATTITUDE:

            if (length >= 6)
            {
                attitude.roll =
                    readI16(&payload[0]);

                attitude.pitch =
                    readI16(&payload[2]);

                attitude.heading =
                    readI16(&payload[4]);
            }

            break;


        default:
            break;
    }
}


// ============================================================
// SIMPLE MSP RX PARSER
// ============================================================

void serviceMSP()
{
    static uint8_t state = 0;

    static uint8_t dataSize = 0;
    static uint8_t command = 0;

    static uint8_t payload[64];
    static uint8_t payloadIndex = 0;

    static uint8_t checksum = 0;

    while (INAVSerial.available())
    {
        uint8_t c = INAVSerial.read();

        switch (state)
        {
            case 0:

                if (c == '$')
                    state = 1;

                break;


            case 1:

                if (c == 'M')
                    state = 2;
                else
                    state = 0;

                break;


            case 2:

                if (c == '>')
                    state = 3;
                else
                    state = 0;

                break;


            case 3:

                dataSize = c;
                checksum = c;
                payloadIndex = 0;
                state = 4;

                break;


            case 4:

                command = c;
                checksum ^= c;

                if (dataSize == 0)
                {
                    state = 6;
                }
                else
                {
                    state = 5;
                }

                break;


            case 5:

                if (payloadIndex < sizeof(payload))
                    payload[payloadIndex] = c;

                payloadIndex++;

                checksum ^= c;

                if (payloadIndex >= dataSize)
                    state = 6;

                break;


            case 6:

                if (checksum == c)
                {
                    processMSP(
                        command,
                        payload,
                        dataSize
                    );
                }

                state = 0;

                break;
        }
    }
}


// ============================================================
// MSP RC OUTPUT
// ============================================================

void sendRC()
{
    uint8_t payload[32];

    for (int i = 0; i < 8; i++)
    {
        payload[i * 2] =
            rcChannels[i] & 0xFF;

        payload[i * 2 + 1] =
            rcChannels[i] >> 8;
    }

    sendMSP(
        MSP_SET_RAW_RC,
        payload,
        16
    );
}


// ============================================================
// RC FAILSAFE
// ============================================================

bool isRcLost()
{
    return
        (millis() - lastRcPacket) >
        RC_TIMEOUT_MS;
}


// ============================================================
// ANGLE WRAP
// ============================================================

float wrap180(float angle)
{
    while (angle > 180.0f)
        angle -= 360.0f;

    while (angle < -180.0f)
        angle += 360.0f;

    return angle;
}


// ============================================================
// DRIFT LEARNING
// ============================================================

void updateDriftEstimate()
{
    if (!gps.valid)
        return;

    float speed =
        gps.speedCms / 100.0f;

    if (speed < MIN_GROUND_SPEED_MS)
        return;

    float heading =
        attitude.heading;

    float groundTrack =
        gps.courseDeg;

    float error =
        wrap180(groundTrack - heading);

    if (!driftInitialized)
    {
        driftDeg = error;
        driftInitialized = true;
    }
    else
    {
        driftDeg =
            driftDeg * (1.0f - DRIFT_ALPHA) +
            error * DRIFT_ALPHA;
    }
}


// ============================================================
// FAILSAFE DECISION
// ============================================================

void updateRecoveryState()
{
    bool lost = isRcLost();

    switch (recoveryState)
    {
        // ----------------------------------------------------
        // NORMAL
        // ----------------------------------------------------

        case STATE_NORMAL:

            if (lost)
            {
                Serial.println(
                    "RC LOST"
                );

                recoveryState =
                    STATE_RC_LOST;
            }

            break;


        // ----------------------------------------------------
        // RC LOST
        // ----------------------------------------------------

        case STATE_RC_LOST:

            if (!lost)
            {
                recoveryState =
                    STATE_NORMAL;

                Serial.println(
                    "RC RECOVERED"
                );

                break;
            }

            recoveryState =
                STATE_VALIDATE;

            break;


        // ----------------------------------------------------
        // VALIDATE
        // ----------------------------------------------------

        case STATE_VALIDATE:

            if (!lost)
            {
                recoveryState =
                    STATE_NORMAL;

                break;
            }

            if (gps.valid &&
                home.valid)
            {
                Serial.println(
                    "GPS + HOME VALID"
                );

                if (ENABLE_AUTONOMOUS_RTH)
                {
                    recoveryState =
                        STATE_RTH;
                }
                else
                {
                    Serial.println(
                        "RTH DISABLED FOR TESTING"
                    );
                }
            }
            else
            {
                Serial.println(
                    "GPS/HOME INVALID - FALLBACK"
                );

                recoveryState =
                    STATE_FALLBACK;
            }

            break;


        // ----------------------------------------------------
        // RTH
        // ----------------------------------------------------

        case STATE_RTH:

            if (!lost)
            {
                Serial.println(
                    "RC RETURNED"
                );

                recoveryState =
                    STATE_NORMAL;

                break;
            }

            break;


        // ----------------------------------------------------
        // FALLBACK
        // ----------------------------------------------------

        case STATE_FALLBACK:

            if (!lost)
            {
                recoveryState =
                    STATE_NORMAL;

                Serial.println(
                    "RC RETURNED"
                );
            }

            break;
    }
}


// ============================================================
// REQUEST TELEMETRY
// ============================================================

void pollINAV()
{
    static uint32_t lastPoll = 0;

    if (millis() - lastPoll < 100)
        return;

    lastPoll = millis();

    requestMSP(MSP_RAW_GPS);
    delay(2);

    requestMSP(MSP_COMP_GPS);
    delay(2);

    requestMSP(MSP_ATTITUDE);
}


// ============================================================
// SERIAL DEBUG
// ============================================================

void printStatus()
{
    static uint32_t last = 0;

    if (millis() - last < 1000)
        return;

    last = millis();

    Serial.println();
    Serial.println("========== STATUS ==========");

    Serial.print("RC valid: ");
    Serial.println(rcValid);

    Serial.print("RC lost: ");
    Serial.println(isRcLost());

    Serial.print("GPS fix: ");
    Serial.println(gps.fix);

    Serial.print("GPS sats: ");
    Serial.println(gps.sats);

    Serial.print("GPS valid: ");
    Serial.println(gps.valid);

    Serial.print("Lat: ");
    Serial.println(gps.latitude);

    Serial.print("Lon: ");
    Serial.println(gps.longitude);

    Serial.print("Altitude cm: ");
    Serial.println(gps.altitudeCm);

    Serial.print("Ground speed cm/s: ");
    Serial.println(gps.speedCms);

    Serial.print("Ground course: ");
    Serial.println(gps.courseDeg);

    Serial.print("Heading: ");
    Serial.println(attitude.heading);

    Serial.print("Home distance: ");
    Serial.println(home.distanceMeters);

    Serial.print("Home direction: ");
    Serial.println(home.directionDeg);

    Serial.print("Drift estimate: ");
    Serial.println(driftDeg);

    Serial.print("State: ");

    switch (recoveryState)
    {
        case STATE_NORMAL:
            Serial.println("NORMAL");
            break;

        case STATE_RC_LOST:
            Serial.println("RC_LOST");
            break;

        case STATE_VALIDATE:
            Serial.println("VALIDATE");
            break;

        case STATE_RTH:
            Serial.println("RTH");
            break;

        case STATE_FALLBACK:
            Serial.println("FALLBACK");
            break;
    }
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(115200);

    delay(1000);

    Serial.println();
    Serial.println(
        "ESP32 INAV 8 FAILSAFE"
    );

    // RC receiver
    RCSerial.begin(
        RC_BAUD,
        SERIAL_8N1,
        RC_RX_PIN,
        RC_TX_PIN
    );

    // INAV UART2
    INAVSerial.begin(
        MSP_BAUD,
        SERIAL_8N1,
        INAV_RX_PIN,
        INAV_TX_PIN
    );

    // Default RC values
    for (int i = 0; i < 16; i++)
        rcChannels[i] = 1500;

    // Typical initial values
    rcChannels[2] = 1000;  // throttle
    rcChannels[3] = 1500;  // yaw

    lastRcPacket = millis();

    Serial.println(
        "UARTs initialized"
    );
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
    /*
       TODO:
       CRSF decoder should update:

       rcChannels[0..15]

       and call:

       lastRcPacket = millis();
       rcValid = true;

       The decoder is intentionally kept separate from
       the failsafe/navigation code.
    */

    serviceMSP();

    pollINAV();

    updateDriftEstimate();

    updateRecoveryState();

    /*
       While RC is valid, transmit the received RC values.
    */

    static uint32_t lastRC = 0;

    if (millis() - lastRC >= 20)
    {
        lastRC = millis();

        if (recoveryState == STATE_NORMAL)
        {
            sendRC();
        }
    }

    printStatus();
}