/*
   ============================================================
   SkyGuard-INAV - ESP32 failsafe / GPS-loss escape supervisor
   ============================================================

   DEVELOPMENT VERSION - USE AT YOUR OWN RISK

   Aircraft: X4 multirotor
   FC:       INAV 8.0.1
   FC port:  UART configured for MSP

   RC input: CRSF / ExpressLRS receiver connected to ESP32
   FC link:  MSP

   INTENDED BEHAVIOR
   -----------------
   NORMAL:
     - CRSF channels are decoded and forwarded to INAV.
     - GPS, home, attitude, and MSP-link health are monitored.
     - A safe escape heading is learned/selected during normal flight.

   RC LOSS + GPS LOSS:
     - Stop forwarding stale pilot RC channels.
     - Activate ALT HOLD.
     - Activate HEADING mode.
     - Fly toward the configured safe escape heading.
     - Maintain a configured recovery altitude.
     - Periodically retry GPS.
     - If GPS becomes valid again, activate RTH.
     - If GPS does not recover, activate controlled landing.

   IMPORTANT SAFETY NOTES
   ----------------------
   - This firmware does NOT directly drive motors.
   - INAV remains responsible for stabilization, motor mixing,
     attitude control, and execution of flight modes.
   - The ESP32 only sends bounded RC-channel commands through MSP.
   - The AUX channel assignments below MUST match your INAV Modes tab.
   - Do not enable autonomous recovery until all bench tests pass.
   - Test first with propellers removed.
   - Verify that a powered-off or crashed ESP32 cannot prevent
     normal RC control or leave INAV in an unsafe mode.
*/

#include <Arduino.h>
#include <math.h>

// ============================================================
// USER CONFIGURATION
// ============================================================

// ---------- ESP32 UARTS ----------

// CRSF receiver is connected to ESP32 UART1.
HardwareSerial RCSerial(1);

// ESP32 MSP link is connected to INAV UART.
HardwareSerial INAVSerial(2);

// Change these to match your actual ESP32 wiring.
static const int RC_RX_PIN = 16;
static const int RC_TX_PIN = 17;

// ESP32 UART connected to INAV MSP UART.
static const int INAV_RX_PIN = 26;
static const int INAV_TX_PIN = 27;

// ExpressLRS / CRSF normal baud rate.
static const uint32_t RC_BAUD = 420000;

// MSP baud. INAV UART must be configured to the same value.
static const uint32_t MSP_BAUD = 115200;

// ---------- RC watchdog ----------

// A valid RC frame must arrive at least this often.
// 200 ms is a conservative starting value for CRSF.
static const uint32_t RC_FRAME_TIMEOUT_MS = 200;

// After RC loss, wait this long before activating the escape maneuver.
// This gives a brief RC dropout time to recover.
static const uint32_t RECOVERY_DECISION_DELAY_MS = 2000;

// RC must remain valid for this long before normal control is restored.
static const uint32_t RC_RECOVER_MS = 3000;

// ---------- GPS requirements ----------

// Minimum satellites required before GPS is considered usable.
static const uint8_t MIN_SATS = 8;

// GPS must remain valid for this long before it may be trusted.
static const uint32_t GPS_VALID_TIME_MS = 3000;

// If no fresh GPS MSP response arrives within this period,
// GPS is considered stale and must not be used for RTH.
static const uint32_t GPS_STALE_TIMEOUT_MS = 1000;

// If no fresh MSP response arrives within this period,
// the FC/MSP link is considered lost.
static const uint32_t MSP_STALE_TIMEOUT_MS = 1000;

// Maximum plausible GPS position jump.
// This is a basic spoofing / glitch plausibility check.
static const float MAX_GPS_JUMP_METERS_PER_SECOND = 50.0f;

// Maximum plausible GPS ground speed in m/s.
static const float MAX_GPS_SPEED_MS = 40.0f;

// ---------- Escape maneuver ----------

// Safe escape heading in degrees, magnetic/compass convention.
// DO NOT blindly use this value in flight.
// Set it to a direction known to be open and safe from your launch site.
static const float SAFE_ESCAPE_HEADING_DEG = 225.0f;

// Target altitude during the GPS-loss escape maneuver, in meters.
static const float ESCAPE_TARGET_ALTITUDE_M = 25.0f;

// Maximum commanded bank/roll during heading capture.
// Keep this conservative for the first tests.
static const float MAX_ESCAPE_ROLL_US = 150.0f;

// Maximum commanded forward pitch during escape.
static const float MAX_ESCAPE_PITCH_US = 120.0f;

// Heading error deadband in degrees.
// Within this error, no roll correction is commanded.
static const float HEADING_DEADBAND_DEG = 4.0f;

// Proportional gain for heading error to roll command.
// Start low. Increase only after bench testing.
static const float HEADING_KP = 3.0f;

// Maximum time allowed to fly the escape maneuver.
// After this, the system enters a controlled landing state.
static const uint32_t ESCAPE_MAX_DURATION_MS = 30000;

// How often to retry GPS / RTH during the escape maneuver.
static const uint32_t GPS_RETRY_INTERVAL_MS = 1000;

// GPS must be valid this long before RTH may be activated.
static const uint32_t GPS_RECOVERY_CONFIRM_MS = 3000;

// ---------- RTH ----------

// Keep FALSE until the complete recovery sequence has been bench tested.
static const bool ENABLE_AUTONOMOUS_RTH = false;

// ---------- Drift / heading-track learning ----------

// Ignore heading/track comparison when ground speed is below this value.
static const float MIN_GROUND_SPEED_MS = 2.0f;

// Low-pass coefficient for heading-track difference.
static const float DRIFT_ALPHA = 0.02f;

// ============================================================
// MSP COMMANDS
// ============================================================

#define MSP_RAW_GPS       106
#define MSP_COMP_GPS      107
#define MSP_ATTITUDE      108
#define MSP_ALTITUDE      109
#define MSP_SET_RAW_RC    200
#define MSP_NAV_STATUS    121

// ============================================================
// RC CHANNEL MAPPING
// ============================================================
//
// IMPORTANT:
// Verify these indices in the INAV Receiver tab.
//
// This firmware assumes a common AETR-style mapping:
//   0 = Roll
//   1 = Pitch
//   2 = Throttle
//   3 = Yaw
//
// AUX channels must be configured in INAV Modes.
// Do NOT assume these defaults match your radio or receiver.
//
static const uint8_t CH_ROLL = 0;
static const uint8_t CH_PITCH = 1;
static const uint8_t CH_THROTTLE = 2;
static const uint8_t CH_YAW = 3;

// Example AUX assignments.
// Change these to match your INAV Modes configuration.
static const uint8_t CH_AUX_ALT_HOLD = 5;
static const uint8_t CH_AUX_HEADING = 6;
static const uint8_t CH_AUX_RTH = 7;

// PWM values used by the escape controller.
static const uint16_t RC_CENTER_US = 1500;
static const uint16_t RC_MIN_US = 1000;
static const uint16_t RC_MAX_US = 2000;

// ============================================================
// RC CHANNEL STORAGE
// ============================================================

uint16_t rcChannels[16];

// True only after at least one valid CRSF channel frame.
bool rcValid = false;

// Timestamp of the last valid CRSF channel frame.
uint32_t lastRcPacketMs = 0;

// Timestamp when continuously valid RC recovery began.
uint32_t rcRecoveredSinceMs = 0;

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

// Timestamp when continuously valid GPS began.
uint32_t gpsValidSinceMs = 0;

// Timestamp of the last received GPS MSP response.
uint32_t lastGpsRxMs = 0;

// Last trusted GPS latitude/longitude, scaled by 1e7.
int32_t lastTrustedLatitude = 0;
int32_t lastTrustedLongitude = 0;

// Timestamp of the last trusted GPS position.
uint32_t lastTrustedGpsMs = 0;

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

uint32_t lastHomeRxMs = 0;

// ============================================================
// ATTITUDE STATE
// ============================================================

struct AttitudeData
{
    int16_t roll = 0;
    int16_t pitch = 0;
    int16_t heading = 0;
};

AttitudeData attitude;

uint32_t lastAttitudeRxMs = 0;

// ============================================================
// HEADING / TRACK DIFFERENCE
// ============================================================
//
// This is NOT a wind estimate.
//
// On a multirotor, heading and ground track can differ because
// the aircraft may intentionally fly sideways or backward.
//
// It may be used only as a diagnostic / plausibility indicator.
//
float headingTrackDifferenceDeg = 0.0f;
bool headingTrackInitialized = false;

// ============================================================
// RECOVERY STATE MACHINE
// ============================================================

enum RecoveryState
{
    STATE_NORMAL,

    // RC frame has stopped arriving.
    STATE_RC_LOST,

    // RC has been lost long enough to begin recovery decisions.
    STATE_VALIDATE,

    // GPS/home are valid: request INAV RTH.
    STATE_RTH,

    // GPS is lost or untrusted: activate ALT HOLD + HEADING escape.
    STATE_ESCAPE_ALT_HOLD,
    STATE_ESCAPE_HEADING,
    STATE_ESCAPE_FLYING,

    // GPS recovered during escape: transition to RTH.
    STATE_GPS_RECOVERED,

    // Escape time expired without GPS recovery: controlled landing.
    STATE_LAND,

    // MSP/FC communication is unavailable.
    STATE_MSP_FAILURE
};

RecoveryState recoveryState = STATE_NORMAL;

// Timestamp when RC was first detected as lost.
uint32_t rcLostSinceMs = 0;

// Timestamp when the escape maneuver began.
uint32_t escapeStartedMs = 0;

// Timestamp of the last GPS retry during escape.
uint32_t lastGpsRetryMs = 0;

// Timestamp when GPS became continuously valid during escape.
uint32_t gpsRecoverySinceMs = 0;

// ============================================================
// MSP UTILITIES
// ============================================================

// ------------------------------------------------------------
// Send a legacy MSP request/response frame.
// ------------------------------------------------------------
void sendMSP(uint8_t command, const uint8_t *payload, uint8_t length)
{
    uint8_t checksum = length ^ command;

    for (uint8_t i = 0; i < length; i++)
    {
        checksum ^= payload[i];
    }

    INAVSerial.write('$');
    INAVSerial.write('M');
    INAVSerial.write('<');

    INAVSerial.write(length);
    INAVSerial.write(command);

    for (uint8_t i = 0; i < length; i++)
    {
        INAVSerial.write(payload[i]);
    }

    INAVSerial.write(checksum);
}

// ------------------------------------------------------------
// Request an MSP telemetry message from INAV.
// ------------------------------------------------------------
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
//
// NOTE:
// Before relying on GPS fix, course, and altitude fields,
// verify the exact INAV 8.0.1 MSP payload layout for your build.
//
void processMSP(uint8_t command, uint8_t *payload, uint8_t length)
{
    switch (command)
    {
        case MSP_RAW_GPS:
        {
            if (length >= 16)
            {
                uint8_t previousFix = gps.fix;
                uint8_t previousSats = gps.sats;

                gps.fix = payload[0];
                gps.sats = payload[1];

                gps.latitude = readI32(&payload[2]);
                gps.longitude = readI32(&payload[6]);

                gps.altitudeCm = readI16(&payload[10]) * 100;
                gps.speedCms = readU16(&payload[12]);
                gps.courseDeg = readU16(&payload[14]);

                lastGpsRxMs = millis();

                // Basic GPS validity test.
                bool good =
                    gps.fix >= 2 &&
                    gps.sats >= MIN_SATS;

                // Basic plausibility checks.
                float speedMs = gps.speedCms / 100.0f;

                if (speedMs > MAX_GPS_SPEED_MS)
                {
                    good = false;
                }

                // Detect an implausible position jump.
                if (lastTrustedGpsMs != 0 && good)
                {
                    float dtSec =
                        (millis() - lastTrustedGpsMs) / 1000.0f;

                    if (dtSec > 0.1f)
                    {
                        // Approximate local distance using small-angle
                        // latitude/longitude scaling. This is only a
                        // sanity check, not a precise geodesic calculation.
                        float dLat =
                            (gps.latitude - lastTrustedLatitude) / 1e7f;

                        float dLon =
                            (gps.longitude - lastTrustedLongitude) / 1e7f;

                        float metersPerDegLat = 111320.0f;
                        float metersPerDegLon =
                            111320.0f *
                            cosf(gps.latitude / 1e7f * DEG_TO_RAD);

                        float dx = dLon * metersPerDegLon;
                        float dy = dLat * metersPerDegLat;

                        float jumpMeters = sqrtf(dx * dx + dy * dy);

                        float jumpRate = jumpMeters / dtSec;

                        if (jumpRate > MAX_GPS_JUMP_METERS_PER_SECOND)
                        {
                            good = false;
                        }
                    }
                }

                if (good)
                {
                    if (gpsValidSinceMs == 0)
                    {
                        gpsValidSinceMs = millis();
                    }

                    if (millis() - gpsValidSinceMs >= GPS_VALID_TIME_MS)
                    {
                        gps.valid = true;

                        lastTrustedLatitude = gps.latitude;
                        lastTrustedLongitude = gps.longitude;
                        lastTrustedGpsMs = millis();
                    }
                }
                else
                {
                    gpsValidSinceMs = 0;
                    gps.valid = false;
                }

                (void)previousFix;
                (void)previousSats;
            }

            break;
        }

        case MSP_COMP_GPS:
        {
            if (length >= 5)
            {
                home.distanceMeters = readU16(&payload[0]);
                home.directionDeg = readI16(&payload[2]);

                lastHomeRxMs = millis();

                // A distance greater than 1 m does not prove that home
                // is valid. It only proves that a home vector was received.
                home.valid =
                    gps.valid &&
                    home.distanceMeters > 1;
            }

            break;
        }

        case MSP_ATTITUDE:
        {
            if (length >= 6)
            {
                attitude.roll = readI16(&payload[0]);
                attitude.pitch = readI16(&payload[2]);
                attitude.heading = readI16(&payload[4]);

                lastAttitudeRxMs = millis();
            }

            break;
        }

        default:
            break;
    }
}

// ============================================================
// MSP RECEIVER PARSER
// ============================================================

void serviceMSP()
{
    static uint8_t state = 0;
    static uint8_t dataSize = 0;
    static uint8_t command = 0;
    static uint8_t payload[64];
    static uint8_t payloadIndex = 0;
    static uint8_t checksum = 0;
    static uint32_t lastByteMs = 0;

    uint32_t now = millis();

    // Reset parser if bytes stop arriving in the middle of a frame.
    if (state != 0 && now - lastByteMs > 50)
    {
        state = 0;
    }

    while (INAVSerial.available())
    {
        uint8_t c = INAVSerial.read();
        lastByteMs = now;

        switch (state)
        {
            case 0:
                if (c == '$')
                {
                    state = 1;
                }
                break;

            case 1:
                state = (c == 'M') ? 2 : 0;
                break;

            case 2:
                state = (c == '>') ? 3 : 0;
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

                state = (dataSize == 0) ? 6 : 5;
                break;

            case 5:
                if (payloadIndex < sizeof(payload))
                {
                    payload[payloadIndex] = c;
                }

                payloadIndex++;
                checksum ^= c;

                if (payloadIndex >= dataSize)
                {
                    state = 6;
                }
                break;

            case 6:
                if (checksum == c)
                {
                    // Do not pass a length larger than the buffer.
                    uint8_t safeLength = dataSize;

                    if (safeLength > sizeof(payload))
                    {
                        safeLength = sizeof(payload);
                    }

                    processMSP(command, payload, safeLength);
                }

                state = 0;
                break;
        }
    }
}

// ============================================================
// CRSF RECEIVER DECODER
// ============================================================
//
// This is a minimal CRSF RC-channels-frame decoder.
//
// It supports the standard CRSF RC_CHANNELS_PACKED frame:
//   Type 0x16
//   22 bytes payload
//   16 channels, 11 bits each
//
// CRC8 is the CRSF DVB-S2 checksum.
//
// IMPORTANT:
// Validate this against your actual receiver and firmware version.
//

static const uint8_t CRSF_SYNC_BYTE = 0xC8;
static const uint8_t CRSF_FRAME_TYPE_RC_CHANNELS = 0x16;

// Update RC channels from a valid CRSF RC channel frame.
void decodeCrsfChannels(const uint8_t *payload)
{
    uint32_t packed =
        ((uint32_t)payload[0]) |
        ((uint32_t)payload[1] << 8) |
        ((uint32_t)payload[2] << 16) |
        ((uint32_t)payload[3] << 24);

    // CRSF packs 16 x 11-bit channels into 22 bytes.
    // The extraction below processes channels sequentially.
    uint32_t bitOffset = 0;

    for (uint8_t channel = 0; channel < 16; channel++)
    {
        uint32_t value = 0;

        // Extract 11 bits, potentially spanning byte boundaries.
        for (uint8_t bit = 0; bit < 11; bit++)
        {
            uint32_t byteIndex = (bitOffset + bit) / 8;
            uint32_t bitIndex = (bitOffset + bit) % 8;

            if (byteIndex < 22)
            {
                uint32_t bitValue =
                    (payload[byteIndex] >> bitIndex) & 0x01;

                value |= bitValue << bit;
            }
        }

        bitOffset += 11;

        // Convert CRSF 11-bit value to approximately 988-2012 us.
        // This is the common CRSF channel scaling.
        uint16_t pulseUs =
            988 + (uint16_t)(((uint32_t)value * 1024) / 2047);

        // Clamp to a safe RC range.
        if (pulseUs < RC_MIN_US)
        {
            pulseUs = RC_MIN_US;
        }

        if (pulseUs > RC_MAX_US)
        {
            pulseUs = RC_MAX_US;
        }

        rcChannels[channel] = pulseUs;
    }

    rcValid = true;
    lastRcPacketMs = millis();
}

// CRC8 used by CRSF.
uint8_t crsfCrc8(const uint8_t *data, uint8_t length)
{
    uint8_t crc = 0;

    for (uint8_t i = 0; i < length; i++)
    {
        crc ^= data[i];

        for (uint8_t bit = 0; bit < 8; bit++)
        {
            if (crc & 0x80)
            {
                crc = (crc << 1) ^ 0xD5;
            }
            else
            {
                crc <<= 1;
            }
        }
    }

    return crc;
}

// Read and decode CRSF frames from the receiver UART.
void serviceRC()
{
    static uint8_t buffer[64];
    static uint8_t index = 0;
    static uint8_t expectedLength = 0;

    while (RCSerial.available())
    {
        uint8_t c = RCSerial.read();

        if (index == 0)
        {
            // Wait for CRSF address/sync byte.
            if (c != CRSF_SYNC_BYTE)
            {
                continue;
            }

            buffer[index++] = c;
        }
        else if (index == 1)
        {
            // Second byte is frame length.
            expectedLength = c;

            if (expectedLength < 2 || expectedLength > sizeof(buffer) - 2)
            {
                index = 0;
                continue;
            }

            buffer[index++] = c;
        }
        else
        {
            buffer[index++] = c;

            // Full frame: sync + length + payload + CRC.
            if (index >= expectedLength + 2)
            {
                uint8_t payloadLength = expectedLength - 2;

                uint8_t receivedCrc = buffer[index - 1];
                uint8_t calculatedCrc =
                    crsfCrc8(&buffer[2], payloadLength);

                if (receivedCrc == calculatedCrc &&
                    buffer[2] == CRSF_FRAME_TYPE_RC_CHANNELS &&
                    payloadLength == 22)
                {
                    decodeCrsfChannels(&buffer[3]);
                }

                index = 0;
            }
        }
    }
}

// ============================================================
// RC HEALTH
// ============================================================

bool isRcFrameLost()
{
    return !rcValid ||
           (millis() - lastRcPacketMs) > RC_FRAME_TIMEOUT_MS;
}

// ============================================================
// MSP / TELEMETRY HEALTH
// ============================================================

bool isMspStale()
{
    return (millis() - lastGpsRxMs) > MSP_STALE_TIMEOUT_MS &&
           (millis() - lastAttitudeRxMs) > MSP_STALE_TIMEOUT_MS;
}

bool isGpsStale()
{
    return (millis() - lastGpsRxMs) > GPS_STALE_TIMEOUT_MS;
}

bool isHomeStale()
{
    return (millis() - lastHomeRxMs) > GPS_STALE_TIMEOUT_MS;
}

bool isAttitudeStale()
{
    return (millis() - lastAttitudeRxMs) > MSP_STALE_TIMEOUT_MS;
}

// ============================================================
// MSP RC OUTPUT
// ============================================================

// Send the first 8 RC channels to INAV.
// Verify whether your INAV build expects 8 or 16 channels.
void sendRC()
{
    uint8_t payload[16];

    for (int i = 0; i < 8; i++)
    {
        payload[i * 2] = rcChannels[i] & 0xFF;
        payload[i * 2 + 1] = rcChannels[i] >> 8;
    }

    sendMSP(MSP_SET_RAW_RC, payload, 16);
}

// ============================================================
// ANGLE HELPERS
// ============================================================

float wrap180(float angle)
{
    while (angle > 180.0f)
    {
        angle -= 360.0f;
    }

    while (angle < -180.0f)
    {
        angle += 360.0f;
    }

    return angle;
}

float wrap360(float angle)
{
    while (angle < 0.0f)
    {
        angle += 360.0f;
    }

    while (angle >= 360.0f)
    {
        angle -= 360.0f;
    }

    return angle;
}

// ============================================================
// HEADING / TRACK DIFFERENCE
// ============================================================

void updateHeadingTrackDifference()
{
    if (!gps.valid || isAttitudeStale())
    {
        return;
    }

    float speed = gps.speedCms / 100.0f;

    if (speed < MIN_GROUND_SPEED_MS)
    {
        return;
    }

    float heading = attitude.heading;
    float groundTrack = gps.courseDeg;

    float error = wrap180(groundTrack - heading);

    if (!headingTrackInitialized)
    {
        headingTrackDifferenceDeg = error;
        headingTrackInitialized = true;
    }
    else
    {
        // Use shortest angular difference to avoid problems near 180°.
        headingTrackDifferenceDeg = wrap180(
            headingTrackDifferenceDeg +
            DRIFT_ALPHA * wrap180(error - headingTrackDifferenceDeg)
        );
    }
}

// ============================================================
// SAFE HEADING SELECTION
// ============================================================
//
// This uses the last trusted home bearing when available.
//
// If home was never valid, it uses the configured safe escape heading.
//
float selectSafeEscapeHeading()
{
    if (home.valid && !isHomeStale())
    {
        return wrap360((float)home.directionDeg);
    }

    return SAFE_ESCAPE_HEADING_DEG;
}

// ============================================================
// RC COMMAND HELPERS
// ============================================================

uint16_t constrainUs(int value)
{
    if (value < RC_MIN_US)
    {
        return RC_MIN_US;
    }

    if (value > RC_MAX_US)
    {
        return RC_MAX_US;
    }

    return (uint16_t)value;
}

// Convert a heading error into a bounded roll command.
//
// Positive heading error means the target heading is clockwise
// from the current heading, using standard compass convention.
//
// The sign must be verified on the bench for your aircraft.
uint16_t headingRollCommand(float targetHeadingDeg)
{
    float currentHeading = attitude.heading;

    float error =
        wrap180(targetHeadingDeg - currentHeading);

    if (fabsf(error) < HEADING_DEADBAND_DEG)
    {
        return RC_CENTER_US;
    }

    float rollCommand = HEADING_KP * error;

    if (rollCommand > MAX_ESCAPE_ROLL_US)
    {
        rollCommand = MAX_ESCAPE_ROLL_US;
    }

    if (rollCommand < -MAX_ESCAPE_ROLL_US)
    {
        rollCommand = -MAX_ESCAPE_ROLL_US;
    }

    return constrainUs(RC_CENTER_US + (int)rollCommand);
}

// Small forward pitch command used to move away from the area.
uint16_t escapePitchCommand()
{
    return constrainUs(RC_CENTER_US + (int)MAX_ESCAPE_PITCH_US);
}

// ============================================================
// RECOVERY STATE MACHINE
// ============================================================

void updateRecoveryState()
{
    bool rcLost = isRcFrameLost();
    bool mspFailed = isMspStale();
    bool gpsUsable =
        gps.valid &&
        !isGpsStale() &&
        home.valid &&
        !isHomeStale();

    switch (recoveryState)
    {
        case STATE_NORMAL:
        {
            if (mspFailed)
            {
                recoveryState = STATE_MSP_FAILURE;
                break;
            }

            if (rcLost)
            {
                rcLostSinceMs = millis();
                recoveryState = STATE_RC_LOST;

                Serial.println("RC FRAME LOST");
            }

            break;
        }

        case STATE_RC_LOST:
        {
            if (!rcLost)
            {
                recoveryState = STATE_NORMAL;
                Serial.println("RC RECOVERED - SHORT DROPOUT");
                break;
            }

            if (millis() - rcLostSinceMs >= RECOVERY_DECISION_DELAY_MS)
            {
                recoveryState = STATE_VALIDATE;
                Serial.println("RC LOSS CONFIRMED - VALIDATING");
            }

            break;
        }

        case STATE_VALIDATE:
        {
            if (!rcLost)
            {
                recoveryState = STATE_NORMAL;
                break;
            }

            if (mspFailed)
            {
                recoveryState = STATE_MSP_FAILURE;
                break;
            }

            if (gpsUsable)
            {
                if (ENABLE_AUTONOMOUS_RTH)
                {
                    recoveryState = STATE_RTH;
                    Serial.println("GPS/HOME VALID - RTH REQUESTED");
                }
                else
                {
                    // Do not repeatedly print this in the main loop.
                    recoveryState = STATE_ESCAPE_ALT_HOLD;
                    Serial.println("RTH DISABLED - ESCAPE TEST MODE");
                }
            }
            else
            {
                recoveryState = STATE_ESCAPE_ALT_HOLD;
                Serial.println("GPS/HOME INVALID - ESCAPE MODE");
            }

            break;
        }

        case STATE_RTH:
        {
            if (!rcLost)
            {
                recoveryState = STATE_NORMAL;
                Serial.println("RC RETURNED - RTH HANDOFF");
                break;
            }

            // If GPS becomes invalid during RTH, fall back to escape.
            if (!gpsUsable)
            {
                recoveryState = STATE_ESCAPE_ALT_HOLD;
                Serial.println("GPS LOST DURING RTH - ESCAPE MODE");
            }

            break;
        }

        case STATE_ESCAPE_ALT_HOLD:
        {
            if (!rcLost)
            {
                recoveryState = STATE_NORMAL;
                break;
            }

            if (mspFailed)
            {
                recoveryState = STATE_MSP_FAILURE;
                break;
            }

            // In a full implementation, verify ALT HOLD through
            // INAV navigation/status telemetry before proceeding.
            recoveryState = STATE_ESCAPE_HEADING;
            Serial.println("ALT HOLD REQUESTED");

            break;
        }

        case STATE_ESCAPE_HEADING:
        {
            if (!rcLost)
            {
                recoveryState = STATE_NORMAL;
                break;
            }

            if (mspFailed)
            {
                recoveryState = STATE_MSP_FAILURE;
                break;
            }

            escapeStartedMs = millis();
            lastGpsRetryMs = 0;
            gpsRecoverySinceMs = 0;

            recoveryState = STATE_ESCAPE_FLYING;
            Serial.println("HEADING MODE REQUESTED - ESCAPE ACTIVE");

            break;
        }

        case STATE_ESCAPE_FLYING:
        {
            if (!rcLost)
            {
                recoveryState = STATE_NORMAL;
                Serial.println("RC RETURNED - ESCAPE CANCELLED");
                break;
            }

            if (mspFailed)
            {
                recoveryState = STATE_MSP_FAILURE;
                break;
            }

            // Periodically check whether GPS has recovered.
            if (millis() - lastGpsRetryMs >= GPS_RETRY_INTERVAL_MS)
            {
                lastGpsRetryMs = millis();

                if (gpsUsable)
                {
                    if (gpsRecoverySinceMs == 0)
                    {
                        gpsRecoverySinceMs = millis();
                    }

                    if (millis() - gpsRecoverySinceMs >=
                        GPS_RECOVERY_CONFIRM_MS)
                    {
                        recoveryState = STATE_GPS_RECOVERED;
                        Serial.println("GPS RECOVERED - RTH HANDOFF");
                    }
                }
                else
                {
                    gpsRecoverySinceMs = 0;
                }
            }

            // If GPS does not recover, enter controlled landing.
            if (millis() - escapeStartedMs >= ESCAPE_MAX_DURATION_MS)
            {
                recoveryState = STATE_LAND;
                Serial.println("ESCAPE TIMEOUT - CONTROLLED LANDING");
            }

            break;
        }

        case STATE_GPS_RECOVERED:
        {
            if (!rcLost)
            {
                recoveryState = STATE_NORMAL;
                break;
            }

            if (gpsUsable && ENABLE_AUTONOMOUS_RTH)
            {
                recoveryState = STATE_RTH;
                Serial.println("RTH REQUESTED AFTER GPS RECOVERY");
            }
            else if (!gpsUsable)
            {
                recoveryState = STATE_ESCAPE_ALT_HOLD;
                Serial.println("GPS LOST AGAIN - ESCAPE MODE");
            }

            break;
        }

        case STATE_LAND:
        {
            if (!rcLost)
            {
                recoveryState = STATE_NORMAL;
                Serial.println("RC RETURNED - LANDING CANCELLED");
            }

            break;
        }

        case STATE_MSP_FAILURE:
        {
            // Do not attempt to control the aircraft if the FC link
            // is unreliable. Let INAV's native failsafe handle it.
            if (!mspFailed)
            {
                recoveryState = STATE_NORMAL;
                Serial.println("MSP LINK RECOVERED");
            }

            break;
        }
    }
}

// ============================================================
// ESCAPE CONTROL OUTPUT
// ============================================================

void updateEscapeControls()
{
    if (recoveryState != STATE_ESCAPE_FLYING)
    {
        return;
    }

    float targetHeading = selectSafeEscapeHeading();

    // Command bounded roll to rotate toward the escape heading.
    rcChannels[CH_ROLL] = headingRollCommand(targetHeading);

    // Command a small forward pitch to move away from the area.
    rcChannels[CH_PITCH] = escapePitchCommand();

    // Keep yaw centered; heading mode controls heading.
    rcChannels[CH_YAW] = RC_CENTER_US;

    // Do not directly command throttle here.
    // ALT HOLD should manage altitude.
    // Verify the correct neutral/hover throttle value on the bench.
    rcChannels[CH_THROTTLE] = RC_CENTER_US;
}

// ============================================================
// REQUEST TELEMETRY
// ============================================================

void pollINAV()
{
    static uint32_t lastPoll = 0;

    if (millis() - lastPoll < 100)
    {
        return;
    }

    lastPoll = millis();

    requestMSP(MSP_RAW_GPS);

    // Avoid using delay() in the main loop where possible.
    // This small delay separates MSP requests.
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
    {
        return;
    }

    last = millis();

    Serial.println();
    Serial.println("========== STATUS ==========");

    Serial.print("RC valid: ");
    Serial.println(rcValid);

    Serial.print("RC frame lost: ");
    Serial.println(isRcFrameLost());

    Serial.print("GPS valid: ");
    Serial.println(gps.valid);

    Serial.print("GPS stale: ");
    Serial.println(isGpsStale());

    Serial.print("GPS fix: ");
    Serial.println(gps.fix);

    Serial.print("GPS sats: ");
    Serial.println(gps.sats);

    Serial.print("MSP stale: ");
    Serial.println(isMspStale());

    Serial.print("Home valid: ");
    Serial.println(home.valid);

    Serial.print("Heading: ");
    Serial.println(attitude.heading);

    Serial.print("Escape heading: ");
    Serial.println(selectSafeEscapeHeading());

    Serial.print("Heading/track diff: ");
    Serial.println(headingTrackDifferenceDeg);

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

        case STATE_ESCAPE_ALT_HOLD:
            Serial.println("ESCAPE_ALT_HOLD");
            break;

        case STATE_ESCAPE_HEADING:
            Serial.println("ESCAPE_HEADING");
            break;

        case STATE_ESCAPE_FLYING:
            Serial.println("ESCAPE_FLYING");
            break;

        case STATE_GPS_RECOVERED:
            Serial.println("GPS_RECOVERED");
            break;

        case STATE_LAND:
            Serial.println("LAND");
            break;

        case STATE_MSP_FAILURE:
            Serial.println("MSP_FAILURE");
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
    Serial.println("SkyGuard-INAV ESP32 FAILSAFE");
    Serial.println("DEVELOPMENT VERSION - DO NOT FLY");

    // Initialize all RC channels to center.
    for (int i = 0; i < 16; i++)
    {
        rcChannels[i] = RC_CENTER_US;
    }

    // Do not assume the aircraft is armed or flying at startup.
    // Do not send RC data until a valid CRSF frame has been received.
    rcValid = false;
    lastRcPacketMs = 0;

    // Initialize receiver UART.
    RCSerial.begin(
        RC_BAUD,
        SERIAL_8N1,
        RC_RX_PIN,
        RC_TX_PIN
    );

    // Initialize MSP UART.
    INAVSerial.begin(
        MSP_BAUD,
        SERIAL_8N1,
        INAV_RX_PIN,
        INAV_TX_PIN
    );

    Serial.println("UARTs initialized");
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop()
{
    uint32_t now = millis();

    // Read and decode CRSF receiver frames.
    serviceRC();

    // Read and decode MSP responses from INAV.
    serviceMSP();

    // Request GPS, home, and attitude telemetry.
    pollINAV();

    // Update diagnostic heading/track difference.
    updateHeadingTrackDifference();

    // Update recovery state machine.
    updateRecoveryState();

    // If in escape mode, calculate bounded RC commands.
    updateEscapeControls();

    // Send RC data to INAV at 50 Hz.
    static uint32_t lastRcTxMs = 0;

    if (now - lastRcTxMs >= 20)
    {
        lastRcTxMs = now;

        // Send only after valid RC has been received.
        if (rcValid)
        {
            sendRC();
        }
    }

    // Print status once per second.
    printStatus();
}