/**
    =========
    METERMAN METERNODE
    meternode.cpp
    =========
    Operates a 'pulse meter node' that counts and records LED pulses into 'read
    entries', corresponding to a set interval (e.g. a 15s period).  These
    entries are periodically transmitted to a receiver(s) over packet radio.
    The meter node device this code was written for is designed for
    non-invasive, 'permissionless' monitoring of electricity smart meters, but
    the code and meter node device could be used to count the pulsing of any
    LED.  A CT clamp circuit is also included for reading AC current.

    The meter node has a corresponding gateway receiver, which takes the form of
    a Raspberry Pi 'Hat' (daughterboard).  This gateway could aggregate multiple
    nodes. These are part of a basic monitoring system as described at
    http://leehonan.com/meterman.

    The meter node can be interacted with over serial (FTDI) and also packet
    radio messages.

    Meter node hardware consists of a custom board with a basic ATMEGA 328P
    circuit, an RFM69 radio, a RTC (PCF2123), battery (or DC adapter) with
    voltage divider, FTDI header, a sub-circuit to integrate a 'puck' that goes
    over the LED being counted, and a CT clamp sub-circuit for AC current
    sensing.  The puck attaches using a cable and consists of a photoresistor
    and a LED that flashes in unison with the smart meter's LED (other modes are
    configurable, including turning this off to conserve battery - recommended).

    Uses the following libraries (with licencing):  RadioHead (GPL Version 2
    Licence), PCF2123 (MIT Licence), LowPower (Creative Commons
    Attribution-ShareAlike 3.0 Unported), and EMonLib (GPL Version 3).

    This program is licenced as follows below.

    -----------
    MIT License
    -----------

    Copyright (c) 2017 Lee Honan (lee at leehonan dot com).

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to
    deal in the Software without restriction, including without limitation the
    rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
    sell copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
    IN THE SOFTWARE.
 */


// *****************************************************************************
//    Main Config Parameters
//    'DEF_*' constants are defaults for configuration variables of the same
//    name. Most of these will be stored to EEPROM and can be updated by
//    serial terminal commands.

//    Have minimised use of #DEFINE, #IFDEF etc for easier debugging.
// *****************************************************************************

#include <Arduino.h>
#include <avr/wdt.h>
#include <EEPROM.h>

// http://www.airspayce.com/mikem/arduino/RadioHead/
#include <RH_RF69.h>
#include <RHReliableDatagram.h>

// https://github.com/LowPowerLab/LowPower/archive/master.zip
#include <LowPower.h>

// https://github.com/jvsalo/pcf2123/archive/master.zip
// uses https://github.com/PaulStoffregen/Time/archive/master.zip
#include <PCF2123.h>

// https://github.com/openenergymonitor/EmonLib
#include <EmonLib.h>

static const int8_t FW_VERSION = 3;
// TODO: test/prove CT code (circuit works)
// TODO: debug crazy time setting on isolated nodes (e.g. 2093-00-05 13:42:21) -
//          PREQ failure?

// Log Levels
typedef enum {
    logNull = 0,        // outputs to serial regardless of runtime log level
    logError = 1,
    logWarn = 2,
    logInfo = 3,
    logDebug = 4
} LogLev;

// Default runtime log level.
// Can control through serial command, save to EEPROM.
static const LogLev DEF_LOG_LEVEL = logDebug;

// if using HW/HCW module, must set to true
static const bool RADIO_HIGH_POWER = false;

// Initial power level in dBm.  Use -18 to +13 for W/CW, -14 to +20 for HW/HCW:
static const int8_t DEF_TX_POWER = 0;

// Target RSSI for auto-tuning, 0 is disable auto-tuning
static const int8_t DEF_TGT_RSSI = -75;

// Node ID and Gateway ID.  Gateway is usually 1.  Node between 2 and 254.
// 255 is broadcast (RadioHead => RH_BROADCAST_ADDRESS).
static const uint8_t DEF_NODE_ID = 2;
static const uint8_t DEF_GATEWAY_ID = 1;

// Network octets, akin to IP address but with an extra subnet (as the 4 octets
// define a subnet, with node addressing within this).
// Need at least 2 octets to be non-zero, so will have 3rd and 4th begin at 1.
static const uint8_t DEF_NETWORK_ID_O1 = 0;  //0 to 254
static const uint8_t DEF_NETWORK_ID_O2 = 0;  //0 to 254
static const uint8_t DEF_NETWORK_ID_O3 = 1;  //1 to 254 by convention
static const uint8_t DEF_NETWORK_ID_O4 = 1;  //1 to 254 by convention

// AES 128b encryption key, shared amongst nodes.  Must be 16 chars/bytes,
// chars ASCII 32 - 126 (space, symbols, letters, numbers).
static const uint8_t KEY_LENGTH = 16;
static const uint8_t DEF_ENCRYPT_KEY[KEY_LENGTH] =
        {'C','H','A','N','G','E','_','M','E','_','P','L','E','A','S','E'};

static const uint32_t SERIAL_BAUD = 115200;

// config whether node can sleep
static const bool SLEEP_ENABLED = true;
bool canSleep = true;   // used at runtime if SLEEP_ENABLED

// Default meter unit.  Unit of measure for pulse count, only used for logging,
// display, etc - intelligence is at consumer/Gateway.  E.g. "Wh", "*0.1Wh",
// "mls"
static const char DEF_METER_UNIT[KEY_LENGTH] = {"Wh"};

// Meter interval resolution in seconds.  This sets the period (max 255s) over
// which reads are accumulated into a single 'reading' - e.g. at 5s a read entry
// will represent 5s of reads (not less, sometimes more with sleep causing
// processing latency). Setting this to 0 will disable intervals, there will be
// one entry per read.
static const uint8_t DEF_METER_INT_RES_SEC = 30;

static const bool CT_ENABLED = false;

// *****************************************************************************
//    General Init - Pins
// *****************************************************************************

static const uint8_t RADIO_INTERRUPT_PIN = 2;   // IC4
                                                // uses 328P interrupt 0

static const uint8_t RADIO_SS_PIN = 10;         // IC16
                                                // HIGH is off, LOW on

static const uint8_t PR_INTERRUPT_PIN = 3;      // IC5
                                                // uses 328P interrupt 1
static const uint8_t PUCK_LED_PIN = 4;          // IC6
static const uint8_t RTC_SS_PIN = 7;            // IC13
                                                // HIGH is off, LOW on
static const uint8_t BUTTON_PIN = 6;            // IC12
                                                // auxiliary button
static const uint8_t VIN_DIV_PIN = A0;          // IC23
static const uint8_t CT_CLAMP_PIN = A1;         // IC24

// *****************************************************************************
//    General Init - General Variables
// *****************************************************************************

// variable to hold MCU reset cause
uint8_t resetFlags __attribute__ ((section(".noinit")));

uint32_t btnEventStartMillis = 0;   // button on start time in millis
uint32_t btnFunctionTimer = 0;      // used to time button functions
bool btnAdjustMode = false;         // whether adjustment mode is active

// Message delays are effectively a minimum, and are approximate due to sleep.
// In practice, if meter LED pulses frequently enough to trigger PR interrupt
// every few seconds then variances will be slight.

// Seconds delay between sending out instruction request messages - keep
// reasonably short (e.g. 60s) as act as useful heartbeat also used to trigger
// general check (beforehand) for any new unsolicited messages.
static const uint16_t GINR_MSG_INTERVAL_SEC = 120;

// seconds delay between sending out clock sync ping request messages. Also
// fires at startup.
static const uint16_t PING_MSG_INTERVAL_SEC = 900;

// global temporary variables, used somewhat arbitrarily vs local static vars
char tmpStr[40] = "";
uint32_t tmpInt = 0ul;

// buffer for serial input/output, serial buffer position pointer
static const uint8_t SERIAL_IN_BUFFER_SIZE = 40;
char serInBuff[SERIAL_IN_BUFFER_SIZE] = "";
uint8_t serialBuffPos = 0;

// *****************************************************************************
//    General Init - Config Vars
// *****************************************************************************

// Config Variables (corresponding defaults are defined above with DEF_*).  Can
// be changed with serial commands and are saved to EEPROM.
LogLev cfgLogLevel = DEF_LOG_LEVEL;
uint8_t cfgPuckLEDRate = 0;
uint16_t cfgPuckLEDTime = 0;
int8_t cfgTXPower = 0;
int8_t cfgTgtRSSI = 0;
uint8_t cfgNodeId = 0;
uint8_t cfgGatewayId = 0;
uint8_t cfgNetworkId1 = 0;
uint8_t cfgNetworkId2 = 0;
uint8_t cfgNetworkId3 = 0;
uint8_t cfgNetworkId4 = 0;
uint8_t cfgEncryptKey[KEY_LENGTH] =
        {'\0','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0',
        '\0','\0','\0'};
char cfgMeterUnit[KEY_LENGTH] = {""};  // one less with null-terminator
uint8_t cfgMeterInterval = 0;

// *****************************************************************************
//    General Init - Logging
// *****************************************************************************

// Serial event/status logging output prefixes
static const char LOG_ERROR_LBL[] PROGMEM = "ERROR";
static const char LOG_WARN_LBL[] PROGMEM = "WARN";
static const char LOG_INFO_LBL[] PROGMEM = "INFO";
static const char LOG_DEBUG_LBL[] PROGMEM = "DEBUG";

// *****************************************************************************
//    General Init - Commands (Serial)
// *****************************************************************************

// Abbreviated vs earlier revisions to save progmem, reference online

// Serial command (RX) strings.

// help
static const char SER_CMD_HELP[] PROGMEM = "HELP";

// sleep toggle
static const char SER_CMD_SLEEP[] PROGMEM = "Z";

// dump meter state to console
static const char SER_CMD_DUMP[] PROGMEM = "DUMP";

// reset EEPROM config to defaults
static const char SER_CMD_RCFG[] PROGMEM = "RCFG";

// print/set time (set with TIME=[time as epoch])
static const char SER_CMD_TIME[] PROGMEM = "TIME";

// print/set loglevel (set with LOGL=[log level])
static const char SER_CMD_LOGL[] PROGMEM = "LOGL";

// print/set encryption key (set with EKEY=[key])
static const char SER_CMD_EKEY[] PROGMEM = "EKEY";

// print/set network id (set with NETI=[network id])
static const char SER_CMD_NETI[] PROGMEM = "NETI";

// print/set gateway id (set with GWID=[gateway id])
static const char SER_CMD_GWID[] PROGMEM = "GWID";

// print/set node id (set with NOID=[node id])
static const char SER_CMD_NOID[] PROGMEM = "NOID";

// print/set target rssi for auto-tuning (set with TRSS=[target RSSI -100 to 0
// (0=disable)])
static const char SER_CMD_TRSS[] PROGMEM = "TRSS";

// print/set transmission power (set with TXPW=[tx power in dBi])
static const char SER_CMD_TXPW[] PROGMEM = "TXPW";

// print/set meter value (set with MTRV=[meter value])
static const char SER_CMD_MTRV[] PROGMEM = "MTRV";

// print/set meter unit (set with MTRU=[meter units])
static const char SER_CMD_MTRU[] PROGMEM = "MTRU";

// print/set meter interval (set with MTRI=[meter interval in sec])
static const char SER_CMD_MTRI[] PROGMEM = "MTRI";

// toggle test mode
static const char SER_CMD_TEST[] PROGMEM = "TEST";

// print/set puck LED rate (set with LEDR=[rate per meter LED])
static const char SER_CMD_LEDR[] PROGMEM = "LEDR";

// print/set puck LED time (set with LEDT=[time in millis])
static const char SER_CMD_LEDT[] PROGMEM = "LEDT";

// Array of commands, used to print list on help or invalid input
const char* const SER_CMDS[] PROGMEM = {SER_CMD_HELP, SER_CMD_SLEEP,
        SER_CMD_DUMP, SER_CMD_RCFG, SER_CMD_TIME, SER_CMD_LOGL, SER_CMD_EKEY,
        SER_CMD_NETI, SER_CMD_GWID, SER_CMD_NOID, SER_CMD_TRSS, SER_CMD_TXPW,
        SER_CMD_MTRV, SER_CMD_MTRU, SER_CMD_MTRI, SER_CMD_TEST, SER_CMD_LEDR,
        SER_CMD_LEDT};

// *****************************************************************************
//    General Init - Radio Message Types
// *****************************************************************************

// Meter Rebase
static const char RMSG_MREBASE[] PROGMEM = "MREB";

// Meter Update with current
static const char RMSG_MUPC[] PROGMEM = "MUPC";

// Meter Update without current
static const char RMSG_MUP_[] PROGMEM = "MUP_";

// Instruction Request to Gateway
static const char RMSG_GINR[] PROGMEM = "GINR";

// Clock Sync Ping Request to Gateway
static const char RMSG_PREQ[] PROGMEM = "PREQ";

// Clock Sync Ping Response from Gateway
static const char RMSG_PRSP[] PROGMEM = "PRSP";

// Meter instruction (from gateway) to set meter value
static const char RMSG_MVAI[] PROGMEM = "MVAI";

// Meter instruction (from gateway) to set meter interval
static const char RMSG_MINI[] PROGMEM = "MINI";

// Meter instruction (from gateway) to set puck LED
static const char RMSG_MPLI[] PROGMEM = "MPLI";

// 'no op' Meter instruction (from gateway)
static const char RMSG_MNOI[] PROGMEM = "MNOI";

// General purpose message (can broadcast)
static const char RMSG_GMSG[] PROGMEM = "GMSG";

// *****************************************************************************
//    Battery and Voltage Divider
//
//    Voltage divider is [VBatt <> R1 <> VIN_DIV_PIN <> R2 + 0.1uF cap <> GND]
//    Doesn't matter if R1 > R2 or vice versa.  Needs to be sized to scale
//    down voltage on pin to < 3.3V; e.g. for 5V VBatt VIN_DIV_RSCALE_MULT
//    should be >= 2. Will only work with regulator in place as
//    needed for accurate voltage measurement/scaling from 0 to 1023 at pin.
//
// *****************************************************************************

// Analogue reading to voltage divisor constant (1024/3.3V),
// result gives voltage reading at pin:
static const float VIN_DIV_VOLT_DIVISOR = 310.3f;

// Multiplier to scale voltage at pin to actual voltage upstream of voltage
// divider. Formula is (1/(R2/(R1+R2))) => 3.7 with R1=2.7M, R2=1M, 4 with
// R1=3M, R2=1M
static const float VIN_DIV_RSCALE_MULT = 3.7f;

// correction factor multipler, set by comparing with voltmeter.  Then * 1000 to
// get millivolts.
uint16_t VIN_DIV_CORRECT_MULT = 1000;

// last battery (or VIN) voltage reading in millivolts (to give some precision
// without a float)
uint16_t vinVoltageMV = 0;


// *****************************************************************************
//    CT Clamp
//
//    Uses YHDC SCT-013-000 CT, 100A maximum current
//    Using EMonLib for RMS calc etc (for now)
//
//    22 Ohm burden resistor used to give voltage proportionate to CT clamp's
//    secondary coil voltage.  Can measure up to 97A (23.3kW @ 240V).
//
//    Pmax=(240 * 3.3 * 2000) / (2 *sqrt(2) * 22) = 23.3kW
//
//    Expect 0.935 Vrms, 2.612 V peak-peak
//
//    Voltage divider is 2 * 470K
//
//    WARNING: do not unplug CT clamp from node while it's connected to a 'live
//    wire'. Apparently diode in clamp offers some protection but don't risk it.
//
// *****************************************************************************
EnergyMonitor emon1;
static const float CT_IRMS_CORRECT_MULT = 111.1;
static const float CT_AC_VOLTAGE = 230.0;
static const double CT_IRMS_HIGH_PASS = 0.2;
static const int16_t CT_NUM_SAMPLES = 1480;   // proportionate to AC cycle, freq

// *****************************************************************************
//    Radio Init
//
//    All message-driven request/response interactions are logically
//    asynchronous; but implemented by RadioHead via synchronous send/ACK
//    message pairs. Messaging initiated by client node to facilitate long sleep
//    periods.  As such, the gateway needs to queue any commands or updates for
//    meter nodes.  Max message data length is 60 *bytes*, with no
//    recognition of payload elements as anything other than a big 'string'.
//    E.g. an uint32_t of 2,000,000,000 will be 80b/10B not 32b/4B.
//
// *****************************************************************************

static const float RADIO_FREQ = 915.0f;

// Modem config per RadioHead docs.  Note that lower rates (strangely) are
// unreliable with latest board.  FSK seems most reliable.
//
// From FSK_Rb4_8Fd9_6 through to FSK_Rb125Fd125 work well (could go higher).
// Use fastest rate that yields acceptable range and reasonably low TX power
// (unless running on DC adapter and not concerned with RF 'noise').

//Using FSK, Whitening, bit rate = 125kbps, modulation frequency = 125kHz.
static const RH_RF69::ModemConfigChoice MODEM_CONFIG = RH_RF69::FSK_Rb125Fd125;

// Transmit and Receive timeouts (millis).  Long timeouts can make serial
// communication etc laggy if gateway not up.
static const uint16_t TX_TIMEOUT = 500;
static const uint16_t RX_TIMEOUT = 800;

// Radio Driver and Message Manager
RH_RF69 radio(RADIO_SS_PIN, RADIO_INTERRUPT_PIN);
RHReliableDatagram msgManager(radio, cfgNodeId);

// Last message RSSI (received signal strength indicator).  0 is highest, -100
// is lowest.
int8_t lastRSSIAtNode = 0;
int8_t lastRSSIAtGateway = 0;
// average RSSI as advised by gateway since TX power last set
int8_t averageRSSIAtGateway = 0;

// RSSI target tolerance range, outside this will cause TX Power to be
// adjusted
static const uint8_t TX_POW_RSSI_TOLERANCE = 10;

// minimum interval to check RSSI, auto-tune TXPow
static const uint16_t TX_POW_CHECK_INTERVAL_SEC = 60;

// number of consecutive TX failures before TXPow incremented
static const uint8_t TX_POW_MAX_FAILS_TO_ADJ = 3;

// if txConsFailCount >= TX_POW_MAX_FAILS_TO_ADJ will increase TX power
int8_t txConsFailCount = 0;

// radio/node Id of last message sender
uint8_t lastMsgFrom = 0;

// Intermediate string buffer for message contents.  Used as easier to work with
// than byte buffer.  KEY_LENGTH 'fudge factor' added as while length is fixed,
// an accidental overflow can be caught and handled through validation instead
// of creating an actual overflow.
char msgBuffStr[RH_RF69_MAX_MESSAGE_LEN + KEY_LENGTH] = "";

// Actual buffer used to pass payload to/from the RadioHead library.
uint8_t radioMsgBuff[RH_RF69_MAX_MESSAGE_LEN] = "";


// *****************************************************************************
//    Metering Init
//
// *****************************************************************************

// Test settings.  Toggled via TESTMODE serial commmand, simulates meter LED
// pulses at TEST_METER_PULSE_INTERVAL (minimum delay between pulses) with
// TEST_METER_PULSE_TIME setting the duration of the pulse itself (pulse width).
bool testMode = false;
static const uint16_t TEST_METER_PULSE_INTERVAL = 1000;     // millis
static const uint16_t TEST_METER_PULSE_TIME = 10;           // millis
bool testPulseOn = false;
uint32_t whenTestPulseChange = 0ul;                         //millis

// Maximum meter value => 4 billion entries (which may be greater than number of
// pulses depending on interval setting).  UL type will overflow at ~4.3
// billion. Could use long long but constrained to limit message size.
static const uint32_t METER_MAX_VALUE = 4000000000ul;

// max of 255 with unsigned short 'pointers'; little benefit in making
// longer than ~10 as is redundant on message send
static const uint8_t METER_READ_BUFFER_LEN = 10;

typedef enum {
    entryNull = 0,
    entryOpen = 1,          // interval open, accepts new reads
    entryClosed = 2,        // closed, interval expired, ready to send
    entrySent = 3           // entry sent
} MeterEntryState;

// Meter buffer is a ring buffer (circular queue) to hold last 'n' read entries
// with accumulation meter value and time, with oldest element being overwritten
// on overflow.
// Time is since UNIX epoch, resolution is seconds; as millis is
// challenged by RTC resolution, message sizes, and effects of sleeping on
// timers.  Could tack millis on but little value.

struct MeterRead {
   uint32_t readTime = 0;
   uint32_t meterValue = 0;     // accumulated meter value
   double rmsCurrent = 0.00;    // current at interval close
   MeterEntryState entryState = entryNull;
};

// Ring buffer and associated vars are not volatile as accessed asynchronously
// to interrupt processing (which updates newRawMeterReads)
struct MeterRead meterReadBuffer[METER_READ_BUFFER_LEN];

// Number of new meter 'closed' entries, ready to be 'popped' off the buffer
uint16_t newClosedMeterEntries = 0;

// Number of new raw meter readings, ready to be 'pushed' to the buffer
volatile uint32_t newRawMeterReads = 0;

// global var for temporary MeterRead object
struct MeterRead tmpRead;

// Global var.  If true, will force a 'rebase' message to be sent ASAP;
// indicating that time or meter count have been reset.  Recipient should not
// rely on this, at least alerting on big jumps forward, or reversions in time
// or meter values.
bool forceRebase = false;

// Meter Read Buffer 'read pointer' - next index to read from buffer (oldest
// read) despite reference to 'pop' elements are left as-is until they are
// overwritten
uint8_t mrbPopNext = 0;
// write pointer - next index to write to
uint8_t mrbPushNext = 0;

// Messages are sent when the number of elements a message can include is
// reached, or a maximum interval expires (whichever is first).  This can be
// changed to only send the most recent n reads at the interval specified - for
// higher-frequency meters.

// max time/read elements per 60B MUP message - forced to 2 if CT_ENABLED
static const uint8_t METER_MSG_MAX_ELEMENTS = 4;

// Max seconds delay between sending out meter messages.  Favour meter interval
// duration to control msg rate, use this as a fallback for low frequency
// metering.
static const uint8_t MAX_METER_MSG_INTERVAL_SEC = 180;

// whether to only send latest METER_MSG_MAX_ELEMENTS on
// MAX_METER_MSG_INTERVAL_SEC in a single message, ignoring older reads.
static const bool SEND_METER_TIMER_ONLY = false;

// The number of 'watched meter' LED flash/pulses per 'puck' LED flash in
// response.  E.g. could flash once every 10 to consume battery.  0 means LED is
// disabled (except to indicate boot).
static const uint8_t DEF_METER_PULSES_PER_LED_PULSE = 0;

volatile uint32_t whenPuckLEDOn = 0ul;  //millis

// runtime var to implement pulse rate
volatile uint8_t ledPulseCountdown = DEF_METER_PULSES_PER_LED_PULSE;

// The max duration of each LED pulse emitted by the puck in millis. Setting to
// 0 will cause to flash in unison with meter.  Maximum of 3000, but will always
// go low/off on falling edge of meter's pulse.
static const uint16_t DEF_LED_PULSE_TIME_MS = 100;

// *****************************************************************************
//    RTC & Timers
//
// *****************************************************************************

// Create an rtc object.  If power has been lost will be 1 Jan 1970 00:00:00.
PCF2123 rtc(RTC_SS_PIN);

// Max acceptable latency of ping req/resp interaction with Gateway to set clock
static const uint8_t SYNC_TIME_MAX_DRIFT_SEC = 3;

// Default UNIX epoch time in case RTC has an invalid timestamp and needs to be
// re-initialised.
static const int32_t INIT_TIME = 1483228800ul;        //  1 Jan 2017 00:00:00

// Global timestamp vars.  Timestamps are UNIX epoch in seconds (since 1 Jan
// 1970 00:00:00).
uint32_t whenBooted = 0ul;
uint32_t whenLastGInstMsg = 0ul;
uint32_t whenLastPingMsg = 0ul;
uint32_t whenLastMeterMsg = 0ul;
uint32_t whenLastTXPowCheck = 0ul;
uint32_t secondsSlept = 0ul;

// *****************************************************************************
//    Runtime Logging
//
// *****************************************************************************

// used to detect when a new line is started, to write log level
bool newLogLine = true;


void printNewLine(){
    Serial.write("\r\n");
    newLogLine = true;
}


void print_P(const char* flashStr){
    strcpy_P(tmpStr, flashStr);
    Serial.write(tmpStr);
}


void println_P(const char* flashStr){
    print_P(flashStr);
    printNewLine();
}


void printLogLevel(LogLev logLevel, bool printColon){
    switch (logLevel) {
        case logNull:
            return;
        case logError:
            print_P(LOG_ERROR_LBL);
            break;
        case logWarn:
            print_P(LOG_WARN_LBL);
            break;
        case logInfo:
            print_P(LOG_INFO_LBL);
            break;
        case logDebug:
            print_P(LOG_DEBUG_LBL);
            break;
    }
    if (printColon)
        Serial.write(": ");
}


void writeLog(char* debugText, LogLev logLevel){
    if (cfgLogLevel >= logLevel){
        if (newLogLine)
            printLogLevel(logLevel, true);
        Serial.write(debugText);
        newLogLine = false;
    }
}


void writeLogLn(char* debugText, LogLev logLevel){
    if (cfgLogLevel >= logLevel){
        if (newLogLine)
            printLogLevel(logLevel, true);
        Serial.write(debugText);
        printNewLine();
    }
}


void writeLog(const char* debugText, LogLev logLevel){
    writeLog((char*)debugText, logLevel);
}


void writeLogLn(const char* debugText, LogLev logLevel){
    writeLog(debugText, logLevel);
    printNewLine();
}


void writeLog(uint32_t debugText, LogLev logLevel){
    static char printStr[12] = "";
    sprintf(printStr, "%lu", debugText);
    writeLog(printStr, logLevel);
}


void writeLogLn(uint32_t debugText, LogLev logLevel){
    writeLog(debugText, logLevel);
    printNewLine();
}


void writeLog(uint16_t debugText, LogLev logLevel){
    writeLog((uint32_t)debugText, logLevel);
}


void writeLogLn(uint16_t debugText, LogLev logLevel){
    writeLog((uint32_t)debugText, logLevel);
    printNewLine();
}


void writeLog(int32_t debugText, LogLev logLevel){
    static char printStr[12] = "";
    sprintf(printStr, "%ld", debugText);
    writeLog(printStr, logLevel);
}


void writeLogLn(int32_t debugText, LogLev logLevel){
    writeLog(debugText, logLevel);
    printNewLine();
}


void writeLog(int16_t debugText, LogLev logLevel){
    writeLog((int32_t)debugText, logLevel);
}


void writeLogLn(int16_t debugText, LogLev logLevel){
    writeLog((int32_t)debugText, logLevel);
    printNewLine();
}


void writeLog(double debugText, LogLev logLevel){
    static char printStr[12] = "";
    // use int hack as dtostr takes up too much memory, sprintf floats not
    // supported...
    sprintf(printStr, "%d.%02d", (int)debugText, (int)(debugText*100)%100);
    writeLog(printStr, logLevel);
}


void writeLogLn(double debugText, LogLev logLevel){
    writeLog(debugText, logLevel);
    printNewLine();
}


void writeLogF(const __FlashStringHelper* debugText, LogLev logLevel){
    // easiest to do redundant serial.print as can't simply cast FlashString
    if (cfgLogLevel >= logLevel){
        if (newLogLine)
            printLogLevel(logLevel, true);
        Serial.print(debugText);
        newLogLine = false;
    }
}


void writeLogLnF(const __FlashStringHelper* debugText, LogLev logLevel){
    if (cfgLogLevel >= logLevel){
        if (newLogLine)
            printLogLevel(logLevel, true);
        Serial.print(debugText);
    }
    printNewLine();
}


// *****************************************************************************
//
//    Functions & Main Loop...
//
// *****************************************************************************

uint32_t getNowTimestampSec();
void resetConfig();
void sendRadioMsg(uint8_t recipient, bool checkReply);


void print2Digits(int digits){
    if(digits < 10)
        writeLogF(F("0"), logNull);
    writeLog(digits, logNull);
}


void printPrompt(){
    writeLogF(F(" > "), logNull);
}


void printNetworkId(){
    writeLogF(F("Net Id: "), logNull);
    writeLog(cfgNetworkId1, logNull);
    writeLogF(F("."), logNull);
    writeLog(cfgNetworkId2, logNull);
    writeLogF(F("."), logNull);
    writeLog(cfgNetworkId3, logNull);
    writeLogF(F("."), logNull);
    writeLogLn(cfgNetworkId4, logNull);
}


void printUnits(){
    writeLog(cfgMeterUnit, logNull);
}


void printCmdHelp(){
    printPrompt();
    writeLogF(F("Commands: "), logNull);
    for (uint8_t i = 0; i <= 17; i++){
        print_P((char*)pgm_read_word(&(SER_CMDS[i])));
        writeLogF(F(" "), logNull);
    }
    printNewLine();
}

bool strStartsWith(const char *strBody, const char *strPrefix){
    /*
        Tests if string starts with given prefix.  Case insensitive.
     */

    return (strncasecmp(strBody, strPrefix, strlen(strPrefix)) == 0);
}


uint8_t strStartsWithP(const char *strBody, const char *strPrefix){
    /*
        Variant of strStartsWith for PROGMEM prefix, test for presence of
        '=' (used to distinguish a setter from getter, e.g. 'TIME=').
     */

    static char withEq [] = "";
    strcpy_P(withEq, strPrefix);
    strcat_P(withEq, PSTR("="));
    if(strStartsWith(strBody, withEq))
        return 2;   // setter
    else if(strncasecmp_P(strBody, strPrefix, strlen_P(strPrefix)) == 0)
        return 1;   // getter/normal
    else
        return 0;   // no match
}


double getRMSCurrent(){
    /*
        Returns RMS Current from CT clamp in amps.
     */
    static double irmsCurrent = 0.0;
    irmsCurrent = emon1.calcIrms(CT_NUM_SAMPLES);
    if (irmsCurrent > CT_IRMS_HIGH_PASS)
        return irmsCurrent;
    else
        return 0.0;
}


double getRMSPower(double rmsCurrent){
    /*
        Returns apparent RMS Power from CT clamp in watts.
     */
    return rmsCurrent * CT_AC_VOLTAGE;
}


void printCurrent(){
    static double irmsCurrent = 0.0;
    irmsCurrent = getRMSCurrent();
    printPrompt();
    writeLogF(F("IRMS: "), logNull);
    writeLog(irmsCurrent, logNull);
    writeLogF(F(", PRMS: "), logNull);
    writeLogLn(getRMSPower(irmsCurrent), logNull);
}


uint16_t freeRAM(){
    /*
        Returns free SRAM in bytes (328P has 2kB total)
     */
    extern int __heap_start, *__brkval;
    int v;
    return (uint16_t) &v - (__brkval == 0 ? (int) &__heap_start :
            (int) __brkval);
}


void printMeterBuffer(){
    /*
        Dumps meter buffer to serial (terminal)
     */
    printPrompt();
    writeLogLnF(F("Meter: "), logNull);
    for (int i = 0; i < METER_READ_BUFFER_LEN; i++){
        printPrompt();
        writeLogF(F(" ["), logNull);
        writeLog(meterReadBuffer[i].readTime, logNull);
        writeLogF(F(","), logNull);
        writeLog(meterReadBuffer[i].meterValue, logNull);
        writeLogF(F(","), logNull);
        writeLog(meterReadBuffer[i].rmsCurrent, logNull);
        writeLogF(F(","), logNull);
        writeLog(meterReadBuffer[i].entryState, logNull);
        writeLogLnF(F("]"), logNull);
    }
    printNewLine();
}


uint8_t getMtrBuffIxOffset(uint8_t buffIndex, int8_t offset){
    /*
        Gets the meter index offset from the index given -
        allows rewind or fast-forward
     */

    if (offset < 0 && abs(offset) > buffIndex)
        // abs(offset) may be > METER_READ_BUFFER_LEN:
        return METER_READ_BUFFER_LEN - ((abs(offset) - buffIndex) %
                METER_READ_BUFFER_LEN);
    else if (offset < 0)
        return buffIndex - abs(offset);
    else
        return (buffIndex + offset) % METER_READ_BUFFER_LEN;
}


uint32_t getNewMeterValue(uint32_t lastMeterValue, uint32_t readValue,
            bool newAccumValue){
    /*
        Determines new value of accumulation meter given a new read
     */

    // Cycle meter when at METER_MAX_VALUE, or handle reset.
    // Need to handle cycling at receiver
    if (newAccumValue || (lastMeterValue + readValue >= METER_MAX_VALUE)){
        forceRebase = true;
        return readValue;
    }
    else
        return (lastMeterValue + readValue);
}


void checkStaleEntry(){
    /*
        Checks for an open meter read entry that needs closing (e.g. interval
        expired), handles finalising of new read entry if created from read or
        rebase.
     */

    // set previous entry to close if over-time/stale
    if (cfgMeterInterval > 0 &&
        getNowTimestampSec() - meterReadBuffer[mrbPushNext].readTime
            >= cfgMeterInterval
        && meterReadBuffer[mrbPushNext].entryState == entryOpen)
            meterReadBuffer[mrbPushNext].entryState = entryClosed;

    // process close (set in several places, not just above), create a new
    // 'open' entry
    if (meterReadBuffer[mrbPushNext].entryState == entryClosed){
        // Finalise 'closed', ready to send meter entry
        if (CT_ENABLED)
            meterReadBuffer[mrbPushNext].rmsCurrent = getRMSCurrent();

        newClosedMeterEntries++;

        // advance write pointer
        mrbPushNext = getMtrBuffIxOffset(mrbPushNext, 1);
        // set current to 0
        meterReadBuffer[mrbPushNext].rmsCurrent = 0.00;

        // Nudge next read pointer forward on a write pointer collision
        if (mrbPopNext == mrbPushNext)
            mrbPopNext = getMtrBuffIxOffset(mrbPopNext, 1);
    }
}


void pushToMeterBuffer(uint32_t readTimestampSec, uint32_t readValue,
        bool newAccumValue){
    /*
        Writes next element in ring buffer, incrementing push/write pointer.
     */

    // If a non-zero meter interval is set, check if last read inside meter
    // interval resolution, and if so append to entry; otherwise 'close' it.
    if (cfgMeterInterval > 0 && ! newAccumValue){
        if (readTimestampSec - meterReadBuffer[mrbPushNext].readTime
                < cfgMeterInterval){
            // entry time stays as when interval began
            meterReadBuffer[mrbPushNext].entryState = entryOpen;
            meterReadBuffer[mrbPushNext].meterValue =
                    getNewMeterValue(meterReadBuffer[mrbPushNext].meterValue,
                        readValue, newAccumValue);
            return;
        }
        else {
            checkStaleEntry();  //close old entry and creates new
            meterReadBuffer[mrbPushNext].entryState = entryOpen;
            meterReadBuffer[mrbPushNext].readTime = readTimestampSec;
            meterReadBuffer[mrbPushNext].meterValue =
                    getNewMeterValue(meterReadBuffer[
                        getMtrBuffIxOffset(mrbPushNext, -1)].meterValue,
                        readValue, newAccumValue);
        }
    }
    // if a zero meter interval is set, or is rebased, create a new record and
    // move pointer forward
    else {
        meterReadBuffer[mrbPushNext].entryState = entryClosed;
        meterReadBuffer[mrbPushNext].readTime = readTimestampSec;
        meterReadBuffer[mrbPushNext].meterValue =
                    getNewMeterValue(meterReadBuffer[
                        getMtrBuffIxOffset(mrbPushNext, -1)].meterValue,
                        readValue, newAccumValue);
        checkStaleEntry();      // closes entry and creates new
    }
}


bool popFromMeterBuffer(MeterRead& meterread, bool getLast){
    /*
        If getLast is true, returns last 'closed' element in ring buffer,
        advancing pop/read pointer to 'skip' any unpopped reads.

        Otherwise returns next 'closed' element in ring buffer and increments
        read pointer.  Returns from oldest to newest, stopping after most recent
        'closed' (ready to send) read.

        Caller can repeat pop until false returned.
     */

    checkStaleEntry();

    if (newClosedMeterEntries == 0)
        return false;

    static MeterEntryState entrySearchState;
    entrySearchState = entryNull;
    static uint8_t entryIx;

    if (getLast){
        // search backwards for latest 'closed' entry, failing if we hit
        // sent first
        entryIx = mrbPushNext;
        while (entrySearchState != entryClosed
                && entrySearchState != entrySent){
            entryIx = getMtrBuffIxOffset(entryIx, -1);
            entrySearchState = meterReadBuffer[entryIx].entryState;
        }
    }
    else {
        // get next entry to be 'popped'
        entryIx = mrbPopNext;
        entrySearchState = meterReadBuffer[entryIx].entryState;
    }

    meterread.readTime = meterReadBuffer[entryIx].readTime;
    meterread.meterValue = meterReadBuffer[entryIx].meterValue;
    meterread.entryState = meterReadBuffer[entryIx].entryState;
    meterread.rmsCurrent = meterReadBuffer[entryIx].rmsCurrent;

    if (entrySearchState == entryClosed){
        // set to sent
        meterReadBuffer[entryIx].entryState = entrySent;
        // advance 'pop next' pointer
        mrbPopNext = getMtrBuffIxOffset(entryIx, 1);
        // We have returned a new 'closed', ready to send meter read
        if (getLast)
            newClosedMeterEntries = 0;
        else
            newClosedMeterEntries--;
        return true;
    }
    else
        return false;
}


void checkPuckLEDState(){
    /*
        Turn Puck LED off if needed.
     */
    if (cfgPuckLEDTime > 0 && whenPuckLEDOn > 0 &&
            (millis() - whenPuckLEDOn >= cfgPuckLEDTime)){
        PORTD = PORTD & B11101111;      // LED off
        whenPuckLEDOn = 0ul;
        ledPulseCountdown = 0;
    }
}


void watchedLEDOn(){
    /*
        Record meter read and turn puck LED on.
        INTERRUPT CODE.  BE CAREFUL to keep lean, e.g. avoid serial prints

        Use of comparator and schmitt trigger makes debouncing unnecessary.
    */
    newRawMeterReads++;

    // do reset here in case 'stuck' processing high frequency pulses - unlikely
    // with electricity meter (e.g. >=10Hz); belt & braces
    wdt_reset();

    // Check if puck LED is enabled.  For cfgPuckLEDRate,
    // 1 == 1:1 rate vs watched meter, 2 == 1:2, 0 = disabled.
    if (cfgPuckLEDRate >= 1){
        // If due to go on, set on, set time LED went on
        if (cfgPuckLEDTime == 0 || (cfgPuckLEDTime > 0 &&
                (cfgPuckLEDRate == 1 ||
                (cfgPuckLEDRate > 1 && ledPulseCountdown <= 1))
                )){
            PORTD = PORTD | B00010000;  // on
            whenPuckLEDOn = millis();
        }
        // if rate is > 1:1, update or reset countdown
        if (cfgPuckLEDRate > 1 && ledPulseCountdown > 1)
            ledPulseCountdown--;
        else if (cfgPuckLEDRate > 1 && ledPulseCountdown <= 1)
            ledPulseCountdown = cfgPuckLEDRate;
    }
}


void watchedLEDChanged(){
    /*
        Dispatches rising or falling edges of LED pulses.  Used to support
        turning Puck LED on/off.  Not used if LED not set to blink.

        INTERRUPT CODE.  BE CAREFUL to keep lean, e.g. avoid serial prints
    */

    static uint8_t prPinState = 0;
    prPinState = PIND & B00001000;      // Digital pin 3

    if (prPinState == B00001000)
    // is rising so watched LED on
        watchedLEDOn();
    else
        PORTD = PORTD & B11101111;
}


void printTime(tmElements_t timeTME, LogLev logLevel) {
    /*
       Print formatted timestamp to serial out if runtime log level is >=
       logLevel
    */
    if (cfgLogLevel < logLevel)
        return;

    print2Digits(1970 + timeTME.Year);
    writeLogF(F("-"), logNull);
    print2Digits(timeTME.Month);
    writeLogF(F("-"), logNull);
    print2Digits(timeTME.Day);
    writeLogF(F(" "), logNull);
    print2Digits(timeTME.Hour);
    writeLogF(F(":"), logNull);
    print2Digits(timeTME.Minute);
    writeLogF(F(":"), logNull);
    print2Digits(timeTME.Second);
}


void getRTCTime(tmElements_t *nowTimeT){
    /*
       Gets time from RTC, enabling and disabling RTC's SPI CS
    */
    digitalWrite(RTC_SS_PIN, LOW);
    rtc.time_get(nowTimeT);
    digitalWrite(RTC_SS_PIN, HIGH);
}


void printNowTime(LogLev logLevel) {
    static tmElements_t nowTimeT;
    getRTCTime(&nowTimeT);
    printTime(nowTimeT, logLevel);
}


uint32_t getNowTimestampSec(){
    static tmElements_t nowTimeTS;
    getRTCTime(&nowTimeTS);
    return makeTime(nowTimeTS);
}


void adjustTSVar(uint32_t * timestampVar, uint32_t timeSecs){
    /*
       Adjust global timestamps to avoid negative/wild results from duration
       calcs
    */
    static uint64_t adjTime = 0ul;
    adjTime = timeSecs - (getNowTimestampSec() - (*timestampVar));
    *timestampVar = adjTime >= 0 ? (uint32_t)adjTime : 0ul;
}


void setNowTimestampSec(uint32_t timeSecs){
    /*
       Sets UTC time in seconds since UNIX Epoch @ midnight Jan 1 1970
    */

     // Adjust global timestamps
     adjustTSVar(&whenBooted, timeSecs);
     adjustTSVar(&whenLastGInstMsg, timeSecs);
     adjustTSVar(&whenLastMeterMsg, timeSecs);
     adjustTSVar(&whenLastPingMsg, timeSecs);
     adjustTSVar(&whenLastTXPowCheck, timeSecs);

     // Convert Unix timestamp to tmElements_t and set RTC
     tmElements_t newTime;
     breakTime(timeSecs, newTime);
     rtc.time_set(&newTime);

     // push out 0 value read to update time, force rebase
     writeLogF(F("Time: "), logDebug);
     printNowTime(logDebug);
     printNewLine();
     pushToMeterBuffer(getNowTimestampSec(),
        meterReadBuffer[getMtrBuffIxOffset(mrbPushNext, -1)].meterValue, true);
     forceRebase = true;
}


int8_t getTXPowMin(){
    if (RADIO_HIGH_POWER)
        return -2;
    else
        return -18;
}


int8_t getTXPowMax(){
    if (RADIO_HIGH_POWER)
        return 20;
    else
        return 13;
}


bool isTXPowValid(int8_t TXPowValue){
    return (TXPowValue >= getTXPowMin() && TXPowValue <= getTXPowMax());
}


void putConfigToMem(){
    /*
       Uses EEPROM Put function to write current config to EEPROM.  Only writes
       to chip if value differs (i.e. update).
    */

    uint8_t eeAddress = 0;
    writeLogLnF(F("Updating EEPROM"), logInfo);
    wdt_reset();
    EEPROM.put(eeAddress, (uint8_t)cfgLogLevel);
    eeAddress += sizeof((uint8_t)cfgLogLevel);
    EEPROM.put(eeAddress, cfgPuckLEDRate);
    eeAddress += sizeof(cfgPuckLEDRate);
    EEPROM.put(eeAddress, cfgPuckLEDTime);
    eeAddress += sizeof(cfgPuckLEDTime);
    EEPROM.put(eeAddress, cfgMeterInterval);
    eeAddress += sizeof(cfgMeterInterval);
    for (int i = 0; i < KEY_LENGTH; i++){
        EEPROM.put(eeAddress, cfgMeterUnit[i]);
        eeAddress++;
    }
    EEPROM.put(eeAddress, cfgTXPower);
    eeAddress += sizeof(cfgTXPower);
    wdt_reset();
    EEPROM.put(eeAddress, cfgTgtRSSI);
    eeAddress += sizeof(cfgTgtRSSI);
    EEPROM.put(eeAddress, cfgNodeId);
    eeAddress += sizeof(cfgNodeId);
    EEPROM.put(eeAddress, cfgGatewayId);
    eeAddress += sizeof(cfgGatewayId);
    EEPROM.put(eeAddress, cfgNetworkId1);
    eeAddress += sizeof(cfgNetworkId1);
    EEPROM.put(eeAddress, cfgNetworkId2);
    eeAddress += sizeof(cfgNetworkId2);
    EEPROM.put(eeAddress, cfgNetworkId3);
    eeAddress += sizeof(cfgNetworkId3);
    EEPROM.put(eeAddress, cfgNetworkId4);
    eeAddress += sizeof(cfgNetworkId4);
    for (uint8_t i = 0; i < KEY_LENGTH; i++){
        EEPROM.put(eeAddress, cfgEncryptKey[i]);
        eeAddress++;
    }
}


void updatePuckLEDRate(uint8_t newRate, bool persist){
    /*
       Updates cfgPuckLEDRate and sets appropriate interrupt
    */
    cfgPuckLEDRate = newRate;
    ledPulseCountdown = 0;      // reset countdown

    PORTD = PORTD & B11101111;      // ensure LED off

    // use change (up/down) if puck LED set to flash, otherwise just 'falling'
    // (to record LED pulse end, falling edge)
    detachInterrupt(digitalPinToInterrupt(PR_INTERRUPT_PIN));
    if (cfgPuckLEDRate > 0)
        attachInterrupt(digitalPinToInterrupt(PR_INTERRUPT_PIN),
                watchedLEDChanged, CHANGE);
    else
        attachInterrupt(digitalPinToInterrupt(PR_INTERRUPT_PIN),
                watchedLEDOn, FALLING);

    writeLogF(F("LED: 1 per meter's "), logNull);
    writeLogLn(cfgPuckLEDRate, logNull);

    if (persist)
        putConfigToMem();  // apply config and write to EEPROM
}


void getConfigFromMem(){
    /*
       Reads from EEPROM, testing each value's validity.  Any failure will
       result in EEPROM being re-written.
    */

    uint8_t eeAddress = 0;
    uint8_t byteVal = 0;
    int8_t intVal = 0;
    uint16_t doubleVal = 0;
    uint8_t byteValArray[KEY_LENGTH] = {0};
    bool EEPROMValid = true;

    writeLogLnF(F("Reading EEPROM"), logInfo);
    wdt_reset();
    EEPROM.get(eeAddress, byteVal);
    if (byteVal >= logNull && byteVal <= logDebug)
        cfgLogLevel = (LogLev)byteVal;
    else
        EEPROMValid = false;
    eeAddress++;

    EEPROM.get(eeAddress, byteVal);
    if (byteVal >= 0 && byteVal < 255)
        updatePuckLEDRate(byteVal,false);
    else
        EEPROMValid = false;
    eeAddress++;

    EEPROM.get(eeAddress, doubleVal);
    if (doubleVal <= 3000)
        cfgPuckLEDTime = doubleVal;
    else
        EEPROMValid = false;
    eeAddress++;
    eeAddress++;

    EEPROM.get(eeAddress, byteVal);
    if (byteVal >= 0 && byteVal < 255)
        cfgMeterInterval = byteVal;
    else
        EEPROMValid = false;
    eeAddress++;

    memset(byteValArray, '\0', sizeof(byteValArray));
    for (int i = 0; i < KEY_LENGTH; i++){
        EEPROM.get(eeAddress, byteValArray[i]);
        if (byteValArray[i] != '\0' && (byteValArray[i] < 32 ||
                byteValArray[i] > 126))
            EEPROMValid = false;
        eeAddress++;
    }

    if (EEPROMValid){
        memset(cfgMeterUnit, '\0', sizeof(cfgMeterUnit));
        strncpy(cfgMeterUnit, (char*)byteValArray, sizeof(byteValArray));
    }

    EEPROM.get(eeAddress, intVal);
    if (isTXPowValid(intVal))
        cfgTXPower = intVal;
    else
        EEPROMValid = false;
    eeAddress++;

    EEPROM.get(eeAddress, intVal);
    if (intVal >= -100 && intVal <= 0)
        cfgTgtRSSI = intVal;
    else
        EEPROMValid = false;
    eeAddress++;

    wdt_reset();

    EEPROM.get(eeAddress, byteVal);
    if (byteVal > 0 && byteVal < 255)
        cfgNodeId = byteVal;
    else
        EEPROMValid = false;
    eeAddress++;

    EEPROM.get(eeAddress, byteVal);
    if (byteVal > 0 && byteVal < 255)
        cfgGatewayId = byteVal;
    else
        EEPROMValid = false;
    eeAddress++;

    EEPROM.get(eeAddress, byteVal);
    if (byteVal >= 0 && byteVal < 255)
        cfgNetworkId1 = byteVal;
    else
        EEPROMValid = false;
    eeAddress++;

    EEPROM.get(eeAddress, byteVal);
    if (byteVal >= 0 && byteVal < 255)
        cfgNetworkId2 = byteVal;
    else
        EEPROMValid = false;
    eeAddress++;

    EEPROM.get(eeAddress, byteVal);
    if (byteVal > 0 && byteVal < 255)
        cfgNetworkId3 = byteVal;
    else
        EEPROMValid = false;
    eeAddress++;

    EEPROM.get(eeAddress, byteVal);
    if (byteVal > 0 && byteVal < 255)
        cfgNetworkId4 = byteVal;
    else
        EEPROMValid = false;
    eeAddress++;

    memset(byteValArray, '\0', sizeof(byteValArray));
    for (int i = 0; i < KEY_LENGTH; i++){
        EEPROM.get(eeAddress, byteValArray[i]);
        if (byteValArray[i] < 32 || byteValArray[i] > 126)
            EEPROMValid = false;
        eeAddress++;
    }

    if (EEPROMValid){
        memset(cfgEncryptKey, '\0', sizeof(cfgEncryptKey));
        memcpy(cfgEncryptKey, byteValArray, sizeof(byteValArray));
    }

    if (! EEPROMValid){
        writeLogLnF(F("EEPROM Invalid"), logError);
        resetConfig();
        putConfigToMem();
    }
}


void applyRadioConfig(){
    /*
       Applies current radio config parameters.  May be invoked from changes to
       config through serial commands.
    */

    writeLogLnF(F("Radio Init"), logDebug);

    if (!msgManager.init())         // also intialises radio driver
      writeLogLnF(F("MsgMgr failed"), logError);

    msgManager.setThisAddress(cfgNodeId);
    msgManager.setTimeout(TX_TIMEOUT);

    if (!radio.setModemConfig(MODEM_CONFIG)) {
        writeLogLnF(F("ModemCfg failed"), logError);
    }

    if (!radio.setFrequency(RADIO_FREQ)) {
        writeLogLnF(F("SetFreq failed"), logError);
    }

    radio.setTxPower(cfgTXPower, RADIO_HIGH_POWER);

    uint8_t syncwords[] =
            {cfgNetworkId1, cfgNetworkId2, cfgNetworkId3, cfgNetworkId4};
    radio.setSyncWords(syncwords, sizeof(syncwords));

    radio.setEncryptionKey(cfgEncryptKey);

    radio.sleep();
}


void resetConfig(){
    /*
       Sets config parameters to defaults
    */
    cfgLogLevel = DEF_LOG_LEVEL;
    updatePuckLEDRate(DEF_METER_PULSES_PER_LED_PULSE,false);
    cfgPuckLEDTime = DEF_LED_PULSE_TIME_MS;
    cfgMeterInterval = DEF_METER_INT_RES_SEC;
    strncpy(cfgMeterUnit, DEF_METER_UNIT, KEY_LENGTH - 1);
    cfgTXPower = DEF_TX_POWER;
    cfgTgtRSSI = DEF_TGT_RSSI;
    cfgNodeId = DEF_NODE_ID;
    cfgGatewayId = DEF_GATEWAY_ID;
    cfgNetworkId1 = DEF_NETWORK_ID_O1;
    cfgNetworkId2 = DEF_NETWORK_ID_O2;
    cfgNetworkId3 = DEF_NETWORK_ID_O3;
    cfgNetworkId4 = DEF_NETWORK_ID_O4;
    memcpy(cfgEncryptKey, DEF_ENCRYPT_KEY, KEY_LENGTH);
    putConfigToMem();
    applyRadioConfig();
}


bool changeTXPower(int8_t newValue, bool toMem){
    if (! isTXPowValid(newValue))
        return false;

    writeLogF(F("TX Power: "), logInfo);
    writeLog(newValue, logInfo);
    writeLogF(F(", Avg RSSI: "), logInfo);
    writeLogLn(averageRSSIAtGateway, logInfo);

    cfgTXPower = newValue;
    averageRSSIAtGateway = 0;
    if (toMem)
        putConfigToMem();
    radio.setTxPower(cfgTXPower, RADIO_HIGH_POWER);

    sprintf_P(msgBuffStr, RMSG_GMSG);
    sprintf(msgBuffStr, "%s,%s %d", msgBuffStr, "Set TXPow to ", cfgTXPower);
    sendRadioMsg(cfgGatewayId, false);

    radio.sleep();
    return true;
}


void checkTXtoRSSI(){
    /**
        Check if new RSSI from Gateway (reflecting signal strength measured at
        the Gateway, for this node), adjust TX power if needed
     */

    if (lastRSSIAtGateway < 0){
        averageRSSIAtGateway = (averageRSSIAtGateway == 0 ? lastRSSIAtGateway :
                (averageRSSIAtGateway + lastRSSIAtGateway) / 2);
        lastRSSIAtGateway = 0;
    }

    if ((cfgTgtRSSI < 0) && averageRSSIAtGateway < 0 &&
            (whenLastTXPowCheck == 0 ||
                    getNowTimestampSec() - whenLastTXPowCheck >=
                    TX_POW_CHECK_INTERVAL_SEC)){
        if (abs(averageRSSIAtGateway - cfgTgtRSSI) > TX_POW_RSSI_TOLERANCE){
            // if RSSI at Gateway is too weak, increase power
            if (averageRSSIAtGateway < cfgTgtRSSI &&
                    isTXPowValid(cfgTXPower + 1))
                changeTXPower(cfgTXPower + 1, false);
            // if RSSI at Gateway is too strong, decrease power
            else if  (averageRSSIAtGateway > cfgTgtRSSI &&
                    isTXPowValid(cfgTXPower - 1))
                changeTXPower(cfgTXPower - 1, false);
        }

        whenLastTXPowCheck = getNowTimestampSec();
    }
}


void toggleSleepMode(){
    canSleep = !canSleep;
    writeLogF(F("Sleep: "), logNull);
    writeLogLnF(canSleep ? F("ON") : F("OFF"), logNull);
}


int readLineSerial(int readChar, char *serialBuffer){
    /*
       Reads a CR-terminated line from serial RX.  Will reset state when a CR
       occurs.
   */

    int retPos;
    wdt_reset();
    // Check if first char is a 'z' - if so, process immediately to toggle sleep
    // mode
    if (SLEEP_ENABLED && readChar > 0 && serialBuffPos == 0 &&
            (readChar == 'z' || readChar == 'Z')){
        toggleSleepMode();
        return -1;
    }

    // Check if invalid chars.  Ignore.
    else if (readChar > 0 && (readChar < 32 || readChar > 127) &&
            readChar != '\r' && readChar != '\b')
        return -1;

    // Check if at maximum input length.  Ignore until deleted or return
    // pressed.
    else if (readChar > 0 && serialBuffPos >= (SERIAL_IN_BUFFER_SIZE -1) &&
            readChar != '\r' && readChar != '\b')
        return -1;

    else if (readChar > 0) {   // allow for null terminator
        switch (readChar) {
            case '\b':
                serialBuffPos--;
                serialBuffer[serialBuffPos] = '\0';

                // hacky way to realise working backspace -
                // backspace+space+backspace
                Serial.write("\b\x20\b");
                break;
            case '\r': // Return on CR
                printNewLine();
                retPos = serialBuffPos;
                serialBuffPos = 0;  // reset index
                return retPos;
            default:
                if (serialBuffPos < SERIAL_IN_BUFFER_SIZE - 1) {
                    serialBuffer[serialBuffPos++] = readChar;
                    serialBuffer[serialBuffPos] = '\0';
                    // echo the value that was read back to the serial port.
                    Serial.write(readChar);
                }
        }
    }
    // No end of line has been found, so return -1.
    return -1;
}


void checkSerialInput() {
    /*
       Checks for and handles serial commands
    */
    if (readLineSerial(Serial.read(), serInBuff) > 0) {
        char cmdVal[20];
        memset(cmdVal, '\0', sizeof(cmdVal));
        tmpInt = 0ul;

        enum cmdValid {
            invalid = 0,
            valid = 1,
            dump = 2
        } cmdStatus = invalid;

        // help
        if (strStartsWithP(serInBuff, SER_CMD_HELP) == 1){
            printCmdHelp();
            cmdStatus = valid;
        }

        // Dump - will trigger all other 'query' commands
        if (strStartsWithP(serInBuff, SER_CMD_DUMP) == 1){
            tmpRead = meterReadBuffer[getMtrBuffIxOffset(mrbPushNext, -1)];
            printPrompt();
            writeLogF(F("Last Meter... Time: "), logNull);
            writeLog(tmpRead.readTime, logNull);

            writeLogF(F(", Value: "), logNull);
            writeLog(tmpRead.meterValue, logNull);
            writeLogF(F(" "), logNull);
            printUnits();

            writeLogF(F(", Current: "), logNull);
            writeLog(tmpRead.rmsCurrent, logNull);
            writeLogF(F(" A"), logNull);
            printNewLine();

            printPrompt();
            writeLogF(F("New Raw Reads: "), logNull);
            writeLogLn(newRawMeterReads, logNull);

            printPrompt();
            writeLogF(F("New Closed Entries: "), logNull);
            writeLogLn(newClosedMeterEntries, logNull);

            printPrompt();
            writeLogF(F("Last MUPD: "), logNull);
            writeLogLn(whenLastMeterMsg, logNull);

            printPrompt();
            writeLogF(F("Meter Next Push Ix: "), logNull);
            writeLogLn(mrbPushNext, logNull);

            printPrompt();
            writeLogF(F("Meter Next Pop Ix: "), logNull);
            writeLogLn(mrbPopNext, logNull);

            printMeterBuffer();

            printPrompt();
            writeLogF(F("Batt (mV): "), logNull);
            writeLogLn(vinVoltageMV, logNull);

            printPrompt();
            writeLogF(F("Free RAM (B): "), logNull);
            writeLogLn(freeRAM(), logNull);

            printPrompt();
            writeLogF(F("Can Sleep: "), logNull);
            writeLogLnF(canSleep ? F("Y"): F("N"), logNull);

            printPrompt();
            writeLogF(F("Slept (s): "), logNull);
            writeLogLn(secondsSlept, logNull);

            cmdStatus = dump;
        }

        // reset config
        if (strStartsWithP(serInBuff, SER_CMD_RCFG) == 1){
            resetConfig();
            cmdStatus = valid;
        }

        // set time
        if (strStartsWithP(serInBuff, SER_CMD_TIME) == 2){
            strncpy(cmdVal, serInBuff + strlen_P(SER_CMD_TIME) + 1,
                    (strlen(serInBuff) - strlen_P(SER_CMD_TIME) -1));
            tmpInt = strtoul(cmdVal,NULL,0);
            if (tmpInt > 0){
                setNowTimestampSec(tmpInt);
                cmdStatus = valid;
            }
            else{
                printPrompt();
                writeLogF(F("Invalid Time"), logNull);
            }
        }

        // print time, also echoes after being set
        if (cmdStatus == dump || strStartsWithP(serInBuff, SER_CMD_TIME) >= 1){
            printPrompt();
            writeLogF(F("Time: "), logNull);
            printNowTime(logNull);
            printNewLine();
            if (cmdStatus != dump)
                cmdStatus = valid;
        }

        // set log level
        if (strStartsWithP(serInBuff, SER_CMD_LOGL) == 2){
            strncpy(tmpStr, serInBuff + strlen_P(SER_CMD_LOGL) + 1,
                    (strlen(serInBuff) - strlen_P(SER_CMD_LOGL) -1));
            if (strStartsWithP(tmpStr, LOG_ERROR_LBL) == 1)
                cfgLogLevel = logError;
            else if (strStartsWithP(tmpStr, LOG_WARN_LBL) == 1)
                cfgLogLevel = logWarn;
            else if (strStartsWithP(tmpStr, LOG_INFO_LBL) == 1)
                cfgLogLevel = logInfo;
            else if (strStartsWithP(tmpStr, LOG_DEBUG_LBL) == 1)
                cfgLogLevel = logDebug;
            else{
                printPrompt();
                writeLogLnF(F("Invalid Log Level"), logNull);
                return;
            }
            putConfigToMem();  // apply config and write to EEPROM
            cmdStatus = valid;
        }

        // print log level, also echoes after being set
        if (cmdStatus == dump || strStartsWithP(serInBuff, SER_CMD_LOGL) >= 1){
            printPrompt();
            writeLogF(F("Log Level: "), logNull);
            printLogLevel(cfgLogLevel, false);
            printNewLine();
            if (cmdStatus != dump)
                cmdStatus = valid;
        }

        // set radio encryption key
        if (strStartsWithP(serInBuff, SER_CMD_EKEY) == 2){
            strncpy(tmpStr, serInBuff + strlen_P(SER_CMDS[6]) + 1,
                    (strlen(serInBuff) - strlen_P(SER_CMDS[6]) -1));
            if (strlen(tmpStr) != KEY_LENGTH){
                printPrompt();
                writeLogLnF(F("Invalid Key"), logNull);
            }
            else{
                memcpy(cfgEncryptKey, tmpStr, strlen(tmpStr));
                putConfigToMem();
                applyRadioConfig();
                cmdStatus = valid;
            }
        }

        // print radio encryption key, also echoes after being set
        if (cmdStatus == dump || strStartsWithP(serInBuff, SER_CMD_EKEY) >= 1){
            printPrompt();
            writeLogF(F("Key: "), logNull);
            Serial.write(cfgEncryptKey, sizeof(cfgEncryptKey));
            printNewLine();
            if (cmdStatus != dump)
                cmdStatus = valid;
        }

        // set radio net id
        if (strStartsWithP(serInBuff, SER_CMD_NETI) == 2){
            strncpy(tmpStr, serInBuff + strlen_P(SER_CMD_NETI) + 1,
                    (strlen(serInBuff) - strlen_P(SER_CMD_NETI) -1));
            uint8_t addr1 = 0;
            uint8_t addr2 = 0;
            uint8_t addr3 = 0;
            uint8_t addr4 = 0;
            if(sscanf(tmpStr, "%hhu.%hhu.%hhu.%hhu",
                    &addr1, &addr2, &addr3, &addr4) != 4){
                printPrompt();
                writeLogLnF(F("Invalid Address"), logNull);
            }
            else{
                cfgNetworkId1 = addr1;
                cfgNetworkId2 = addr2;
                cfgNetworkId3 = addr3;
                cfgNetworkId4 = addr4;
                putConfigToMem();
                applyRadioConfig();
                cmdStatus = valid;
            }
        }

        // print radio net id, also echoes after being set
        if (cmdStatus == dump || strStartsWithP(serInBuff, SER_CMD_NETI) >= 1){
            printPrompt();
            printNetworkId();
            if (cmdStatus != dump)
                cmdStatus = valid;
        }

        // set gateway id
        if (strStartsWithP(serInBuff, SER_CMD_GWID) == 2){
            strncpy(cmdVal, serInBuff + strlen_P(SER_CMD_GWID) + 1,
                    (strlen(serInBuff) - strlen_P(SER_CMD_GWID) -1));
            tmpInt = strtoul(cmdVal,NULL,0);
            if (tmpInt < 1 || tmpInt > 253){
                printPrompt();
                writeLogLnF(F("Invalid Gateway Id"), logNull);
            }
            else{
                cfgGatewayId = tmpInt;
                putConfigToMem();
                applyRadioConfig();
                cmdStatus = valid;
            }
        }

        // print gateway id, also echoes after being set
        if (cmdStatus == dump || strStartsWithP(serInBuff, SER_CMD_GWID) >= 1){
            printPrompt();
            writeLogF(F("Gateway Id: "), logNull);
            writeLogLn(cfgGatewayId, logNull);
            if (cmdStatus != dump)
                cmdStatus = valid;
        }

        // set node id
        if (strStartsWithP(serInBuff, SER_CMD_NOID) == 2){
            strncpy(cmdVal, serInBuff + strlen_P(SER_CMD_NOID) + 1,
                    (strlen(serInBuff) - strlen_P(SER_CMD_NOID) -1));
            tmpInt = strtoul(cmdVal,NULL,0);
            if (tmpInt < 1 || tmpInt > 253){
                printPrompt();
                writeLogLnF(F("Invalid Node Id"), logNull);
            }
            else{
                cfgNodeId = tmpInt;
                putConfigToMem();
                applyRadioConfig();
                cmdStatus = valid;
            }
        }

        // print node id, also echoes after being set
        if (cmdStatus == dump || strStartsWithP(serInBuff, SER_CMD_NOID) >= 1){
            printPrompt();
            writeLogF(F("Node Id: "), logNull);
            writeLogLn(cfgNodeId, logNull);
            if (cmdStatus != dump)
                cmdStatus = valid;
        }

        // set target rssi
        if (strStartsWithP(serInBuff, SER_CMD_TRSS) == 2){
            strncpy(cmdVal, serInBuff + strlen_P(SER_CMD_TRSS) + 1,
                    (strlen(serInBuff) - strlen_P(SER_CMD_TRSS) -1));
            static int16_t tgtRSSI = 0;
            tgtRSSI = strtol(cmdVal,NULL,0);
            if (tgtRSSI < -100 || tgtRSSI > 0){
                printPrompt();
                writeLogLnF(F("Invalid RSSI"), logNull);
            }
            else{
                cfgTgtRSSI = tgtRSSI;
                putConfigToMem();
                cmdStatus = valid;
            }
        }

        // set TX power
        if (strStartsWithP(serInBuff, SER_CMD_TXPW) == 2){
            strncpy(cmdVal, serInBuff + strlen_P(SER_CMD_TXPW) + 1,
                    (strlen(serInBuff) - strlen_P(SER_CMD_TXPW) -1));
            static int32_t txPow = 0;
            txPow = strtol(cmdVal,NULL,0);

            if (! isTXPowValid(txPow)){
                printPrompt();
                writeLogLnF(F("Invalid TX Power"), logNull);
            }
            else{
                changeTXPower(txPow, true);
                cmdStatus = valid;
            }
        }

        // print radio/rssi stats, done when target rssi or tx power read/set
        if (cmdStatus == dump || strStartsWithP(serInBuff, SER_CMD_TRSS) >= 1 ||
                strStartsWithP(serInBuff, SER_CMD_TXPW) >= 1){
            printPrompt();
            writeLogF(F("TX Power: "), logNull);
            writeLogLn(cfgTXPower, logNull);
            printPrompt();
            writeLogF(F("Target RSSI: "), logNull);
            writeLogLn(cfgTgtRSSI, logNull);
            printPrompt();
            writeLogF(F("Last SSI from Gateway: "), logNull);
            writeLogLn(lastRSSIAtNode, logNull);
            printPrompt();
            writeLogF(F("Avg RSSI at Gateway: "), logNull);
            writeLogLn(averageRSSIAtGateway, logNull);
            if (cmdStatus != dump)
                cmdStatus = valid;
        }

        // set meter value
        if (strStartsWithP(serInBuff, SER_CMD_MTRV) == 2){
            strncpy(cmdVal, serInBuff + strlen_P(SER_CMD_MTRV) + 1,
                    (strlen(serInBuff) - strlen_P(SER_CMD_MTRV) -1));
            tmpInt = strtoul(cmdVal,NULL,0);
            if (tmpInt > 0 && tmpInt < METER_MAX_VALUE){
                pushToMeterBuffer(getNowTimestampSec(), tmpInt, true);
                printPrompt();
                writeLogF(F("Set Meter: "), logNull);
                writeLog(meterReadBuffer[
                    getMtrBuffIxOffset(mrbPushNext, -1)].meterValue, logNull);
                printUnits();
                printNewLine();
                forceRebase = true;
                cmdStatus = valid;
            }
        }

        // set meter unit
        if (strStartsWithP(serInBuff, SER_CMD_MTRU) == 2){
            strncpy(cmdVal, serInBuff + strlen_P(SER_CMD_MTRU) + 1,
                    (strlen(serInBuff) - strlen_P(SER_CMD_MTRU) -1));
            if (strlen(cmdVal) > KEY_LENGTH){
                printPrompt();
                writeLogF(F("Invalid unit: "), logNull);
                writeLog(cmdVal, logNull);
                writeLogF(F(". Must be < "), logNull);
                writeLog(KEY_LENGTH, logNull);
                writeLogLnF(F(" char"), logNull);
            }
            else{
                strncpy(cfgMeterUnit, cmdVal, KEY_LENGTH - 1);
                putConfigToMem();  // apply config and write to EEPROM
                cmdStatus = valid;
            }
        }

        // print meter unit, also echoes after being set
        if (cmdStatus == dump || strStartsWithP(serInBuff, SER_CMD_MTRU) >= 1){
            printPrompt();
            writeLogF(F("Meter Unit: "), logNull);
            printUnits();
            printNewLine();
            if (cmdStatus != dump)
                cmdStatus = valid;
        }

        // set meter interval
        if (strStartsWithP(serInBuff, SER_CMD_MTRI) == 2){
            strncpy(cmdVal, serInBuff + strlen_P(SER_CMD_MTRI) + 1,
                    (strlen(serInBuff) - strlen_P(SER_CMD_MTRI) -1));
            tmpInt = strtoul(cmdVal,NULL,0);
            if (tmpInt >= 0 && tmpInt <= UINT8_MAX){
                cfgMeterInterval = tmpInt;
                putConfigToMem();  // apply config and write to EEPROM
                cmdStatus = valid;
            }
        }

        // print meter interval, also echoes after being set
        if (cmdStatus == dump || strStartsWithP(serInBuff, SER_CMD_MTRI) >= 1){
            printPrompt();
            writeLogF(F("Meter Interval (s): "), logNull);
            writeLogLn(cfgMeterInterval, logNull);
            if (cmdStatus != dump)
                cmdStatus = valid;
        }

        // toggle testmode
        if (strStartsWithP(serInBuff, SER_CMD_TEST) == 2){
                testMode = ! testMode;
                if (testMode)
                    canSleep = false;
                printPrompt();
                writeLogF(F("Test: "), logNull);
                writeLogLnF(testMode ? F("ON") : F("OFF"), logNull);
                cmdStatus = valid;
        }

        // set LED rate
        if (strStartsWithP(serInBuff, SER_CMD_LEDR) == 2){
            strncpy(cmdVal, serInBuff + strlen_P(SER_CMD_TEST) + 1,
                    (strlen(serInBuff) - strlen_P(SER_CMD_TEST) -1));
            tmpInt = strtoul(cmdVal,NULL,0);
            if (tmpInt >= 0 && tmpInt <= UINT8_MAX){
                updatePuckLEDRate(tmpInt,true);
                cmdStatus = valid;
            }
            else{
                printPrompt();
                writeLogLnF(F("Invalid LED rate"), logNull);
            }
        }

        // print puck LED rate, also echoes after being set
        if (cmdStatus == dump || strStartsWithP(serInBuff, SER_CMD_LEDR) >= 1){
            printPrompt();
            writeLogF(F("LED: 1 per meter's "), logNull);
            writeLogLn(cfgPuckLEDRate, logNull);
            if (cmdStatus != dump)
                cmdStatus = valid;
        }

        // set LED time
        if (strStartsWithP(serInBuff, SER_CMD_LEDT) == 2){
            strncpy(cmdVal, serInBuff + strlen_P(SER_CMD_LEDR) + 1,
                    (strlen(serInBuff) - strlen_P(SER_CMD_LEDR) -1));
            tmpInt = strtoul(cmdVal,NULL,0);
            if (tmpInt >= 0 && tmpInt <= 3000){
                cfgPuckLEDTime = tmpInt;
                putConfigToMem();  // apply config and write to EEPROM
                cmdStatus = valid;
            }
            else{
                printPrompt();
                writeLogLnF(F("Invalid LED time"), logNull);
            }
        }

        // print puck LED time, also echoes after being set
        if (cmdStatus == dump || strStartsWithP(serInBuff, SER_CMD_LEDT) >= 1){
            printPrompt();
            writeLogF(F("LED Pulse (ms): "), logNull);
            writeLogLn(cfgPuckLEDTime, logNull);
            if (cmdStatus != dump)
                cmdStatus = valid;
        }

        if (cmdStatus == invalid){
            printPrompt();
            writeLogLnF(F("Invalid Command"), logNull);
            printCmdHelp();
        }
    }
}


void processMsgRecv(){
    /**
       Processes and dispatches a newly-received message.
     */

     lastRSSIAtNode = radio.lastRssi();
     writeLogF(F("Got msg: "), logDebug);
     writeLog((char*)radioMsgBuff, logDebug);
     writeLogF(F(". RSSI = "), logDebug);
     writeLogLn(lastRSSIAtNode, logDebug);

     strcpy(msgBuffStr, "");
     sprintf(msgBuffStr, "%s", (char*)radioMsgBuff);

    //  MVAI:
    //  meter instruction (from gateway) to set accumulation meter value
    //  format: MVAI;<new_meter_value>,<last_node_rssi>
    //  e.g.: MVAI;120000,-70
     if (strStartsWithP(msgBuffStr, RMSG_MVAI) == 1){
        // get new meter value from message
        static uint32_t newRead = 0ul;
        sscanf (msgBuffStr, "%*[^,],%lu,%hhd", &newRead, &lastRSSIAtGateway);

        if (newRead > 0 && newRead < UINT32_MAX){
            pushToMeterBuffer(getNowTimestampSec(), newRead, true);
            writeLogF(F("Got MVAI. Changed ("), logInfo);
            if (cfgLogLevel >= logInfo)
                printUnits();
            writeLogF(F("): "), logInfo);
            writeLog(meterReadBuffer[
                    getMtrBuffIxOffset(mrbPushNext, -2)].meterValue, logInfo);
            writeLogF(F(" to: "), logInfo);
            writeLogLn(newRead, logInfo);
        }
        else
            writeLogLnF(F("Got invalid MVAI"), logError);
    }

    // MINI:
    // meter instruction (from gateway) to set accumulation meter interval
    // format: MINI,<new_interval>,<last_node_rssi>
    // e.g.: MINI;5,-70
    else if (strStartsWithP(msgBuffStr, RMSG_MINI) == 1){
            // get new meter interval from message
            static uint32_t newInterval = 0ul;
            sscanf (msgBuffStr, "%*[^,],%lu,%hhd", &newInterval,
                    &lastRSSIAtGateway);

            if (newInterval >= 0 && newInterval <= UINT8_MAX){
                writeLogF(F("Got MINI. Changed "), logInfo);
                writeLog(cfgMeterInterval, logInfo);
                writeLogF(F(" secs to "), logInfo);
                writeLogLn(newInterval, logInfo);
                cfgMeterInterval = (uint8_t) newInterval;
                putConfigToMem();
            }
            else
                writeLogLnF(F("Got invalid MINI"), logError);
        }

    // MPLI:
    // meter instruction (from gateway) to set LED pulse rate and duration
    // format: MPLI,<led_pulse_rate>,<led_pulse_duration>,<last_node_rssi>
    // e.g.: MPLI;1,500,-70
    else if (strStartsWithP(msgBuffStr, RMSG_MPLI) == 1){
        // get new LED puck settings from message
        static uint32_t ledPulseRate;
        static uint32_t ledPulseTime;
        ledPulseRate = 0ul;
        ledPulseTime = 0ul;

        sscanf (msgBuffStr, "%*[^,],%lu,%lu,%hhd", &ledPulseRate,
                &ledPulseTime, &lastRSSIAtGateway);

        if (ledPulseRate <= UINT8_MAX && ledPulseTime <= 3000){
            cfgPuckLEDTime = (uint16_t) ledPulseTime;
            writeLogLnF(F("Got MPLI"), logInfo);
            updatePuckLEDRate((uint8_t) ledPulseRate, false);
            writeLog(cfgPuckLEDRate, logInfo);
            writeLogF(F(" LED Pulse (ms): "), logInfo);
            writeLogLn(cfgPuckLEDTime, logInfo);
            putConfigToMem();
        }
        else
            writeLogLnF(F("Got invalid MPLI"), logError);
    }

    // PRSP:
    // ping response from gateway, used to sync clock
    // format: PRSP,<request_time_node>,<current_time_gateway>,<last_node_rssi>
    // e.g.:   PRSP;14968429155328,1496842915428,-70
    else if (strStartsWithP(msgBuffStr, RMSG_PRSP) == 1){
        // get new clock value from message
        static uint32_t reqTime;
        static uint32_t respTime;
        reqTime = 0ul;
        respTime = 0ul;
        sscanf (msgBuffStr, "%*[^,],%lu,%lu,%hhd", &reqTime, &respTime,
                    &lastRSSIAtGateway);
        if ((getNowTimestampSec() - reqTime <= SYNC_TIME_MAX_DRIFT_SEC) &&
                (respTime > 0) &&
                (abs(getNowTimestampSec() - respTime) > SYNC_TIME_MAX_DRIFT_SEC)
            ){
            //compensate for latency from gateway
            respTime += ((getNowTimestampSec() - reqTime) / 2);
            writeLogF(F("Got PRSP. Changed from "), logDebug);
            writeLog(getNowTimestampSec(), logDebug);
            writeLogF(F(" to "), logDebug);
            writeLogLn(respTime, logDebug);
            setNowTimestampSec(respTime);
        }
        else
            writeLogLnF(F("Got PRSP. No update"), logInfo);
    }

    // MNOI:
    // 'no op' ACK to GINR, provides RSSI for auto-tuning
    // format: MNOI,<last_node_rssi>
    // e.g.: MNOI;-70
    else if (strStartsWithP(msgBuffStr, RMSG_MNOI) == 1){
        sscanf (msgBuffStr, "%*[^,],%hhd", &lastRSSIAtGateway);
    }

    else if (strStartsWithP(msgBuffStr, RMSG_GMSG) == 1){
        writeLogF(F("Got bcast: "), logDebug);
        writeLogLn(msgBuffStr, logDebug);
    }

    else {
        writeLogF(F("Unexpected msg: "), logWarn);
        writeLogLn(msgBuffStr, logWarn);
    }

    strcpy(msgBuffStr, "");
    // last message received may have RSSI from this node as measured at Gateway
    checkTXtoRSSI();
}


void checkRadioMsg(){
    /**
       Checks for newly received messages and calls processMsgRecv() - which is
       also called from within sendRadioMsg - hence separate function.
     */

    if (msgManager.available()){
        uint8_t lenBuff = sizeof(radioMsgBuff);
        memset(radioMsgBuff, 0, sizeof(radioMsgBuff));

        // Check for newly arrived message and ACK.  No wait/timeout as not
        // assured of having one.
        if (msgManager.recvfromAck(radioMsgBuff, &lenBuff, &lastMsgFrom))
            processMsgRecv();
    }
    wdt_reset();
    radio.sleep();
}


void sendRadioMsg(uint8_t recipient, bool checkReply){
    /*
        Sends whatever's in msgBuffStr to radio recipient
    */

    if (strlen(msgBuffStr) > RH_RF69_MAX_MESSAGE_LEN){
       writeLogF(F("Msg send failed. Too long: "), logError);
       writeLogLn(msgBuffStr, logError);
       return;
    }

    uint8_t lenBuff = sizeof(radioMsgBuff);
    memset(radioMsgBuff, 0, sizeof(radioMsgBuff));

    writeLogF(F("Sending: "), logDebug);
    writeLogLn(msgBuffStr, logDebug);
    memcpy(radioMsgBuff, msgBuffStr, strlen(msgBuffStr));
    wdt_reset();

    // Send message with an ack timeout as specified by TX_TIMEOUT
    if (msgManager.sendtoWait(radioMsgBuff, lenBuff, recipient)){
        txConsFailCount = 0;
        // Wait for a reply from the Gateway if instructed to
        // TODO: seems unreliable for GINR
        if (checkReply &&
            msgManager.recvfromAckTimeout(radioMsgBuff, &lenBuff, RX_TIMEOUT,
                    &lastMsgFrom))
            processMsgRecv();
        else if (checkReply)
            writeLogLnF(F("No ACK recv"), logInfo);
    }
    else {
        writeLogF(F("Failed to send msg: "), logWarn);
        writeLogLn(msgBuffStr, logWarn);
        // reset TX power to max in case too weak - auto-tuning will
        // detune as needed
        txConsFailCount++;
        if (txConsFailCount >= TX_POW_MAX_FAILS_TO_ADJ &&
                cfgTXPower < getTXPowMax())
            averageRSSIAtGateway = -100;    // force signal strength increase
    }
    wdt_reset();
    radio.sleep();
}


void sleepNode(){
    // do not sleep if there is buffered serial input, or it's
    // less than 15s after boot
    if (!SLEEP_ENABLED || !canSleep || serialBuffPos > 0 ||
            getNowTimestampSec() - whenBooted < 15)
        return;

    writeLogLnF(F("Sleeping til interrupt"), logDebug);
    static uint32_t whenSleepStarted;
    whenSleepStarted = getNowTimestampSec();
    radio.sleep();      // should be asleep already

    // set SPI SS pins to high (disable), not LOW.
    // testing shows no benefit of setting other pins to output/low
    digitalWrite(RADIO_SS_PIN, HIGH);
    digitalWrite(RTC_SS_PIN, HIGH);

    // sleep MCU, Timers, etc (most functions except interrupt); with AD
    // Converter and Brownout Detector off FOREVER best option if interrupt
    // fired regularly as the turns watchdog timer off (unlike 8S and other
    // timed options)
    Serial.flush();
    wdt_disable();

    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
    wdt_enable(WDTO_8S);

    //awake
    // don't bother with overflow as > 100 years!
    secondsSlept += getNowTimestampSec() - whenSleepStarted;

}


uint16_t getAvgAnalogVal(uint8_t analogPin){
    /**
       Generic analog read (used for batt)
       TODO: change to RMS and use for CT clamp?
     */
    static uint16_t pinRead = 0;
    static uint8_t i = 0;

    // throw-away reads to stabilise
    for (i = 0; i < 3; i++){
        analogRead(analogPin);
        delay(100);
    }

    pinRead = 0;
    for (i = 0; i < 10; i++)
        pinRead += analogRead(analogPin);

    return (pinRead / 10);
}


void checkBatt(){
    /**
       Checks battery voltage at voltage divider.
     */

    // voltage =
    //      (analog_reading/voltage_divisor)
    //      * voltage_div_scaling_multiplier
    //      * correction_multiplier
    vinVoltageMV = (getAvgAnalogVal(VIN_DIV_PIN) / VIN_DIV_VOLT_DIVISOR) *
                        VIN_DIV_RSCALE_MULT * VIN_DIV_CORRECT_MULT;
    writeLogF(F("Battery Voltage (mV): "), logDebug);
    writeLogLn(vinVoltageMV, logDebug);
}


void checkMeterRebase(){
    // Sent at boot or whenever meter or RTC adjusted to avoid big leaps in time
    // or meter values Assumes meter has been updated
    //
    // MREB:  meter rebase (to gateway) - a time and value baseline
    // format: MREB,<meter_time_start>,<meter_value_start>;
    // e.g.:   MREB,1496842913428,18829393;
    if (forceRebase || (newClosedMeterEntries > 0 && (whenLastMeterMsg == 0))){
        bool gotRead = popFromMeterBuffer(tmpRead, true);
        if (gotRead){
            sprintf_P(msgBuffStr, RMSG_MREBASE);
            sprintf(msgBuffStr, "%s,%lu,%lu", msgBuffStr, tmpRead.readTime,
                    tmpRead.meterValue);
            sendRadioMsg(cfgGatewayId, false);
            whenLastMeterMsg = getNowTimestampSec();
        }
        else
            writeLogLnF(F("Rebase failed - couldn't find latest read"),
                    logError);
        forceRebase = false;
    }
}


void checkMeterTimer(){
    /**
        Sends a meter update to gateway on a timer.

        Two variants of update message - one with CT clamp current, one without.

        MUP_:  meter update (to gateway) - a digest of timestamped accumulation
                meter readings without current reads
        format: MUP_,1 of [<meter_time_start>, <meter_value_start>];
                    2..n of [<interval_duration>, <interval_value>]
        e.g.:   MUP_,1496842913428,18829393;15,1;15,5;15,2;16,3;
        should fit 4 entries unless using > 999Wh per interval

        MUPC:  meter update (to gateway) - a digest of timestamped accumulation
                meter readings with current reads
        format: MUPC,1 of [<meter_time_start>, <meter_value_start>];
                2..n of
                    [<interval_duration>, <interval_value>, <spot_rms_current>]
        e.g.:
            MUPC,1496842913428,18829393;15,1,10.2;15,5,10.7;
        should fit 2-3 entries
     */

     // Meter reads are counted in a raw counter variable and put into the
     // buffer asynchronously to allow for higher-frequency LED rates; empty it
     // if it has reads
     if (newRawMeterReads > 0){
         pushToMeterBuffer(getNowTimestampSec(), newRawMeterReads, false);
         newRawMeterReads = 0;
     }
     else
        checkStaleEntry();

    checkMeterRebase();     // check if set to do rebase

    if ((! SEND_METER_TIMER_ONLY &&
            newClosedMeterEntries >= METER_MSG_MAX_ELEMENTS) ||
            (newClosedMeterEntries > 0 &&
            (getNowTimestampSec() - whenLastMeterMsg >=
                MAX_METER_MSG_INTERVAL_SEC))
            ){
        // write beginning of message
        if (CT_ENABLED)
            sprintf_P(msgBuffStr, RMSG_MUPC);  // msg_type with CT
        else
            sprintf_P(msgBuffStr, RMSG_MUP_);  // msg_type without CT

        static uint8_t max_meter_elements;
        max_meter_elements = CT_ENABLED ? 2: METER_MSG_MAX_ELEMENTS;

        // get start timestamp and meter value (i.e. result of previous read
        // sent)
        uint32_t lastTime =
                meterReadBuffer[getMtrBuffIxOffset(mrbPopNext, -1)].readTime;
        uint32_t lastRead =
                meterReadBuffer[getMtrBuffIxOffset(mrbPopNext, -1)].meterValue;

        sprintf(msgBuffStr, "%s,%lu,%lu", msgBuffStr, lastTime, lastRead);

        if (SEND_METER_TIMER_ONLY &&
                newClosedMeterEntries >= METER_MSG_MAX_ELEMENTS)
            // just send a single 'full' message, ignoring older reads as we are
            // sending on a timer advance meter buffer read pointer accordingly
            // if we have more reads than will fit
            mrbPopNext =
                getMtrBuffIxOffset(mrbPushNext, (-1 - METER_MSG_MAX_ELEMENTS));

        static uint8_t i;
        i = 1;

        // append remaining reads until ring buffer empty or we 'fill'
        // the message

        bool gotRead = true;

        while (gotRead && (i <= max_meter_elements)){
            gotRead = popFromMeterBuffer(tmpRead, false);
            if (gotRead){
                if (tmpRead.readTime >= lastTime && tmpRead.meterValue >=
                        lastRead){
                    sprintf(msgBuffStr, "%s;%lu,%lu", msgBuffStr,
                        tmpRead.readTime - lastTime,
                        tmpRead.meterValue - lastRead);
                    if (CT_ENABLED)
                        sprintf(msgBuffStr, "%s,%d.%01d", msgBuffStr,
                            (int)tmpRead.rmsCurrent,
                            (int)(tmpRead.rmsCurrent*100)%100);
                }
                lastTime = tmpRead.readTime;
                lastRead = tmpRead.meterValue;
            }
            i++;
        }

        sendRadioMsg(cfgGatewayId, false);
        whenLastMeterMsg = getNowTimestampSec();
    }
}


void checkGWInstTimer(){
    /**
       Sends an instruction request to gateway, includes node status data.
       Will get a MINI if no instruction.
       Also (first) checks if any unsolicited messages have been received.

       GINR:
               Instruction request (to gateway).  Also includes status data.
       format: GINR;<battery_voltage>,<uptime_secs>,<sleeptime_secs>,<free_ram>,
                    <last_rssi>,<puck_led_rate>,<puck_led_time>,
                    <meter_interval_time>
       e.g.:   GINR;4300,890000,555000,880,-80,10,100,5
     */

     // check for missed instruction messages
    checkRadioMsg();
    if (whenLastGInstMsg == 0 || (getNowTimestampSec() - whenLastGInstMsg) >=
            GINR_MSG_INTERVAL_SEC){
        checkBatt();
        static uint32_t secondsUptime;
        secondsUptime = getNowTimestampSec() - whenBooted;

        sprintf_P(msgBuffStr, RMSG_GINR);
        sprintf(msgBuffStr, "%s,%d,%lu,%lu,%d,%d,%hhu,%d,%hhu", msgBuffStr,
                vinVoltageMV, secondsUptime, secondsSlept, freeRAM(),
                lastRSSIAtNode, cfgPuckLEDRate, cfgPuckLEDTime,
                cfgMeterInterval);

        sendRadioMsg(cfgGatewayId, true);
        whenLastGInstMsg = getNowTimestampSec();
    }

}

void checkGWClockSync(){
    /**
       Sends a ping request to gateway, response will sync clock.

       PREQ: ping request to gateway
           format: PREQ;<current_time_local>
           e.g.:   PREQ;1496842913428

     */

    if (whenLastPingMsg == 0 || getNowTimestampSec() - whenLastPingMsg >=
            PING_MSG_INTERVAL_SEC){
        sprintf_P(msgBuffStr, RMSG_PREQ);
        sprintf(msgBuffStr, "%s,%lu", msgBuffStr, getNowTimestampSec());
        sendRadioMsg(cfgGatewayId, true);
        whenLastPingMsg = getNowTimestampSec();
    }
}


void blinkLED(uint8_t blinkTimes){
    for (uint8_t i = 1; i <= blinkTimes; i++){
        PORTD = PORTD | B00010000;  // on
        delay(500);
        PORTD = PORTD & B11101111;  // off
        delay(250);
    }
}


void checkTestMode(){
    /*
        If test mode is enabled via TESTMODE serial commmand, simulates meter
        LED pulses at TEST_METER_PULSE_INTERVAL (minimum delay between pulses)
        with TEST_METER_PULSE_TIME setting the duration of the pulse itself
        (pulse width).
    */
    if (!testMode)
        return;

    if (testPulseOn && millis() - whenTestPulseChange >= TEST_METER_PULSE_TIME){
        // pulse is on, pulse time is up, so turn off, simulate rising edge, and
        // reset timer.
        testPulseOn = false;
        PORTD = PORTD & B11101111;      // LED off
        whenTestPulseChange = millis();
    }
    else if (!testPulseOn && millis() - whenTestPulseChange >=
            TEST_METER_PULSE_INTERVAL){
        // pulse is off, interval time is up, so turn on, simulate falling edge.
        testPulseOn = true;
        watchedLEDOn();
        whenTestPulseChange = millis();
    }
}


void checkButton(){
    /*
        Button is multi-function...

        If in sleep mode, any detected press will set sleep off (3 flashes).
        Release as soon as flashing starts.  Will need to hold down until
        interrupt fires (i.e. watched LED pulses).

        If not sleeping, long press of > 3s will start a 60s adjustment mode (7
        flashes) that forces meter LED rate to 1, and time to 0 (mimic watched
        LED).

        If not sleeping, short press of < 1s will set sleep on (5 flashes).
        Release as soon as flashing starts.
    */
    static bool btnDown = false;
    // button is pin 6 with external pull-up (on is LOW)
    btnDown = ((PIND & B01000000) == B00000000);

    if (canSleep && btnDown){
        // button has been pressed while asleep => wake up!
        toggleSleepMode();
        blinkLED(3);
        btnEventStartMillis = 0ul;
    }

    // new event...
    else if (btnDown && btnEventStartMillis == 0 && !btnAdjustMode)
        btnEventStartMillis = millis();

    else if (!canSleep && !btnAdjustMode && !btnDown && btnEventStartMillis > 0
                && (millis() - btnEventStartMillis <= 1000)){
        // button has been pressed and released in <= 1s => sleep on/off toggle
        toggleSleepMode();
        blinkLED(5);
        btnEventStartMillis = 0ul;
    }

    else if (!canSleep && !btnAdjustMode && !btnDown && btnEventStartMillis > 0
            && (millis() - btnEventStartMillis > 1000)
            && (millis() - btnEventStartMillis < 3000))
        // button has been pressed and released in 1-3s => ignore
        btnEventStartMillis = 0ul;

    else if (!canSleep && !btnDown && !btnAdjustMode && btnEventStartMillis > 0
                && (millis() - btnEventStartMillis >= 3000)){
        // button has been pressed and released in >3s => adjustment on
        // for 60s then returns LED rate to 0
        writeLogLnF(F("PR Adj Mode On"), logNull);
        blinkLED(7);
        btnAdjustMode = true;
        btnEventStartMillis = millis();
        btnFunctionTimer = btnEventStartMillis;
        updatePuckLEDRate(1, false);
        cfgPuckLEDTime = 0;
    }

    // if in adjust mode, output voltage every 60s
    if (btnAdjustMode && (millis() - btnFunctionTimer >= 1000)){
        btnFunctionTimer = millis();
        if (millis() - btnEventStartMillis > 60000) {
            // reset puck LED rate to 0, will need to chenge or reboot to
            // revert to original
            writeLogLnF(F("PR Adj Mode Off"), logNull);
            btnEventStartMillis = 0ul;
            btnFunctionTimer = 0ul;
            btnAdjustMode = false;
            updatePuckLEDRate(0, false);
        }
    }

}


// save reset cause from bootloader
void resetFlagsInit(void) __attribute__ ((naked))
                          __attribute__ ((used))
                          __attribute__ ((section (".init0")));


// save reset flags passed from bootloader
void resetFlagsInit(void)
{
  __asm__ __volatile__ ("sts %0, r2\n" : "=m" (resetFlags) :);
}


void printResetVal(uint8_t resetVal){
    writeLogF(F("R_FLAGS: 0x"), logNull);
    Serial.print(resetVal, HEX);

    // check for the reset bits.  Symbols are
    // bit flags, have to be shifted before comparison.

    if (resetVal & (1<<WDRF))
    {
        // watchdog
        writeLogF(F(" WD"), logNull);
        resetVal &= ~(1<<WDRF);
    }
    if (resetVal & (1<<BORF))
    {
        // brownout
        writeLogF(F(" BO"), logNull);
        resetVal &= ~(1<<BORF);
    }
    if (resetVal & (1<<EXTRF))
    {
        // external
        writeLogF(F(" EX"), logNull);
        resetVal &= ~(1<<EXTRF);
    }
    if (resetVal & (1<<PORF))
    {
        // power on
        writeLogF(F(" PO"), logNull);
        resetVal &= ~(1<<PORF);
    }
    if (resetVal != 0x00)
    // unknown - should not happen
        writeLogF(F(" UN"), logNull);

    printNewLine();
}


void setup() {
    // initialise pins

    pinMode(BUTTON_PIN, INPUT);

    pinMode(PUCK_LED_PIN, OUTPUT);
    digitalWrite(PUCK_LED_PIN, LOW);

    pinMode(VIN_DIV_PIN, INPUT);
    pinMode(CT_CLAMP_PIN, INPUT);

    // ensure PR_INTERRUPT_PIN is not pulled up as this will 'defeat' trimpot
    // (reduce resistance on line by providing lower resistance path from 3V3)
    pinMode(PR_INTERRUPT_PIN, INPUT);

    // disconnected pins 'pulled up' rather than left floating
    pinMode(5, INPUT_PULLUP);
    pinMode(8, INPUT_PULLUP);
    pinMode(9, INPUT_PULLUP);
    pinMode(A2, INPUT_PULLUP);
    pinMode(A3, INPUT_PULLUP);
    pinMode(A4, INPUT_PULLUP);
    pinMode(A5, INPUT_PULLUP);

    // wait until connection is up
    while (!Serial)
        delay(1);
    Serial.begin(SERIAL_BAUD);

    printNewLine();
    printNewLine();
    writeLogLnF(F("===BOOT==="), logNull);
    printResetVal(resetFlags);

    // get config from EEPROM
    getConfigFromMem();
    applyRadioConfig();

    // initialise RTC, set time to default (should be overridden by update
    // from gateway)
    writeLogLnF(F("Clock Init"), logDebug);
    rtc.reset();
    setNowTimestampSec(INIT_TIME);      // will leave SPI CS/SS pin high/off

    blinkLED(3);

    whenBooted = getNowTimestampSec();

    wdt_enable(WDTO_8S);    //Time for wait before autoreset

    sprintf_P(msgBuffStr, RMSG_GMSG);
    sprintf(msgBuffStr, "%s,BOOT v%hhu. Flags: %hhu", msgBuffStr, FW_VERSION,
            resetFlags);
    sendRadioMsg(cfgGatewayId, false);

    emon1.current(CT_CLAMP_PIN, CT_IRMS_CORRECT_MULT);

}


void loop() {
    /*
        Do processing, comms on a prioritised basis before sleeping.
        Some functions are run multiple times (rather than a single loop)
        as they involve human input and need to allow time for that input
    */

    static uint8_t doEvery = 0;

    for (doEvery = 1; doEvery <=6; doEvery++){
        // do every time
        checkSerialInput();
        checkButton();
        checkTestMode();
        wdt_reset();

        // do processing if not in middle of serial input
        if (serialBuffPos == 0)
            checkPuckLEDState();

        if (serialBuffPos == 0 && doEvery % 2 == 1)
            checkMeterTimer();

        else if (serialBuffPos == 0 && doEvery == 2)
            checkGWInstTimer();

        else if (serialBuffPos == 0 && doEvery == 4)
            checkGWClockSync();

        else if (serialBuffPos == 0 && doEvery == 6)
            sleepNode();
    }
}
