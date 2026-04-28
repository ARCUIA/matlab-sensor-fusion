/*
 * ARCUIA Sensor Fusion — Teensy 4.x main sketch
 *
 *   9-DOF IMU  : SparkFun ICM-20948  (accel + gyro + mag in one chip)
 *                I2C — connect SDA→18, SCL→19, ADR→GND (0x68) or VCC (0x69)
 *
 *   GPS        : u-blox NEO-M8N / M9N (or any NMEA GPS @ 9600/38400 baud)
 *                TX→Teensy RX1 (pin 0)
 *                Configure for 10 Hz NMEA RMC + GGA, or use UBX-NAV-PVT
 *
 *   USB Serial : 1 Mbaud CSV stream consumed by visualize_live.py
 *
 * Loop strategy:
 *   • Main loop polls IMU at ~500 Hz (every 2000 µs).
 *   • GPS bytes drained continuously and fed to TinyGPS++ —
 *     when a valid fix arrives, filter.feedGPS() is called.
 *   • CSV is printed at every IMU tick (500 lines/s).
 */

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include "ICM_20948.h"      // SparkFun ICM-20948
#include <TinyGPS++.h>      // NMEA GPS parser
#include "AHRSFilter.h"

// ─── Hardware objects ────────────────────────────────────────────────────────
ICM_20948_I2C  imu;
TinyGPSPlus    gps;
HardwareSerial& GPSSerial = Serial1;          // GPS on UART1

AHRSFilter     filter(0.033f);                // Madgwick gain

// ─── Tick scheduling ─────────────────────────────────────────────────────────
constexpr uint32_t IMU_PERIOD_US = 2000;      // 500 Hz
uint32_t lastTickUs = 0;

// ─── Unit conversion ─────────────────────────────────────────────────────────
constexpr float MG_TO_MS2  = 9.80665e-3f;     // milli-g → m/s²
constexpr float DPS_TO_RPS = 0.01745329252f;  // deg/s   → rad/s

// ─────────────────────────────────────────────────────────────────────────────
//  IMU Setup
// ─────────────────────────────────────────────────────────────────────────────
void setupIMU() {
    bool ok = false;
    while (!ok) {
        imu.begin(Wire, 1);                   // ADR pin high → 0x69
        ok = (imu.status == ICM_20948_Stat_Ok);
        if (!ok) {
            Serial.println("# ICM-20948 not found, retrying…");
            delay(500);
        }
    }

    imu.swReset();          delay(50);
    imu.sleep(false);
    imu.lowPower(false);

    // Continuous sample mode for both accel/gyro and magnetometer
    imu.setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr),
                      ICM_20948_Sample_Mode_Continuous);

    // Full-scale ranges
    ICM_20948_fss_t fss;
    fss.a = gpm16;          // ±16 g  (rocketry needs headroom for boost)
    fss.g = dps2000;        // ±2000 dps
    imu.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), fss);

    // Digital low-pass filter
    ICM_20948_dlpcfg_t dlp;
    dlp.a = acc_d50bw4_n68bw8;
    dlp.g = gyr_d51bw2_n73bw3;
    imu.setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), dlp);
    imu.enableDLPF(ICM_20948_Internal_Acc, true);
    imu.enableDLPF(ICM_20948_Internal_Gyr, true);

    imu.startupMagnetometer();                // AK09916 auxiliary
    Serial.println("# IMU online");
}

// ─────────────────────────────────────────────────────────────────────────────
//  GPS Setup
// ─────────────────────────────────────────────────────────────────────────────
void setupGPS() {
    GPSSerial.begin(9600);                    // u-blox default
    // (Optional) send UBX configuration packets here for 10 Hz / higher baud
    Serial.println("# GPS UART open");
}

// ─────────────────────────────────────────────────────────────────────────────
//  Read IMU and run filter step
// ─────────────────────────────────────────────────────────────────────────────
void readImuAndStep(float dt) {
    if (!imu.dataReady()) return;

    imu.getAGMT();   // pulls accel, gyro, mag, temp in one transaction

    float accel[3] = {
        imu.accX() * MG_TO_MS2,
        imu.accY() * MG_TO_MS2,
        imu.accZ() * MG_TO_MS2
    };
    float gyro[3] = {
        imu.gyrX() * DPS_TO_RPS,
        imu.gyrY() * DPS_TO_RPS,
        imu.gyrZ() * DPS_TO_RPS
    };
    float mag[3] = {                          // already in µT
        imu.magX(),
        imu.magY(),
        imu.magZ()
    };

    filter.update(accel, gyro, mag, dt);
    filter.printCSV();
}

// ─────────────────────────────────────────────────────────────────────────────
//  Drain GPS UART, feed filter when a new fix is parsed
// ─────────────────────────────────────────────────────────────────────────────
void serviceGPS() {
    static uint32_t lastFixAge = 0xFFFFFFFF;

    while (GPSSerial.available()) {
        gps.encode(GPSSerial.read());
    }

    // Only push to filter on fresh fix
    uint32_t age = gps.location.age();
    if (gps.location.isValid() && age < 200 && age != lastFixAge) {
        lastFixAge = age;

        float lla[3] = {
            (float)gps.location.lat(),
            (float)gps.location.lng(),
            (float)gps.altitude.meters()
        };

        // NMEA RMC gives speed-over-ground + course → derive horizontal NED vel.
        // Vertical velocity not in standard NMEA — leave 0 (filter relies on IMU).
        // For true 3D velocity, switch to UBX-NAV-PVT (gives velN/velE/velD).
        float v   = gps.speed.mps();
        float crs = gps.course.deg() * (float)DEG_TO_RAD;
        float velNED[3] = {
            v * cosf(crs),     // North
            v * sinf(crs),     // East
            0.0f               // Down  (use baro/IMU instead)
        };

        filter.feedGPS(lla, velNED);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Arduino setup() / loop()
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(1000000);
    while (!Serial && millis() < 3000) {}     // wait briefly for USB host

    Wire.begin();
    Wire.setClock(400000);                    // 400 kHz I2C

    Serial.println("# ARCUIA Sensor Fusion booting");
    setupIMU();
    setupGPS();
    Serial.println("# Ready — streaming CSV @ 500 Hz");

    lastTickUs = micros();
}

void loop() {
    uint32_t now = micros();

    // 500 Hz IMU + filter tick
    if ((uint32_t)(now - lastTickUs) >= IMU_PERIOD_US) {
        float dt = (float)(now - lastTickUs) * 1e-6f;
        lastTickUs = now;
        readImuAndStep(dt);
    }

    // Always drain GPS (cheap, non-blocking)
    serviceGPS();
}
