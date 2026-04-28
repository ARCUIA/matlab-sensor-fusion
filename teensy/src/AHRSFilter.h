/*
 * AHRSFilter.h
 * ARCUIA Sensor Fusion — Teensy 4.x
 *
 * Architecture:
 *   Madgwick AHRS  →  quaternion orientation  (accel + gyro + mag @ 500 Hz)
 *   6-state EKF    →  NED position + velocity  (IMU propagation + GPS @ 10 Hz)
 *
 * Build with PlatformIO (VS Code). platformio.ini board: teensy41
 */

#pragma once
#include <Arduino.h>
#include <math.h>

// ─── All computed output variables ──────────────────────────────────────────
struct SensorOutputs {
    float pos[3];        // NED position      [m]          (North, East, Down)
    float vel[3];        // NED velocity      [m/s]
    float orient[4];     // quaternion        [w, x, y, z]

    float euler[3];      // ZYX Euler angles  [rad]        [0]=yaw [1]=pitch [2]=roll
    float heading;       // yaw               [deg]        0–360
    float rollPitch[2];  // [pitch_deg, roll_deg]          from euler[1] and euler[2]

    float altitude;      // altitude          [ft]         = -pos[2] * 3.28084
    float climbRate;     // climb rate        [ft/min]     = -vel[2] * 196.85
    float airspeed;      // airspeed          [knots]      = |vel| * 1.944
    float baroPressure;  // baro pressure     [Pa]         ICAO standard atmosphere
};

// ─── Filter class ────────────────────────────────────────────────────────────
class AHRSFilter {
public:
    SensorOutputs out;

    // beta: Madgwick gradient-descent gain
    //   higher = faster tilt correction, more noise sensitivity
    //   0.033 is a good starting point at 500 Hz
    explicit AHRSFilter(float beta = 0.033f);

    // Set GPS reference origin for LLA→NED conversion.
    // Call once on first GPS fix, or hard-code launch site coords.
    void init(float refLat_deg, float refLon_deg, float refAlt_m);

    // Main 500 Hz call — feed raw IMU + mag readings every dt seconds
    //   accel : body-frame [m/s²]   (x, y, z)
    //   gyro  : body-frame [rad/s]  (x, y, z)
    //   mag   : body-frame [µT]     (x, y, z)
    //   dt    : sample period [s]   (use 0.002 for 500 Hz)
    void update(const float accel[3], const float gyro[3],
                const float mag[3],   float dt);

    // GPS correction — call whenever a new GPS fix arrives (~10 Hz)
    //   lla    : [latitude_deg, longitude_deg, altitude_m]
    //   velNED : velocity in NED frame [m/s]
    void feedGPS(const float lla[3], const float velNED[3]);

    // Print one CSV line over Serial for live visualization
    // Format: px,py,pz,vx,vy,vz,qw,qx,qy,qz,alt,clb,spd,hdg,pitch,roll,yaw,baro
    void printCSV() const;

private:
    // ── Madgwick state ───────────────────────────────────────────────────────
    float _q[4];      // quaternion [w, x, y, z]
    float _beta;

    // ── Navigation EKF — state [px, py, pz, vx, vy, vz] ────────────────────
    float _x[6];        // state vector
    float _cov[6][6];   // state covariance

    // ── LLA reference ────────────────────────────────────────────────────────
    float _refLat;    // radians
    float _refLon;    // radians
    float _refAlt;    // meters
    bool  _refSet;

    // ── Internal methods ─────────────────────────────────────────────────────
    void madgwickUpdate(const float a[3], const float g[3],
                        const float m[3], float dt);
    void navPredict    (const float accel[3], float dt);
    void navCorrectGPS (const float posNED[3], const float velNED[3]);
    void computeOutputs();

    // ── Static math helpers ──────────────────────────────────────────────────
    static void  quatToRotMat  (const float q[4], float R[3][3]);
    static void  quatToEulerZYX(const float q[4], float e[3]);
    static void  normalizeQuat (float q[4]);
    static float stdAtmPressure(float alt_m);
    static void  llaToNED(float lat_rad, float lon_rad, float alt_m,
                          float refLat,  float refLon,  float refAlt,
                          float ned[3]);
};
