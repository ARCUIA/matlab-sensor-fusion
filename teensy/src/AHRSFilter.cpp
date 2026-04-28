/*
 * AHRSFilter.cpp
 * ARCUIA Sensor Fusion — Teensy 4.x
 */

#include "AHRSFilter.h"

// ─── Constants ───────────────────────────────────────────────────────────────
static constexpr float DEG2RAD  = 0.01745329251f;   // pi/180
static constexpr float RAD2DEG  = 57.2957795131f;   // 180/pi
static constexpr float G_NED    = 9.81f;            // gravity [m/s²], positive down in NED
static constexpr float R_EARTH  = 6378137.0f;       // WGS84 equatorial radius [m]

// ─── Constructor ─────────────────────────────────────────────────────────────
AHRSFilter::AHRSFilter(float beta)
    : _beta(beta), _refLat(0), _refLon(0), _refAlt(0), _refSet(false)
{
    // Identity quaternion (level, heading north)
    _q[0] = 1.0f;  _q[1] = 0.0f;  _q[2] = 0.0f;  _q[3] = 0.0f;

    // Zero navigation state
    for (int i = 0; i < 6; ++i) _x[i] = 0.0f;

    // Initial covariance — large uncertainty on position, moderate on velocity
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 6; ++j)
            _cov[i][j] = 0.0f;
    _cov[0][0] = _cov[1][1] = _cov[2][2] = 100.0f;  // position uncertainty [m²]
    _cov[3][3] = _cov[4][4] = _cov[5][5] = 10.0f;   // velocity uncertainty [(m/s)²]

    memset(&out, 0, sizeof(out));
    out.orient[0] = 1.0f;
}

// ─── init ─────────────────────────────────────────────────────────────────────
void AHRSFilter::init(float refLat_deg, float refLon_deg, float refAlt_m) {
    _refLat = refLat_deg * DEG2RAD;
    _refLon = refLon_deg * DEG2RAD;
    _refAlt = refAlt_m;
    _refSet = true;
}

// ─── update (500 Hz) ──────────────────────────────────────────────────────────
void AHRSFilter::update(const float accel[3], const float gyro[3],
                         const float mag[3],   float dt)
{
    madgwickUpdate(accel, gyro, mag, dt);
    navPredict(accel, dt);
    computeOutputs();
}

// ─── feedGPS (~10 Hz) ─────────────────────────────────────────────────────────
void AHRSFilter::feedGPS(const float lla[3], const float velNED[3]) {
    if (!_refSet) {
        // Auto-initialize reference on first GPS fix
        init(lla[0], lla[1], lla[2]);
        return;
    }

    float ned[3];
    llaToNED(lla[0] * DEG2RAD, lla[1] * DEG2RAD, lla[2],
             _refLat, _refLon, _refAlt, ned);
    navCorrectGPS(ned, velNED);
}

// ─── Madgwick AHRS ────────────────────────────────────────────────────────────
// Gradient-descent algorithm fusing accelerometer + gyroscope + magnetometer.
// Reference: Madgwick, S.O.H. (2010), "An efficient orientation filter for
//            inertial and inertial/magnetic sensor arrays"
//
// Quaternion convention: q = [w, x, y, z]  (scalar first)
// Frame: body → NED (earth)
void AHRSFilter::madgwickUpdate(const float a[3], const float g[3],
                                  const float m[3], float dt)
{
    float q0 = _q[0], q1 = _q[1], q2 = _q[2], q3 = _q[3];

    // ── Step 1: Gyro integration — rate of change of quaternion ──────────────
    //    q_dot = 0.5 * q ⊗ [0, gx, gy, gz]
    float qDot0 = 0.5f * (-q1*g[0] - q2*g[1] - q3*g[2]);
    float qDot1 = 0.5f * ( q0*g[0] + q2*g[2] - q3*g[1]);
    float qDot2 = 0.5f * ( q0*g[1] - q1*g[2] + q3*g[0]);
    float qDot3 = 0.5f * ( q0*g[2] + q1*g[1] - q2*g[0]);

    // ── Step 2: Gradient correction from accelerometer + magnetometer ─────────
    float aN = sqrtf(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
    float mN = sqrtf(m[0]*m[0] + m[1]*m[1] + m[2]*m[2]);

    if (aN > 0.0f && mN > 0.0f) {
        // Normalize
        float ax = a[0]/aN,  ay = a[1]/aN,  az = a[2]/aN;
        float mx = m[0]/mN,  my = m[1]/mN,  mz = m[2]/mN;

        // Pre-computed products to reduce flops
        float _2q0 = 2.0f*q0, _2q1 = 2.0f*q1, _2q2 = 2.0f*q2, _2q3 = 2.0f*q3;
        float q0q0=q0*q0, q0q1=q0*q1, q0q2=q0*q2, q0q3=q0*q3;
        float q1q1=q1*q1, q1q2=q1*q2, q1q3=q1*q3;
        float q2q2=q2*q2, q2q3=q2*q3;
        float q3q3=q3*q3;

        // Magnetic field reference direction in NED (flatten to horizontal plane)
        float hx = mx*(q0q0+q1q1-q2q2-q3q3) + 2.0f*my*(q1q2-q0q3) + 2.0f*mz*(q1q3+q0q2);
        float hy = 2.0f*mx*(q1q2+q0q3) + my*(q0q0-q1q1+q2q2-q3q3) + 2.0f*mz*(q2q3-q0q1);
        float _2bx = sqrtf(hx*hx + hy*hy);
        float _2bz = -2.0f*mx*(q1q3-q0q2) + 2.0f*my*(q2q3+q0q1)
                     + mz*(q0q0-q1q1-q2q2+q3q3);
        float _4bx = 2.0f*_2bx;
        float _4bz = 2.0f*_2bz;

        // Objective function errors (expected vs measured)
        // Gravity direction (NED: g points down = [0,0,1] in earth frame)
        float f1 = 2.0f*(q1q3 - q0q2)            - ax;
        float f2 = 2.0f*(q0q1 + q2q3)            - ay;
        float f3 = 1.0f - 2.0f*(q1q1 + q2q2)     - az;
        // Magnetic field direction
        float f4 = _2bx*(0.5f-q2q2-q3q3) + _2bz*(q1q3-q0q2)       - mx;
        float f5 = _2bx*(q1q2-q0q3)       + _2bz*(q0q1+q2q3)       - my;
        float f6 = _2bx*(q0q2+q1q3)       + _2bz*(0.5f-q1q1-q2q2)  - mz;

        // Jacobian transpose × error (gradient of objective function)
        float s0 = -_2q2*f1 + _2q1*f2
                   - _2bz*q2*f4 + (-_2bx*q3+_2bz*q1)*f5 + _2bx*q2*f6;
        float s1 =  _2q3*f1 + _2q0*f2 - 4.0f*q1*f3
                   + _2bz*q3*f4 + (_2bx*q2+_2bz*q0)*f5 + (_2bx*q3-_4bz*q1)*f6;
        float s2 = -_2q0*f1 + _2q3*f2 - 4.0f*q2*f3
                   + (-_4bx*q2-_2bz*q0)*f4 + (_2bx*q1+_2bz*q3)*f5 + (_2bx*q0-_4bz*q2)*f6;
        float s3 =  _2q1*f1 + _2q2*f2
                   + (-_4bx*q3+_2bz*q1)*f4 + (-_2bx*q0+_2bz*q2)*f5 + _2bx*q1*f6;

        // Normalize gradient and subtract from gyro rate
        float sN = sqrtf(s0*s0 + s1*s1 + s2*s2 + s3*s3);
        if (sN > 0.0f) {
            float inv = _beta / sN;
            qDot0 -= s0 * inv;
            qDot1 -= s1 * inv;
            qDot2 -= s2 * inv;
            qDot3 -= s3 * inv;
        }
    }

    // ── Step 3: Integrate and normalize ──────────────────────────────────────
    _q[0] = q0 + qDot0 * dt;
    _q[1] = q1 + qDot1 * dt;
    _q[2] = q2 + qDot2 * dt;
    _q[3] = q3 + qDot3 * dt;
    normalizeQuat(_q);
}

// ─── EKF Predict (500 Hz) ─────────────────────────────────────────────────────
// State: x = [px, py, pz, vx, vy, vz]  (NED frame, meters / m/s)
//
// Process model:
//   p_dot = v
//   v_dot = R_bn * a_body + [0, 0, g]     (rotate body accel to NED, add gravity)
//
// Covariance propagation uses the constant-velocity Jacobian:
//   F = [I3, dt*I3]
//       [03,    I3 ]
void AHRSFilter::navPredict(const float accel[3], float dt) {
    // Rotation matrix: body frame → NED
    float R[3][3];
    quatToRotMat(_q, R);

    // Specific force in NED + gravity  (gives true acceleration)
    float aN[3];
    for (int i = 0; i < 3; ++i)
        aN[i] = R[i][0]*accel[0] + R[i][1]*accel[1] + R[i][2]*accel[2];
    aN[2] += G_NED;   // NED down is positive, so add +g to Down component

    // State propagation
    for (int i = 0; i < 3; ++i) {
        _x[i]   += _x[i+3]*dt + 0.5f*aN[i]*dt*dt;  // p += v*dt + 0.5*a*dt²
        _x[i+3] += aN[i]*dt;                         // v += a*dt
    }

    // Covariance propagation  P = F*P*F^T + Q
    // Cross terms: P_pv += P_vv * dt,  P_pp += (P_pv + P_vp)*dt + P_vv*dt²
    static constexpr float Q_VEL = 0.05f;    // velocity process noise variance  [(m/s)²/s]
    static constexpr float Q_POS = 0.001f;   // position process noise variance  [m²/s]

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            _cov[i][j]     += (_cov[i+3][j] + _cov[i][j+3])*dt + _cov[i+3][j+3]*dt*dt;
            _cov[i][j+3]   += _cov[i+3][j+3]*dt;
            _cov[i+3][j]   += _cov[i+3][j+3]*dt;
        }
        _cov[i][i]     += Q_POS * dt;
        _cov[i+3][i+3] += Q_VEL * dt;
    }
}

// ─── EKF GPS Correction ───────────────────────────────────────────────────────
// Sequential scalar updates (no 6×6 matrix inversion needed)
// Measurement model: z = H*x,  H = I6  (GPS directly measures pos + vel)
void AHRSFilter::navCorrectGPS(const float posNED[3], const float velNED[3]) {
    // GPS measurement noise variances
    static constexpr float R_POS_H = 2.56f;   // horizontal [m²]  (1.6 m CEP)²
    static constexpr float R_POS_V = 9.00f;   // vertical   [m²]  (3.0 m)²
    static constexpr float R_VEL   = 0.01f;   // velocity   [(m/s)²]

    float meas[6] = {posNED[0], posNED[1], posNED[2],
                     velNED[0], velNED[1], velNED[2]};
    float R[6]    = {R_POS_H, R_POS_H, R_POS_V, R_VEL, R_VEL, R_VEL};

    // For each measurement component, run one scalar Kalman update
    for (int k = 0; k < 6; ++k) {
        float innov = meas[k] - _x[k];          // innovation
        float S     = _cov[k][k] + R[k];        // innovation variance
        float Sinv  = 1.0f / S;

        // Kalman gain  K = P[:,k] / S
        float K[6];
        for (int i = 0; i < 6; ++i) K[i] = _cov[i][k] * Sinv;

        // State update
        for (int i = 0; i < 6; ++i) _x[i] += K[i] * innov;

        // Covariance update  P = (I - K*H)*P  where H = e_k (unit row vector)
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j)
                _cov[i][j] -= K[i] * _cov[k][j];
    }
}

// ─── computeOutputs ───────────────────────────────────────────────────────────
// Derives all output variables from the filter state after each 500 Hz update.
void AHRSFilter::computeOutputs() {

    // pos / vel / orient — direct state extraction
    for (int i = 0; i < 3; ++i) { out.pos[i] = _x[i];   out.vel[i] = _x[i+3]; }
    for (int i = 0; i < 4; ++i)   out.orient[i] = _q[i];

    // ── Euler ZYX  →  [yaw, pitch, roll] in radians ──────────────────────────
    quatToEulerZYX(_q, out.euler);

    // ── heading  =  yaw in degrees, wrapped 0–360 ────────────────────────────
    out.heading = fmodf(out.euler[0] * RAD2DEG + 360.0f, 360.0f);

    // ── rollPitch  [pitch_deg, roll_deg]  (euler[1]=pitch, euler[2]=roll) ─────
    out.rollPitch[0] = out.euler[1] * RAD2DEG;   // pitch
    out.rollPitch[1] = out.euler[2] * RAD2DEG;   // roll

    // ── altitude [ft]  =  -NED_down * 3.28084 ────────────────────────────────
    out.altitude  = _x[2] * -3.28084f;

    // ── climb rate [ft/min]  =  -vel_down * 196.85 ───────────────────────────
    out.climbRate = _x[5] * -196.85f;

    // ── airspeed [knots]  =  |vel| * 1.944 ───────────────────────────────────
    float vx = _x[3], vy = _x[4], vz = _x[5];
    out.airspeed = sqrtf(vx*vx + vy*vy + vz*vz) * 1.944f;

    // ── barometric pressure [Pa] — ICAO 1976 Standard Atmosphere ─────────────
    //    true altitude [m] = -NED_down (NED Down is negative upward)
    out.baroPressure = stdAtmPressure(-_x[2]);
}

// ─── printCSV ─────────────────────────────────────────────────────────────────
// Outputs one line per call over USB Serial.
// Column order: px,py,pz,vx,vy,vz,qw,qx,qy,qz,alt,clb,spd,hdg,pitch,roll,yaw_r,pitch_r,roll_r,baro
void AHRSFilter::printCSV() const {
    Serial.printf("%.4f,%.4f,%.4f,"           // pos NED
                  "%.4f,%.4f,%.4f,"           // vel NED
                  "%.6f,%.6f,%.6f,%.6f,"      // quaternion
                  "%.2f,%.2f,%.2f,%.2f,"      // alt, clb, spd, hdg
                  "%.3f,%.3f,"                // pitch_deg, roll_deg
                  "%.5f,%.5f,%.5f,"           // yaw_rad, pitch_rad, roll_rad
                  "%.2f\n",                   // baro Pa
        out.pos[0], out.pos[1], out.pos[2],
        out.vel[0], out.vel[1], out.vel[2],
        out.orient[0], out.orient[1], out.orient[2], out.orient[3],
        out.altitude, out.climbRate, out.airspeed, out.heading,
        out.rollPitch[0], out.rollPitch[1],
        out.euler[0], out.euler[1], out.euler[2],
        out.baroPressure);
}

// ─────────────────────────────────────────────────────────────────────────────
//  MATH HELPERS
// ─────────────────────────────────────────────────────────────────────────────

// Quaternion [w,x,y,z] → rotation matrix (body → NED)
//   R such that v_NED = R * v_body
void AHRSFilter::quatToRotMat(const float q[4], float R[3][3]) {
    float w=q[0], x=q[1], y=q[2], z=q[3];
    R[0][0]=1-2*(y*y+z*z);  R[0][1]=2*(x*y-w*z);    R[0][2]=2*(x*z+w*y);
    R[1][0]=2*(x*y+w*z);    R[1][1]=1-2*(x*x+z*z);  R[1][2]=2*(y*z-w*x);
    R[2][0]=2*(x*z-w*y);    R[2][1]=2*(y*z+w*x);    R[2][2]=1-2*(x*x+y*y);
}

// Quaternion [w,x,y,z] → Euler ZYX (aerospace convention)
//   e[0] = yaw   (ψ)  — rotation about NED Down
//   e[1] = pitch (θ)  — rotation about East
//   e[2] = roll  (φ)  — rotation about North/body-x
void AHRSFilter::quatToEulerZYX(const float q[4], float e[3]) {
    float w=q[0], x=q[1], y=q[2], z=q[3];
    // yaw
    e[0] = atan2f(2.0f*(w*z + x*y), 1.0f - 2.0f*(y*y + z*z));
    // pitch — clamp sin argument to [-1, 1] to avoid NaN at singularity
    float sinp = 2.0f*(w*y - z*x);
    sinp = sinp >  1.0f ?  1.0f : (sinp < -1.0f ? -1.0f : sinp);
    e[1] = asinf(sinp);
    // roll
    e[2] = atan2f(2.0f*(w*x + y*z), 1.0f - 2.0f*(x*x + y*y));
}

// Normalize quaternion in place (guards against drift accumulation)
void AHRSFilter::normalizeQuat(float q[4]) {
    float n = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (n > 1e-6f) {
        float inv = 1.0f / n;
        q[0] *= inv;  q[1] *= inv;  q[2] *= inv;  q[3] *= inv;
    }
}

// ICAO 1976 Standard Atmosphere pressure [Pa] given geometric altitude [m]
//   Troposphere  (0 – 11 000 m): T decreases linearly at 6.5 K/km
//   Tropopause   (11 000 – 20 000 m): isothermal at 216.65 K
//   Stratosphere (20 000 – 32 000 m): T increases at 1 K/km
float AHRSFilter::stdAtmPressure(float h) {
    if (h < 0.0f)       h = 0.0f;
    if (h <= 11000.0f) {
        // P = 101325 * (1 − 2.25577×10⁻⁵ · h)^5.25588
        float t = 1.0f - 2.25577e-5f * h;
        return 101325.0f * powf(t, 5.25588f);
    } else if (h <= 20000.0f) {
        // P = 22632.1 * exp(−1.57688×10⁻⁴ · (h − 11000))
        return 22632.1f * expf(-1.57688e-4f * (h - 11000.0f));
    } else {
        // P = 5474.89 * (1 + 4.61574×10⁻⁶ · (h − 20000))^(−34.1632)
        return 5474.89f * powf(1.0f + 4.61574e-6f * (h - 20000.0f), -34.1632f);
    }
}

// Flat-Earth LLA → NED (valid within ~100 km of reference)
//   lat/lon in radians, alt in meters
//   ned[0] = North [m],  ned[1] = East [m],  ned[2] = Down [m]
void AHRSFilter::llaToNED(float lat, float lon, float alt,
                            float refLat, float refLon, float refAlt,
                            float ned[3])
{
    ned[0] = (lat - refLat) * R_EARTH;
    ned[1] = (lon - refLon) * R_EARTH * cosf(refLat);
    ned[2] = -(alt - refAlt);
}
