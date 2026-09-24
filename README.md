# sensor-fusion
ARCUIA Sensor Fusion

Ryan Breen

9-DOF + GPS sensor fusion for rocket navigation. started in simulink freshman year, later ported to c++ for a teensy 4.1. heads up — this is a sim/prototype project, not integrated onto flight hardware. the boards that actually flew (nov 2025 + may 2026, both recovered) run separate flight electronics. this repo is where I've been learning the filter side of things and working toward getting it onto real hardware.

════════════════════════════════════════════════════════ QUICK NUMBERS ════════════════════════════════════════════════════════

so this isn't just words:

500Hz filter update rate 10Hz gps correction rate 6-state EKF (pos + vel, NED frame) 4 sensors fused (imu, mag, gps, baro model) 20 col CSV telemetry, 500 lines/sec 2 versions (matlab/simulink + c++/teensy) ~1400 lines across the filter, sim, and live HUD

════════════════════════════════════════════════════════ WHY TWO VERSIONS ════════════════════════════════════════════════════════

simulink (psuedocode/) came first — that's where I worked out what sensor feeds what and what units everything needs to be in, before any hardware existed to test on.

the c++ port (teensy/) is the same idea pointed at real sensor bytes instead of matlab workspace vars. written to be ready for hardware, not flown or tested against real sensor noise yet.

keeping both because simulink is still the fastest way to test a change before I'd trust it on anything real.

════════════════════════════════════════════════════════ SENSORS ════════════════════════════════════════════════════════

IMU (accel + gyro) — motion and rotation, 500Hz Magnetometer — corrects heading, needs calibration or heading drifts fast GPS — position and velocity fix, ~10Hz Barometer — altitude, currently modeled from a standard atmosphere calc instead of a real sensor

everything gets timestamped so the fast imu data and the slower gps updates line up correctly.

════════════════════════════════════════════════════════ FILTER ════════════════════════════════════════════════════════

two stages: one figures out which way the rocket is pointed using the imu + magnetometer, the other tracks position and velocity using the imu + gps, correcting itself every time a new gps fix comes in.

honestly, this is the part of the project I understand the least at a deep level, it's where I started actually learning this kind of filter theory rather than something I'd call myself an expert in yet.

════════════════════════════════════════════════════════ VISUALIZATION ════════════════════════════════════════════════════════

psuedocode/visualize.m — turns a simulink run into one HUD screen: 3d trajectory, vel, accel, orientation, ground track, error vs ground truth. has a replay mode w/ adjustable speed.

visualize_live.py — same HUD but live, rewritten in python (pyqt5 + pyqtgraph/opengl). reads the csv off usb serial in a background thread so it doesn't stall. matched the colors to the matlab HUD on purpose so sim output and real output look the same.

════════════════════════════════════════════════════════ HOW TO RUN — SIMULATION (psuedocode/) ════════════════════════════════════════════════════════

RUN TRAJECTORY -> type trajectory in the matlab terminal. builds a ground-truth flight path + sensor signals off it RUN SIMULINK -> open arcsensors.slx, run it against the workspace signals from step 1 RUN VISUALIZE -> runs the HUD in visualize.m

sim defaults to a 10s window, extend it for a full launch profile. no sensor noise modeled yet so it's checking the filter logic against clean ground truth, haven't checked it against real hardware noise.

════════════════════════════════════════════════════════ HOW TO RUN — HARDWARE TARGET (teensy/) ════════════════════════════════════════════════════════

not flown yet, this is the plan once the PCB work is further along

open in vscode w/ platformio
platformio.ini: [env:teensy41] platform = teensy board = teensy41 framework = arduino
in main.cpp: create the filter, feed it imu/mag data on a 500Hz timer, feed it gps data when a fix comes in, and stream the output over usb serial
read the stream with visualize_live.py

════════════════════════════════════════════════════════ OUTPUT ════════════════════════════════════════════════════════

streams position, velocity, orientation, altitude, heading, and airspeed over serial as csv, 500 times a second, for the live HUD.

════════════════════════════════════════════════════════ NOTES ════════════════════════════════════════════════════════

this whole thing is a learning project, not flight proven. real hardware work on ARCUIA so far has mostly been soldering + building the boards that actually flew
units will get you, deg vs rad, ned vs enu. convert explicitly, don't assume
no sensor noise in the sim yet, need to check this against real hardware noise before I'd trust it on anything that flies
next: get the EKF running directly on the actual avionics PCB (with gps), that's the real first step toward flying this filter
