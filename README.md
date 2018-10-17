# Quadrotor
This research is part of a long-term goal of a fleet of cooperating Flexible Autonomous Machines operating in an uncertain Environment (FAME) at Arizona State University.

## Summary
This repository contains source code for Teensy-based flight controller for a quadrotor with GY-89 sensor (L3GD20 and LSM303D).

## Components
| Component | Type |
| --- | --- |
| Frame | GEPRC GEP-KX5 Elegant 250mm |
| Microcontroller | Teensy 3.2 |
| RF Module | Digi XBee3 (XB3-24Z8PT-J) |
| Motors | GEP-GR2207 2400kv |
| IMU | GY-89 (L3GD20 and LSM303D) |
| ESCs | RacerStar 20A Brushless ESC Lite 2-4S MultiShot BLHeli BB2 DSHOT600 |
| Props | Bullnose Prop 5x4.5, Carbon Composite |

## Usage
Quadrotor class in `Quadrotor.h` consists of `Xdes` and `X` state variables. Accessing the sensor data can be done through `X.x`, where small `x` is the variable that you want to access. For example, `X.phi` will give you the roll (phi) angle and so on as shown in Table 1 and 2.


|Table 1|Table 2|
|--|--|
|<table> <tr><th>State</th><th>Description</th></tr><tr><td>`X.phi`</td><td>Roll (phi) angle (rad)</td></tr> <tr><td>`X.theta`</td><td>Pitch (theta) angle (rad)</td></tr> <tr><td>`X.psi`</td><td>Yaw (psi) angle (rad)</td></tr> <tr><td>`X.p`</td><td>Angular rate around x-axis (rad/s)</td></tr> <tr><td>`X.q`</td><td>Angular rate around y-axis (rad/s)</td></tr> <tr><td>`X.r`</td><td>Angular rate around z-axis (rad/s)</td></tr></table>| <table> <tr><th>Controls</th><th>Description</th></tr><tr><td>`U1`</td><td>Thrust (N)</td></tr> <tr><td>`U2`</td><td>Moment around x-axis (Nm)</td></tr> <tr><td>`U3`</td><td>Moment around y-axis (Nm)</td></tr> <tr><td>`U4`</td><td>Moment around z-axis (Nm)</td></tr></table>
