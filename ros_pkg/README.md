# Quadrotor 
This repository contains a source code for Teensy-based flight controller (autopilot) for a quadrotor with MPU-6050 imu sensor (L3GD20 and LSM303D). Also, a ROS package for high-level control is included as well. 

## Components
| Component | Type |
| --- | --- |
| Frame | Mini 250 FPV Quadcopter Frame |
| Microcontroller | Teensy 3.2 |
| RF Module | Digi XBee3 (XB3-24Z8PT-J) |
| Motors and ESCs | DJI Snail 2305 Racing Motor |
| IMU | MPU-6050 |
| Props | DJI 5048S Tri-Blade Propeller |

## Usage
Quadrotor class in `Quadrotor.h` consists of `Xdes` and `X` state variables for desired states and actual states, respectively. Accessing the states data can be done through `X.x`, where small `x` is the variable that you want to access. For example, `X.phi` will give you the roll (phi) angle in units of (rad/s) and so on as shown in Table 1 and 2.


|Table 1|Table 2|
|--|--|
|<table> <tr><th>State</th><th>Description</th></tr><tr><td>`X.phi`</td><td>Roll (phi) angle [rad]</td></tr> <tr><td>`X.theta`</td><td>Pitch (theta) angle [rad]</td></tr> <tr><td>`X.psi`</td><td>Yaw (psi) angle [rad]</td></tr> <tr><td>`X.p`</td><td>Angular rate around x-axis [rad/s]</td></tr> <tr><td>`X.q`</td><td>Angular rate around y-axis [rad/s]</td></tr> <tr><td>`X.r`</td><td>Angular rate around z-axis [rad/s]</td></tr> <tr><td>`X.x`</td><td>position in x-axis [m]</td></tr> <tr><td>`X.y`</td><td>position in y-axis [m]</td></tr> <tr><td>`X.z`</td><td>position in z-axis [m]</td></tr> <tr><td>`X.xdot`</td><td>linear velocity in x-axis [m/s]</td></tr> <tr><td>`X.ydot`</td><td>linear velocity in y-axis [m/s]</td></tr> <tr><td>`X.zdot`</td><td>linear velocity in z-axis [m/s]</td></tr> </table>| <table> <tr><th>Controls</th><th>Description</th></tr><tr><td>`U1`</td><td>Thrust [N]</td></tr> <tr><td>`U2`</td><td>Moment around x-axis [Nm]</td></tr> <tr><td>`U3`</td><td>Moment around y-axis [Nm]</td></tr> <tr><td>`U4`</td><td>Moment around z-axis [Nm]</td></tr></table>

## Control Method 
If `control_method = 1`, then change `Quad.Control(1)` in `Quadrotor.ino` and use the following in `Quadrotor::GenerateMotorCommands(void)`
```
void Quadrotor::GenerateMotorCommands(void)
{
  omega1Squared = 130958.617 * U1.current - 1290232.68 * U2.current + 1290232.68 * U3.current + 1290232.68 * U4.current;
  omega2Squared = 130958.617 * U1.current + 1290232.68 * U2.current - 1290232.68 * U3.current + 1290232.68 * U4.current;
  omega3Squared = 130958.617 * U1.current + 1290232.68 * U2.current + 1290232.68 * U3.current - 1290232.68 * U4.current;
  omega4Squared = 130958.617 * U1.current - 1290232.68 * U2.current - 1290232.68 * U3.current - 1290232.68 * U4.current;
  omega1Squared = Quadrotor::CONSTRAIN(omega1Squared, 0, 900000000);
  omega2Squared = Quadrotor::CONSTRAIN(omega2Squared, 0, 900000000);
  omega3Squared = Quadrotor::CONSTRAIN(omega3Squared, 0, 900000000);
  omega4Squared = Quadrotor::CONSTRAIN(omega4Squared, 0, 900000000);
  omega1 = sqrt(omega1Squared);
  omega2 = sqrt(omega2Squared);
  omega3 = sqrt(omega3Squared);
  omega4 = sqrt(omega4Squared);
```
else `control_method = 2`, then change `Quad.Control(2);` in `Quadrotor.ino` and use the following in `Quadrotor::GenerateMotorCommands(void)`
```
void Quadrotor::GenerateMotorCommands(void)
{
  omega1Squared = 130958.617 * U1.current - 1480900 * U2.current + 1480900 * U3.current + 1290232.68 * U4.current;
  omega2Squared = 130958.617 * U1.current + 1480900 * U2.current - 1480900 * U3.current + 1290232.68 * U4.current;
  omega3Squared = 130958.617 * U1.current + 1480900 * U2.current + 1480900 * U3.current - 1290232.68 * U4.current;
  omega4Squared = 130958.617 * U1.current - 1480900 * U2.current - 1480900 * U3.current - 1290232.68 * U4.current;
  omega1Squared = Quadrotor::CONSTRAIN(omega1Squared, 0, 900000000);
  omega2Squared = Quadrotor::CONSTRAIN(omega2Squared, 0, 900000000);
  omega3Squared = Quadrotor::CONSTRAIN(omega3Squared, 0, 900000000);
  omega4Squared = Quadrotor::CONSTRAIN(omega4Squared, 0, 900000000);
  omega1 = sqrt(omega1Squared);
  omega2 = sqrt(omega2Squared);
  omega3 = sqrt(omega3Squared);
  omega4 = sqrt(omega4Squared);
```
