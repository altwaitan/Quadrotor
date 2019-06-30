// Author: Abdullah Altawaitan
// Description: Teensy 3.2 code
// Check the Following:
// 1. The voltage readings
// 2. The Baud rate
// 3. The receiver channels
// 4. The control mode
#include "Communication.h"


Quadrotor Quad;
Communication Vive;
SBUS Radiolink(Serial2);

void setup()
{
  Teensy();
  Quad.MotorInit();
  Quad.SensorInit();
  Quad.FilterInit();
  digitalWrite(LEDRed, LOW);
  digitalWrite(LEDGreen, HIGH);
  Serial.println("Now Ready!");
  Quad.loop_timer = micros();
}


void loop()
{
  // outerCounter for 100Hz
  Quad.outerCounter++;
  Receiver();
  Quad.ArmingState();
  Quad.BatteryVoltageCheck();
  Quad.Estimation(1);
  Vive.ROS_Send(&Quad);
  Quad.Receiver();
  Quad.Control(2);
  Quad.DifferentialFlatness();
  Quad.AttitudeControl();
  Quad.GenerateMotorCommands();
  Quad.LoopCounter();
}

void Teensy()
{
  Serial.begin(115200);
  Serial1.begin(115200);
  Radiolink.begin();
  pinMode(LEDRed, OUTPUT);
  pinMode(LEDGreen, OUTPUT);
  digitalWrite(LEDGreen, LOW);
  digitalWrite(LEDRed, LOW);
  Quad.channel.CH1 = 1000;
  Quad.channel.CH2 = 1000;
  Quad.channel.CH3 = 250;
  Quad.channel.CH4 = 1000;
  Quad.channel.CH5 = 250;
  Quad.channel.CH6 = 250;
  Quad.QuadrotorState = 0;
}

void Receiver()
{
  if (Radiolink.read(Quad.channels, &Quad.failSafe, &Quad.lostFrames))
  {
    Quad.channel.CH1 = Quad.channels[0];
    Quad.channel.CH2 = Quad.channels[1];
    Quad.channel.CH3 = Quad.channels[2];
    Quad.channel.CH4 = Quad.channels[3];
    Quad.channel.CH5 = Quad.channels[4];
    Quad.channel.CH6 = Quad.channels[5];
    Quad.channel.CH7 = Quad.channels[6];
    Quad.channel.CH8 = Quad.channels[7];
    Quad.channel.CH9 = Quad.channels[8];
    Quad.channel.CH10 = Quad.channels[9];
  }
}

void serialEvent1()
{
  Vive.ROS_Receive(&Quad);
}
