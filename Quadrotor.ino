// Author: Abdullah Altawaitan
// Description: Teensy 3.2 code
#include "BaseStation.h"


Quadrotor Quad;
BaseStation Vive;
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
  Vive.HTCVive(&Quad);
  Vive.HTCViveComplementaryFilter(&Quad);
  Quad.Receiver();
  Quad.Control(1);
  Quad.DifferentialFlatness();
  Quad.AttitudeControl();
  Quad.GenerateMotorCommands();
  Quad.XbeeZigbeeSend();
  Quad.LoopCounter();

}

void Teensy()
{
  Serial.begin(230400);
  Serial1.begin(230400);
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
  // Vive.HTCVive(&Quad, SerialMAV);
  // Vive.HTCViveComplementaryFilter(&Quad);
}
