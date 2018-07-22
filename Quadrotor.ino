// Today July 21
#include "Quadrotor.h"

Quadrotor Quad;

void setup()
{
  Teensy();
  delay(200);
  Quad.MotorInit();
  delay(200);
  Quad.SensorInit();
  delay(200);
  Quad.FilterInit();
  delay(200);
 // Check the position of the reciver before starting
 while (Quad.channel.CH3 < 990 || Quad.channel.CH3 > 1020 || Quad.channel.CH4 < 1400)
 {
   digitalWrite(LEDRed, LOW);
   delay(200);
   digitalWrite(LEDRed, LOW);
   delay(200);
 }
   digitalWrite(LEDRed, LOW);
   Quad.loop_timer = micros();
}


void loop()
{
  Quad.ArmingState();
  Quad.BatteryVoltageCheck();
  Quad.Estimation(1);
  Quad.AttitudeControl();
  Quad.AltitudeControl();
  Quad.GenerateMotorCommands();

  Serial.print(Quad.Xdes.psi);
  Serial.print(" , ");
  Serial.print(Quad.X.psi);
  Serial.print(" , ");
  Serial.print(Quad.channel.CH3);
  Serial.print(" , ");
  Serial.print(Quad.PWM2);
  Serial.print(" , ");
  Serial.print(Quad.PWM3);
  Serial.print(" , ");
  Serial.println(Quad.PWM4);
  
  
  while (micros() - Quad.loop_timer < dtMicroseconds);
  //Reset the zero timer.
  Quad.loop_timer = micros();
}

void Teensy()
{
  Serial.begin(115200);
  pinMode(LEDRed, OUTPUT);
  pinMode(LEDGreen, OUTPUT);
  digitalWrite(LEDGreen, LOW);
  digitalWrite(LEDRed, LOW);
  delay(5);
  pinMode(CHANNEL1, INPUT);
  pinMode(CHANNEL2, INPUT);
  pinMode(CHANNEL3, INPUT);
  pinMode(CHANNEL4, INPUT);
  attachInterrupt(CHANNEL1, Receiver, CHANGE);
  attachInterrupt(CHANNEL2, Receiver, CHANGE);
  attachInterrupt(CHANNEL3, Receiver, CHANGE);
  attachInterrupt(CHANNEL4, Receiver, CHANGE);
  delay(50);
}

void Receiver()
{
  if (Quad.lastChannel1 == 0 && digitalReadFast(CHANNEL1) == HIGH)
   {
     Quad.lastChannel1 = 1;
     Quad.receiverChannelTimer1 = micros();
   }
   if (Quad.lastChannel1 == 1 && digitalReadFast(CHANNEL1) == LOW)
   {
     Quad.lastChannel1 = 0;
     Quad.channel.CH1 = micros() - Quad.receiverChannelTimer1;
   }

   if (Quad.lastChannel2 == 0 && digitalReadFast(CHANNEL2) == HIGH)
    {
      Quad.lastChannel2 = 1;
      Quad.receiverChannelTimer2 = micros();
    }
    if (Quad.lastChannel2 == 1 && digitalReadFast(CHANNEL2) == LOW)
    {
      Quad.lastChannel2 = 0;
      Quad.channel.CH2 = micros() - Quad.receiverChannelTimer2;
    }

    if (Quad.lastChannel3 == 0 && digitalReadFast(CHANNEL3) == HIGH)
     {
       Quad.lastChannel3 = 1;
       Quad.receiverChannelTimer3 = micros();
     }
     if (Quad.lastChannel3 == 1 && digitalReadFast(CHANNEL3) == LOW)
     {
       Quad.lastChannel3 = 0;
       Quad.channel.CH3 = micros() - Quad.receiverChannelTimer3;
     }

     if (Quad.lastChannel4 == 0 && digitalReadFast(CHANNEL4) == HIGH)
      {
        Quad.lastChannel4 = 1;
        Quad.receiverChannelTimer4 = micros();
      }
      if (Quad.lastChannel4 == 1 && digitalReadFast(CHANNEL4) == LOW)
      {
        Quad.lastChannel4 = 0;
        Quad.channel.CH4 = micros() - Quad.receiverChannelTimer4;
      }
}
