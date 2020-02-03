/******************************************************************************
                         Cyclope-EDU sample code
                            by Samuel Bonnard

  This project is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Cyclope sample code is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Cyclope-edu code. If not, see <http://www.gnu.org/licenses/>.
******************************************************************************/

void mpu6050() {

  if (!dmpReady) return;

  while (!mpuInterrupt && fifoCount < packetSize) {
  }
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
  }

  else if (mpuIntStatus & 0x02) {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;

#ifdef OUTPUT_READABLE_YAWPITCHROLL
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    if (educ == 0) {
      Pitch = ypr[1] * 180 / M_PI * 1000 / 140 + 1650;
      Roll = ypr[2] * 180 / M_PI * 1000 / 140 + 1500;
      Pitch = constrain(Pitch, 1000, 2000);
      Roll = constrain(Roll, 1000, 2000);
    }
    Throttle = map(analogRead(Throttle_P), 890, 0, 1000, 2000);
    Yaw = map(analogRead(Yaw_P), 1023, 0, 1000, 2000);
    Throttle = constrain(Throttle, 1000, 2000);
    Yaw = constrain(Yaw, 1000, 2000);
  }
#endif
}

