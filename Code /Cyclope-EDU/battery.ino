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

void battery() {
  unsigned long currentMillis = millis();

  V_Batt = ((analogRead(Batt_P) * 3.3) / 1024.0) / 0.5;
  
  /////////// WARNING ///////////
  if (V_Batt <= 3.75) {
    digitalWrite(ledPinR, HIGH);
  }
  ///////////////////////////////
}

