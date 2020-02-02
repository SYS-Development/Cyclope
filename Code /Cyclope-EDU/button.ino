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

int lastPOWER  = 0;

void auxselect() {

  if (digitalRead(POWER_Pin) != lastPOWER) {
    if (digitalRead(POWER_Pin) == 0) {
      if (POWER == 1) POWER = 0;
      else            POWER = 1;
    }
    lastPOWER = digitalRead(POWER_Pin);
  }
  
  if (digitalRead(B_Pin) == 0) { B_and_C = 10; }
   else if (digitalRead(C_Pin) == 0) { B_and_C = 5; }
   else { B_and_C = 0; }

  if (digitalRead(A_Pin) == 0) { A_and_D = 10; }
   else if (digitalRead(D_Pin) == 0) { A_and_D = 5; }
   else { A_and_D = 0; }
  
}

