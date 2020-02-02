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


/*
 * Grâce à cette partie du code vous pourrez personnaliser la télécommande Cyclope et créer 
 * vous même votre script. Attention néanmoins à ne pas utiliser de fonction "delay()"
 * ce qui pourrait bloquer le protocole d'emission. 
 */




void eduselect(){
  
  receiver = 1; // Afin d'utiliser la télécommande avec le récepteur Cyclope-RX, mettez la valeur ci-dessous (receiver) à "0".
  educ = 0; // Afin d'utiliser la partie éducative et donc de désactiver la partie Acceleromètre, mettre la valeur educ à 1.
  
}

void edu() {
  
  ///////////////////////Example///////////////////////
  if (digitalRead(A_Pin) == 0) { //Lorsque le bouton A est appuyé le drone s'incline a droite
    Roll -= 5; // Plus la valeur 5 sera augmentée, plus le véhicule s'inclinera rapidement et inversement
  }
  if (digitalRead(B_Pin) == 0) { //Lorsque le bouton B est appuyé le drone s'incline a gauche
    Roll += 5; // Plus la valeur 5 sera augmentée, plus le véhicule s'inclinera rapidement et inversement
  }



  if (digitalRead(C_Pin) == 0) { //Lorsque le bouton C est appuyé le drone s'incline en avant
    Pitch += 5; // Plus la valeur 5 sera augmentée, plus le véhicule avancera rapidement et inversement
  }
  if (digitalRead(D_Pin) == 0) { //Lorsque le bouton D est appuyé le drone s'incline en arrière
    Pitch -= 5; // Plus la valeur 5 sera augmentée, plus le véhicule reculera rapidement et inversement
  }

  // Cette partie permet de remettre les valeurs de Pitch et Roll par default:
  
  if (digitalRead(A_Pin) == 1 & digitalRead(B_Pin) == 1) { 
    Roll = 1500;
  }
  if (digitalRead(C_Pin) == 1 & digitalRead(D_Pin) == 1) {
    Pitch = 1500;
  }
   //////////////////////////////////////////////////////
}
