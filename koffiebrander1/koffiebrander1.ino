/*
  Koffiebrander.ino - Firmware for controlling a coffee roaster.
  Created by Rein Velt, 2016.
  Released into the public domain.
*/

#include "Koffiebrander.h"

Koffiebrander koffiebrander=Koffiebrander();

void setup()
{
  koffiebrander.init();;
}

void loop()
{
 koffiebrander.run();
}




