This is an alternative driver program for the Nixie Tube Clock NCS314.
It uses the one-wire temperature sensor to set the color of the base LEDs, 
and uses the DS1307 for timekeeping. It hard-codes 24-hour time, Y-M-D 
date format, and Farenheit temperature display. Finally, it uses a 
different "shuffle" method to switch between modes, which I personally 
find more pleasing.

For the original, see https://github.com/afch/NixeTubesShieldNCS314

I import the OneWire library straight into the sketch. The 
Debounce and other code is mostly my own.
