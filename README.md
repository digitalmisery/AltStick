AltStick
========

![Screenshot](https://raw.github.com/digitalmisery/AltStick/master/AltStick.jpeg)

Design files (open-source software and hardware) for the Miniature Rocket Altimeter (AltStick) Project.  

The Hardware folder contains the PCB files created in Eagle

The Software folder contains:
- Board and Programmer definition files for the Arduino IDE to target the AltStick
- Arduino (1.0+) example files
  - Output barometric pressure and temperature via USB keyboard - open a text editor, press the button, and it starts typing
  - Height measurement - zeros out height and then starts sampling height change until button is pressed again - flashes out height (feet) on LED
  - LED cycling - tests out power consumption using sleep modes and watchdog timer
- Bootloader code based on [USBaspLoader](http://www.obdev.at/products/vusb/usbasploader.html) (GitHub fork [here](http://github.com/baerwolf/USBaspLoader)).
  - No external programmer or hardware needed to program Arduino sketches once bootloader is programmed
  - Bootloader activated by holding down button and initiating a reset
  - Bootloader active and USB activity LEDs 
- Arduino libraries
  - PinChangeInt for generating an interrupt on button press during sleep modes
  - UsbKeyboard for using V-USB to emulate a USB keyboard for typing out measurements
- Windows driver for [USBasp](http://www.fischl.de/usbasp/)

Additional info here: http://www.digitalmisery.com/projects/altstick
