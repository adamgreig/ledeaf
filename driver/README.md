# ledeaf driver

This directory contains the PCB design for the driver board.

## Revision 1: 2021-05

### Errata

* Issue going into USB bootloader possibly because BOOT1 (PB2) is floating,
  maybe add a pull-down resistor.

### Thoughts For Next Rev

* Combine BOOTLOAD and PATTERN buttons into one (wire BOOT0 up to a GPIO)
  to reduce BOM count.
* Investigate cheaper STM32 and oscillator.
* Add light sensor to allow automatically scaling brightness.
