# PS/2 to Serial Mouse Converter

A PS/2 mouse to serial converter using an ATmega328p (or similar)
microcontroller. An Arduino with that microcontroller may also be used, but
this converter uses no Arduino libraries.

The converter reads input from a PS/2 mouse (with support for wheel and 5
buttons), and converts it to a serial mouse protocol. The supported protocols
include Microsoft (the classic PC serial mouse), with optional wheel and third
button support, Mouse Systems, and Sun Microsystems (which is a variant of the
Mouse Systems protocol used). Additionally there is a debug mode, which outputs
human-readable text.

The protocol and serial port speed may be configured either by 4 DIP switches,
or hard-coded using compile-time flags.

~ [Kimmo Kulovesi](https://arkku.dev/), 2020-03-01

## Hardware

The default pin assignments are as follows:

* PD2/INT0: PS/2 CLK
* PD4: PS/2 DATA
* PD0/RXD: Serial output (from computer to mouse, optional)
* PD1/TXD: Serial input (to computer)
* PD3: Serial DTR from computer (or tie to ground for always active)
* PB5/SCK: Indicator LED (optional)
* PC2: DIP switch 1 (optional)
* PC3: DIP switch 2 (optional)
* PC4: DIP switch 3 (optional)
* PC5: DIP switch 4 (optional)

The PS/2 CLK and PS/2 DATA lines use the microcontroller's built-in pull-up
resistors, but those are relatively weak. You may consider adding an external
pull-up resistor (e.g. 4.7K to 10K ohm) between each pin and the 5 V supply.
I also use 100 ohm resistors in series with both of the PS/2 lines to limit
current in case of shorts with poorly-designed devices (e.g., many DIY PS/2
device emulators actively drive the output high, which will cause a short when
the host pulls the CLK line low).

The DIP switches use internal pull-ups so they default high. The other side of
the switch should therefore be connected to ground. This means that the "on"
position of each switch actually corresponds to a low level on the pin, which
means that the default is all switches "off". Therefore if you do not install
the DIP switches, the default configuration is Microsoft protocol at 1200 bps,
which is the classic PC serial mouse.

(Obviously it is possible to use pin headers and jumpers instead of DIP
switches, but switches are ideal.)

PC serial ports use RS-232, which nominally have 12 V logic levels. This means
that you **must not** connect the DTR or RXD directly to the microcontroller,
or you will fry the pins! However, most PC serial ports will accept _input_ at
5 V levels, so you may connect the TXD pin. The other two pins are actually
optional, although mouse auto-detection/recognition will not work without
DTR. For full serial port support, you can use a logic level to RS-232
converter, such as a MAX232 chip.

The DTR defaults to active low, which means that if you do not connect it to an
actual serial port DTR output, you should physically connect the pin to ground.

If the indicator LED is installed, it obviously needs a current-limiting
resistor in series.

## Configuration

The configuration may be changed by the 4 DIP switches (if installed):

     1 2 3 4  Setting
     0 0 x x  1200 bps
     0 1 x x  2400 bps (except debug mode, see below)
     1 0 x x  4800 bps
     1 1 x x  9600 bps
     x x 0 0  Microsoft protocol (7N2)
     x x 0 1  Microsoft protocol with wheel (7N2)
     x x 1 0  Mouse Systems protocol (8N1)
     x 0 1 1  Sun Microsystems protocol (8N1)
     0 1 1 1  Debug output (8N1, compile-time determined baud rate)
     1 1 1 1  Debug output (8N1, 9600 bps)

Note that in debug mode configuration, the 2400 bps rate configuration
actually selects a compile-time determined bps, which defaults to 115200 bps
(the same as the Arduino bootloader, which is convenient).

Alternatively, the settings may be hard-coded at compile time as follows:

    make clean
    make FORCE_PROTOCOL=1 FORCE_BAUD=1200

The number after `FORCE_PROTOCOL` is one of:

    0 Microsoft protocol
    1 Microsoft protocol with wheel
    2 Mouse Systems protocol
    3 Sun Microsystems protocol
    4 Debug output

If only one of the two settings (`FORCE_PROTOCOL` and `FORCE_BAUD`) is given,
the other one remains configurable via DIP switches. This allows installing
one or two jumpers instead of 4 switches, for example.

The DTR line is used to view the state of the serial port. With serial serial
mice, the line is used to supply power to the mouse, and hence the mouse works
only when the line is supplying the voltage. However, if you use a logic level
serial port with this converter, the state is typically inverted, which is why
the default is active low. This may be changed with the setting
`SERIAL_STATE_INVERTED`. You may configure it at compile time:

    make clean
    make SERIAL_STATE_INVERTED=0

Note that the internal pull-up on the DTR pin means the default state is high,
i.e., not active when inverted and active when inverted. With the default,
inverted, setting you should physically tie the pin to ground if it is unused,
but with the non-inverted setting it may be simply left unconnected.

## Installation

Build with `make` (requires `avr-gcc`):

    make

You can upload the resulting `ps2serial.hex` using any method of your
choosing. To use a bootloader (e.g., Arduino) you can upload with `avrdude` as
follows:

    make upload PORT=/dev/ttyUSB0 BPS=115200

To "burn" using an external programmer, such the AVR Dragon, you can use:

    make burn BURNER=dragon_isp

The fuses should be set to low `BF`, high `C7`, extended `FD`:

    make fuses BURNER=dragon_pp

However, the fuse values aren't really critical. Also note that these settings
enable clock output on pin PB0/CLKO. Arduino users can probably just make do
with the default fuse settings.

## Operation

Once configured, operation is pretty much automatic. The watchdog timer is used
to recover from errors by resetting the device automatically after 4 seconds
if something goes wrong. A reset may also be forced by sending an exclamation
mark (`!`) over the serial port.

The indicator LED (if installed) is lit when a PS/2 mouse is connected and
blinks on activity.
