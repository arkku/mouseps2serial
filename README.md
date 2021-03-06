# PS/2 to Serial Mouse Converter

A PS/2 mouse to serial converter using an ATmega328p (or similar)
microcontroller. 

The converter reads input from a PS/2 mouse (with support for wheel and 5
buttons), and converts it to a serial mouse protocol. The supported protocols
include Microsoft (the classic PC serial mouse), with optional wheel and third
button support, Mouse Systems, and Sun Microsystems (which is a variant of the
Mouse Systems protocol). Additionally there is a debug mode, which outputs
human-readable text.

The protocol and serial port speed may be configured either by 4 DIP switches,
or hard-coded using compile-time flags. When a "modern" wheel mouse is used,
holding down all buttons and while turning the wheel allows adjusting the mouse
resolution. The speed setting may be persisted across restarts by clicking the
right mouse button while holding down the middle button.

~ [Kimmo Kulovesi](https://arkku.dev/), 2020-03-01

## Hardware

There is a rough [schematic](schematic.pdf) available, but the actual device
I have is currently built on protoboard slightly differently. An
Arduino-compatible board may also be used, as very few additional components
are needed (once you have a logic-level serial port available, for which you
can use a breakout board).

Note that a separate power supply is required, since the PC serial port has no
reliable power source capable of powering both a PS/2 mouse and this converter.
A PC should have plenty of 5 V sources available, however.

### Bill of Materials

* ATmega328p microcontroller
* 22 KΩ resistor (pull-up for microcontroller RST#)
* 14–16 MHz crystal oscillator and capacitors (e.g., 22 pF), or other clock
  source
* 2× 100 Ω resistor (optional, in series on PS/2 lines)
* 2× 4.7–10 KΩ resistor (optional, external pull-up on PS/2 lines)
* 100 nF decoupling capacitor (between V<sub>cc</sub> and GND, near
  microcontroller)
* 10 µF decoupling capacitor (between V<sub>cc</sub> and GND, optional)
* PS/2 female connector
* 2-pin header or connector for power input
* pin header or connector for serial port
* 3-pin header for RTS jumper (optional)
* 4-position DIP switch (optional)
* an LED and its current-limiting resistor (optional)
* MAX232 for serial port level conversion, unless using a logic-level serial
  port (or external converter) already
    - 4× 1 µF capacitor for MAX232 charge pump
    - 100 nF decoupling capacitor
* ISP header for programming (optional)

### Pins

The default pin assignments are as follows:

* PD2/INT0: PS/2 CLK
* PD4: PS/2 DATA
* PD1/TXD: Serial data from mouse to computer
* PD3: Serial RTS from computer (or tie to ground for always active)
* PD0/RXD: Serial data from computer to mouse (optional, tie high or low if
  not connected)
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

PC serial ports use RS-232, which nominally have 12 V levels. This means
that you **must not** connect the RTS or RXD directly to the microcontroller,
or you will fry the pins! However, most PC serial ports will accept _input_ at
5 V levels, so you may be able to connect the TXD pin through as simple
inverting buffer (such as a NOT gate). The other two pins are actually
optional, although mouse auto-detection/recognition will not work without
RTS (this only matters for the Microsoft protocol). For full serial port
support, you can use a logic level to RS-232 converter, such as a MAX232 chip.

The RTS defaults to active low, since the logic level serial port signals are
generally inverted (e.g., by the MAX232). This also means that if you leave it
unconnected, you must jumper the pin to ground or it will be pulled high
(inactive) by the internal pull-up.

While the Microsoft driver pulses RTS to have the mouse identify itself, it
might be that some other drivers pulse DTR instead. In this case, connect the
RTS from the serial port (pin 7 on the DE-9 connector) to DTR instead
(pin 4 on the DE-9 connector).

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
    make FORCE_PROTOCOL=PROTOCOL_MICROSOFT FORCE_BAUD=1200

The protocol after `FORCE_PROTOCOL` is one of:

* `PROTOCOL_MICROSOFT`
* `PROTOCOL_MICROSOFT_WHEEL`
* `PROTOCOL_MOUSE_SYSTEMS`
* `PROTOCOL_SUN`
* `PROTOCOL_DEBUG`

If only one of the two settings (`FORCE_PROTOCOL` and `FORCE_BAUD`) is given,
the other one remains configurable via DIP switches. This allows installing
one or two jumpers instead of 4 switches, for example.

The RTS line is used to view the state of the serial port. With serial serial
mice, the line is used to supply power to the mouse, and hence the mouse works
only when the line is supplying the voltage. However, if you use a logic level
serial port with this converter, the state is typically inverted, which is why
the default is active low. This may be changed with the setting
`SERIAL_STATE_INVERTED`. You may configure it at compile time:

    make clean
    make SERIAL_STATE_INVERTED=0

Note that the internal pull-up on the RTS pin means the default state is high,
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
to recover from errors by resetting the device automatically after a few
seconds if something goes wrong. A reset may also be forced by sending an
exclamation mark (`!`) over the serial port.

The indicator LED (if installed) is lit when a PS/2 mouse is connected and
blinks on activity.

If the mouse driver does not recognize the mouse, it is probably because it
expects to read an id character on RTS pulse, and the pulse isn't working (not
connected, wrong polarity). If using the Microsoft wheel mouse protocol, the id
character may not be recognized by the driver (try the regular Microsoft
protocol to verify).

In case of problems, also try the Mouse Systems protocol when possible, since
it has no handshake requirement (the RTS can simply be jumpered permanently).

Also try the debug mode at 9600 bps on the same serial port and view the data
in a terminal program. Send a question mark `?` over the serial line to query
the status if you see no movement. The status update shows the PS/2 mouse id
(if it is N/A, the PS/2 mouse was not correctly recognized), as well as the RTS
and DIP configuration status. (The DIP switches take effect only on reset, so
you may test the correct operation of the switches in debug mode.)

If you are using the serial mouse in DOS, also try booting without any extended
or expanded memory (no `HIMEM.SYS` or `EMM386.EXE` or equivalent). Also try
a different mouse driver.

## Speed Adjustment

Since some of the newer PS/2 mouse have considerably higher resolution than
classic serial mice, it is possible to reduce the resolution on the fly. This
happens by simultaneously holding down all three main buttons (left, middle,
right) and turning the wheel. Wheel up slows down the mouse (increases the
divisor) and wheel down speeds it up.

The setting can be persisted across restarts by pressing and then releasing
the right button while holding down the middle button. 

Obviously this requires the mouse to have a wheel, but I figure pre-wheel mice
are probably not too fast in the first place. However, the divisor may also
be set by sending the corresponding single-digit number over the serial port.
It is also possible to define it at compile-time as follows:

    make clean
    make MOUSE_DIVISOR=2

This compile-time setting disables the loading and saving of the setting, but
still allows on-the-fly adjustment with the wheel.

The setting `MOUSE_SCALING=1` can be used to enable scaling (kind of mouse
acceleration) in the PS/2 mouse itself. Using the wheel or serial port input
method to set the divisor to 0 also enables scaling with no divisor (i.e., the
same as divisor 1 but with in-mouse scaling).
