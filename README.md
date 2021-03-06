# Analog-MIDI-Merge
Runs on Arduino Nano and other ATmega328 based devices. Generates MIDI messages from values of analog pins, and merges these with MIDI messages received on digital pin D8, and transmits the merged data to digital pin D9. Latency is currently <10ms.

### Please check out the [Technical Wiki](https://github.com/jmsmdy/Analog-MIDI-Merge/wiki) for more details about what's going on under the hood.

## Hardware

This software is designed to work with custom hardware according to the following schematic:

![Schematic](SCHv1.0.png)

The following gives a compact PCB layout for this circuit:
![PCB](PCBv1.0.png)

A detailed hardware guide with some revisions to the hardware for this project is forthcoming.

## Details and Usage

This project makes use of Paul Stroffregen's AltSoftSerial library, which uses hardware-specific tricks to reduce latency of software serial writes and reads. If you want to run this on another device besides the Nano, you should consult the documentation for AltSoftSerial to verify compatibility. The number of MIDI inputs and outputs is limited. It's possible to extend this code to have up to two MIDI inputs and two MIDI outputs simultaneously by using both AltSoftSerial (using pins D8 and D9 on the Nano) and the hardware serial interface (using pins RX and TX), at the cost of losing the ability to use the USB port for serial at the same time. If you want to merge two external MIDI signals (as opposed to just merging internally generated MIDI signals with an external MIDI signal), this code can be easily modified to to handle it: just duplicate the code handling the message buffers, and filter out the realtime messages from one input (otherwise timing clock signals from one device may interfere with the other). Some handy references for MIDI messages are avaiable here: https://www.midi.org/specifications/item/table-1-summary-of-midi-message and here: http://midi.teragonaudio.com/tech/midispec/run.htm.

Analog inputs are handled in a low-level way by writing and reading directly to the ADC registers, which is the only way to ensure the analog reads don't interfere with the very stringent timing requirements of the AltSoftSerial library. The ADC is set to continuous read mode, and interrupts are used to allow other code to run while the ADC makes it readings. Information about non-blocking analog reads can be found here: https://meettechniek.info/embedded/arduino-analog.html.

Information is processed with four buffers: an raw input buffer, a non-realtime message buffer, a realtime message buffer, and a raw output buffer. The input buffer is populated with raw data from the MIDI input. The non-realtime message buffer stores messages like Note On and Note Off in a structured format which is easy to read, modify, and add to. The realtime buffer stores time-critical messages like timing clicks. These buffers are populated by parsing the raw data from the input buffer. The output buffer is then populated using the non-realtime and realtime message buffers. To add new MIDI messages to the existing stream is as easy as `messageBuffer.put(your_midi_message);`

If you change some of the formatting of the data printed to the Serial port, you can also use it with Hairless MIDI<->Serial Bridge to connect a MIDI keyboard and some analog inputs (such as pedals, sliders, knobs) to DAW software on a computer. The Arduino Nano has 8 analog inputs, and other compatible boards like the Teesny have even more, if you require a lot of analog inputs.

The code should be pretty easy to modify. I've commented areas where it is easy to add your own custom code. Just beware not to do anything that blocks for an extended preiod of time, or you risk dropping MIDI messages.
