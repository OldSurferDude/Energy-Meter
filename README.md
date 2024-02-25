# Energy-Meter
# A low cost energy meter
Energy meters cost a lot of money. The goal for this one is under US$30.  In order to do that, though, it’s a DIY project. It's one channel, meaning it will only measure on circuit.

As currently (17 Jan 24) designed, the data is sent to a MySensors gateway.  On the recieving end could be another Arduino Nano, a computer running Ubuntu with a Home Assistant running in a Virtual Box on that computer.

25 February 2024: Version 22 has been running for a week now without problems.
Circuit was modified by inserting a diode between the power supply 5V and the Arduino 5V to prevent current coming from the Arduino going into the power supply.  This happens when the Arduino is powered by the USB connection while programming.  In this situation, without this diode, the power supply will become non-functional.

## Here are some serious safety considerations
Doesn’t matter how cheap it is if it’s not safe.

As a DIYer you will be entirely responsible for your safety, the safety of everyone around you, and all the things around you.

Fuses with holders will cost about US$15.  They won’t do a very good job of protecting a 1 Watt device.  The devices that they would be protecting cost about US$4.  The one thing they would protect against is a high resistance short, one that generates about 200W, which could melt a plastic enclosure.  A catastrophic failure like that is possible, though, highly improbable.  You decide.  I have left them out of the design.

The design is not agency (UL, CES) certified.  If it were found that the device was the source for damage, insurance would be nullified.  The person that installed the device would be responsible.

The power supply looks to be of barely adequate quality.  It is probably the same design as 700mA USB wall warts.  Yes, those burn up when the specs are exceeded or get wet.  As designed, the power supply needs to supply less than 100mA, but bad things can happen in their manufacture and water seems to get everywhere.

Safety note on current transformers.  If there is no burden resistor, the resistance becomes that of air, between 1.5 · 1013 and 6 · 1013 Ω .  V=I2R, which, no matter how little current flowing in the wire being sensed, it’s a lot of volts!  To avoid this from happening, the current transformer burden resistor is **securely** mounted on the current transformer itself.  If a wire from the current transformer breaks, it won’t be a hazard.


**IF YOU BUILD AND INSTALL THIS DEVICE YOU ARE RESPONSIBLE FOR SAFETY**

# Read more [here](https://docs.google.com/document/d/1Qb_S8iBt4_zRzbYbtjIBpqug_ZvvkmUpOjDa-rYTmhk/edit?usp=sharing)

# Note:  This is a work in progress.
The .ino version 14 reports power, energy. Vrms, Irms, VA and power factor.
I've never designed a PCB and this project screems for one.  I could use some help on that.
