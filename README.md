# SMS-remote-control-of-power-outlets-ESP32
remote control of 3 relays by SMS using an ESP32 and a GSM800 module; relays are used to turn power outlets (230V) on and off; BME280 to get temperature/humidity to launch e.g. an anti-freeze program with a heater on power outlet 1

I was interested in a portable box with 2 independently controlled power outlets/wall sockets to be used on my boat. One idea was to be able to turn a heating system on by SMS before I arrived at my boat ...

This code is based on information and code from George Bantique, TechToTinker Youtube channel (https://techtotinker.blogspot.com). Many thanks to George for a this - it gave me a rather quick start into my project!

The aim of this project is to allow a remote control of 2 (or more) power outlets (230 V) and maybe later also a 12 V by SMS. A BME280 can be used to call for temperature, humidity and air pressure data, can be used to send warning SMS, or can be used to start an anti-freeze routine if a heating system is attached to line 1. Both power outlet 1 and 2 can be programmed by SMS for different interval schemes. 
In addition, the 12 V line can be used to control the voltage and send a warning SMS if voltage drops e.g. below 12.7 V indicating that charging of batteries is failing. For this, the box has a 12 V plug that can be either plugged into an independent 12 V system (to be watched and used as power supply for the ESP32/GSM800 etc.) or can be plugged into the inbuilt 12 V power outlet of the remote control box.

Many more items could be thought of to further expand the scope of the box...
