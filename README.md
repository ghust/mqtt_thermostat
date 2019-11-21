# ESP8266 MQTT Thermostat
http://ihormelnyk.com/mqtt_thermostat

Simple MQTT OpenTherm Thermostat based on [OpenTherm Adapter](http://ihormelnyk.com/opentherm_adapter), [OpenTherm Library](http://ihormelnyk.com/opentherm_library), ESP8266 (WeMos D1 Mini) and PID Temperature Controller.

Differences between this and Ihor's code:

- Unique MQTT ID on every reconnect.
- You can use any value you want for the current temp (by publishing on the mqtt_currtemp  channel)
- Because of the previous point, I'm not using the 3 wire sensor for anything (hm, maybe I should have it publish somewhere just in case... :) )
- Debugging to a separate channel (where you can see the variables used in the PID function)


TODO: 

- More logical use of the MQTT topics. Use one topic and then add children dynamically
