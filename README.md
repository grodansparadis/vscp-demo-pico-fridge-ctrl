# vscp-demo-pico-fridge-ctrl

![](https://github.com/grodansparadis/vscp-demo-pico-fridge-ctrl/blob/main/kicad/schema.png)

A VERY simple (VSCP)[https://vscp.org] demo that can be used to control temperature of a fridge using the [W55RP20-EVB-PICO](https://docs.wiznet.io/Product/ioNIC/W55RP20/w55rp20-evb-pico) from [WIZnet](https://docs.wiznet.io) using the MQTT protocol as a transport mechanism.

The firmware is really simple. It has a setpoint (including a hysteresis setting) and a low and high alarm. Temperature and alarms are sent as VSCP events to a MQTT sever of choice.

The hardware is ethernet connected and uses a NTC thermistor for temperature readings and then current temperature values is calculated using the Steinhart–Hart equation. There are some good information on thermistors [here](https://en.wikipedia.org/wiki/Thermistor).

There is a dumb sister project [here]() that use no communication and connection to the world, just a hard setpoint and a LCD display.

## MQTT

### Publish topic

  /vscp/GUID/class/type/index

GUID is deduced from the MAC address of the device (default: FF:FF:FF:FF:FF:FF:FF:FE:00:08:DC:12:34:56:00:01) and class, type is deduced from the events sent. 

So if one want to subscribe to all temperature events from the fridge controller one subscribe to

  vscp/FF:FF:FF:FF:FF:FF:FF:FE:00:08:DC:12:34:56:00:01/1040/6/#

For alarms one subscribe to

  vscp/FF:FF:FF:FF:FF:FF:FF:FE:00:08:DC:12:34:56:00:01/1/2/#

To subscribe to all events from the device subscribe to

  vscp/FF:FF:FF:FF:FF:FF:FF:FE:00:08:DC:12:34:56:00:01/#

### Subscribe topic

  /vscp/GUID

Event sent on this topic will be handled by the device. Read/write registers ca be used to edit or check the configuration among other things.  


## Registers

As all VSCP devices the fridge controller have register that defines it's functionality. The controller is build as a VSCP level II device and therefore only have one page of registers. There is currently no decision matrix defined.

| Register | Description |
| ------ | ----------- |
| 0 | Zone |
| 1 | Subzone |
| 2 | Status <br /> bit 0 - Compressor on=1/off=0. |
| 3 | Config <br /> bit 0 - Send alarm on high.<br /> bit 1 - Send alarm on low.<br />  bit 7 - enable if set to one |
| 4 | Period for temperature event in seconds. Set to zero to disable. Default: 60 |
| 5 | Current fridge temperature * 100 MSB |
| 6 | Current fridge temperature * 100 LSB |
| 7 | B coefficient MSB Default: |
| 8 | B coefficient LSB |
| 9 | Low alarm temperature * 100 int16 MSB |
| 10 | Low alarm temperature * 100 int16 LSB |
| 11 | High alarm temperature * 100 int16 MSB |
| 12 | High alarm temperature * 100 int16 LSB |
| 13 | Hysteresis, uint8 |
| 14 | Settemp, int8 |


## Remote variables

  * Current temperature int16
  * B Coefficient uint16
  * Low alarm temperature, int16
  * High alarm temperature, int16

## Events

  * Temperature - class=10/type=6
  * Alarm - class=1/type=2

## MDF (Module description file)

The module description file can be downloaded from (here)[https://www.eurosource.se/w55rp20_frc01.xml]


![](./IMG_20241028_211113.jpg)
![](./IMG_20241108_161251.jpg)
![](./IMG_20241108_161251.jpg)
![](./IMG_20241108_161251.jpg)
![](./IMG_20241108_161251.jpg)
