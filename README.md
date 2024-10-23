# vscp-demo-pico-fridge-ctrl
A simple VSCP demo that can be used to control temperature of a fridge using the [W55RP20-EVB-PICO](https://docs.wiznet.io/Product/ioNIC/W55RP20/w55rp20-evb-pico) from [WIZnet](https://docs.wiznet.io).

## MQTT

### Publish topic

**Default:** /vscp/GUID/class/type/index

### Subscribe topic

**Default:** /vscp/GUID

Event sent on this topic will be handled by the device.


## Registers

| Offset | Page | Description |
| ------ | ---- | ----------- |
| 0 | 0 | Zone |
| 1 | 0 | Subzone |
| 2 | 0 | Status <br /> bit 0 - Compressor. |
| 3 | 0 | Config <br /> bit 0 - Send alarm on high.<br /> bit 1 - Send alarm on low.<br /> bit 2 - Send alarm on high.<br /> bit 7 - enable |
| 4 | 0 | Period for temperature event. Set to zero to disable. |
| 5 | 0 | Current fridge temperature * 100 MSB |
| 6 | 0 | Current fridge temperature * 100 LSB |
| 7 | 0 | B coefficient MSB |
| 8 | 0 | B coefficient LSB |
| 9 | 0 | Temperature * 100 signed offset MSB |
| 10 | 0 | Temperature * 100 signed offset LSB |
| 11 | 0 | Low alarm temperature * 100 signed MSB |
| 12 | 0 | Low alarm temperature * 100 signed LSB |
| 13 | 0 | High alarm temperature * 100 signed MSB |
| 14 | 0 | High alarm temperature * 100 signed LSB |


## Remote variables

  * Current temperature float
  * B Coefficient uint16
  * Temperature offset, integer
  * Low alarm temperature, integer
  * High alarm temperature, integer

## Events

  * Temperature 1040/6
  * Alarm