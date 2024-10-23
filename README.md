# vscp-demo-pico-fridge-ctrl
A simple VSCP demo that can be used to control temperature of a fridge using the [W55RP20-EVB-PICO](https://docs.wiznet.io/Product/ioNIC/W55RP20/w55rp20-evb-pico) from [WIZnet](https://docs.wiznet.io).

## MQTT

### Publish topic

/vscp/GUID/class/type/index

### Subscribe topic

/vscp/GUID

Event sent on this topic will be handled by the device.


## Registers

| Register | Description |
| ------ | ----------- |
| 0 | Zone |
| 1 | Subzone |
| 2 | Status <br /> bit 0 - Compressor. |
| 3 | Config <br /> bit 0 - Send alarm on high.<br /> bit 1 - Send alarm on low.<br /> bit 2 - Send alarm on high.<br /> bit 7 - enable if set to one |
| 4 | Period for temperature event in seconds. Set to zero to disable. Default: 60 |
| 5 | Current fridge temperature * 100 MSB |
| 6 | Current fridge temperature * 100 LSB |
| 7 | B coefficient MSB Default: |
| 8 | B coefficient LSB |
| 9 | Temperature * 100 signed offset MSB |
| 10 | Temperature * 100 signed offset LSB |
| 11 | Low alarm temperature * 100 signed int16_t MSB |
| 12 | Low alarm temperature * 100 signed int16_t LSB |
| 13 | High alarm temperature * 100 signed int16_t MSB |
| 14 | High alarm temperature * 100 signed int16_t LSB |
| 15 | Hysteresis, unsigned uint8_t |
| 16 | Settemp, signed uint8_t |


## Remote variables

  * Current temperature float
  * B Coefficient uint16
  * Temperature offset, integer
  * Low alarm temperature, integer
  * High alarm temperature, integer

## Events

  * Temperature 1040/6
  * Alarm