// w55rp20_fridge_ctrl.h
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version
// 2 of the License, or (at your option) any later version.
//
// This file is part of the VSCP (https://www.vscp.org)
//
// Copyright (C) 2000-2024 Ake Hedman,
// the VSCP project, <info@vscp.org>
//
// This file is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this file see the file COPYING.  If not, write to
// the Free Software Foundation, 59 Temple Place - Suite 330,
// Boston, MA 02111-1307, USA.
//

#ifndef W55RP20_FRIDGE_CTRL
#define W55RP20_FRIDGE_CTRL

/* Clock */
#define PLL_SYS_KHZ (133 * 1000)

/* Buffer */
#define ETHERNET_BUF_MAX_SIZE (1024 * 2)

/* Socket */
#define SOCKET_MQTT 0

/* Port */
#define PORT_MQTT 1883

/* I/O Pins */
#define COMPRESSOR_RELAY_PIN 27
#define NTC_POWER_PIN        28
#define NTC_ADC_PIN          26 // ADC0

/* Timeout */
#define DEFAULT_MQTT_TIMEOUT 1000 // 1 second

/* Fridge temperature read interval (ms)*/
#define FRIDGE_TEMPERATURE_INTERVAL 20000

/* Bits is alarm status for alarm notification */
#define ALARM_LOW_STATUS  1
#define ALARM_HIGH_STATUS 2

/* MQTT */
#define MQTT_CLIENT_ID "w55rp20-fridge"
#define MQTT_USERNAME  "vscp"
#define MQTT_PASSWORD  "secret"
// #define MQTT_PUBLISH_TOPIC   "publish_topic"
// #define MQTT_PUBLISH_PAYLOAD "Hello, World!"
// #define MQTT_PUBLISH_PERIOD  (1000 * 10) // 10 seconds
// #define MQTT_SUBSCRIBE_TOPIC "subscribe_topic"
#define MQTT_KEEP_ALIVE 60 // 60 milliseconds

// VSCP
#define MDF_URL "eurosource.se/w55rp20_frc01.xml"

/* Registers */
#define FRIDGE_REG_ZONE              0
#define FRIDGE_REG_SUBZONE           1
#define FRIDGE_REG_STATUS            2
#define FRIDGE_REG_CONFIG            3
#define FRIDGE_REG_TEMP_EVENT_PERIOD 4
#define FRIDGE_REG_TEMP_MSB          5
#define FRIDGE_REG_TEMP_LSB          6
#define FRIDGE_REG_B_MSB             7
#define FRIDGE_REG_B_LSB             8
#define FRIDGE_REG_LOW_ALARM_MSB     9
#define FRIDGE_REG_LOW_ALARM_LSB     10
#define FRIDGE_REG_HIGH_ALARM_MSB    11
#define FRIDGE_REG_HIGH_ALARM_LSB    12
#define FRIDGE_REG_HYSTERESIS        13
#define FRIDGE_REG_SETTEMP           14

// *** Defaults ***
#define FRIDGE_DEFAULT_ZONE    0
#define FRIDGE_DEFAULT_SUBZONE 0

/* Status register bits */
#define FRIDGE_STATUS_COMPRESSOR 0

/* Config register bits */
#define FRIDGE_CONFIG_ALARM_HIGH 0
#define FRIDGE_CONFIG_ALARM_LOW  1
#define FRIDGE_CONFIG_ENABLE     7

// NTC
#define DEFAULT_NTC_COEFFICIENT      3450
#define DEFAULT_TEMP_REPORT_INTERVAL 60
#define DEFAULT_TEMP_HYSTERESIS      5
#define DEFAULT_TEMP_LOW_ALARM       -25
#define DEFAULT_TEMP_HIGH_ALARM      -10
#define DEFAULT_TEMP_SET             -20

typedef struct fridgectrl {
  uint16_t bCoefficient;
  bool bActive;            /* True if unit is active */
  bool bAlarmOnLow;        /* True if unit sends alarm on low temperature */
  bool bAlarmOnHigh;       /* True if unit sends alarm on high temperature */
  int16_t temp_current;    /* Current temperature */
  int8_t temp_setpoint;    /* Setpoint temperature */
  int16_t temp_alarm_low;  /* Low temperature alarm limit */
  int16_t temp_alarm_high; /* High temperature alarm limit */
  uint8_t hysteresis;
  uint8_t temp_report_period; /* Interval in seconds for temp reports. Zero
                                    is off */
} fridgectrl_t;

#endif