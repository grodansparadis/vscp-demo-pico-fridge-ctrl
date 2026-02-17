// w55rp20_fridge_ctrl.c
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version
// 2 of the License, or (at your option) any later version.
//
// This file is part of the VSCP (https://www.vscp.org)
//
// Copyright (C) 2000-2026 Ake Hedman,
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

// clang-format off
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "hardware/i2c.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/watchdog.h"
#include "pico/binary_info.h"
#include "pico/bootrom.h"

#include "port_common.h"

#include "wizchip_conf.h"
#include "w5x00_spi.h"

#include "MQTTClient.h"
#include "mqtt_interface.h"

#include "timer/timer.h"

#include <vscp-class.h>
#include <vscp-type.h>
#include "vscp-firmware-helper.h"
#include "vscp-firmware-level2.h"

#include "w55rp20_fridge_ctrl.h"
// clang-format on

// LCD funcs
void
lcd_init(void);

void
i2c_write_byte(uint8_t val);
void
lcd_toggle_enable(uint8_t val);
void
lcd_send_byte(uint8_t val, int mode);
void
lcd_clear(void);
void
lcd_char(char val);
void
lcd_string(const char *s);
void
lcd_set_cursor(int line, int position);

/* Network */
static wiz_NetInfo g_net_info = {
  .mac  = { 0x00, 0x08, 0xDC, 0x12, 0x34, 0x56 }, // MAC address
  .ip   = { 192, 168, 1, 243 },                   // IP address
  .sn   = { 255, 255, 255, 0 },                   // Subnet Mask
  .gw   = { 192, 168, 1, 1 },                     // Gateway
  .dns  = { 192, 168, 1, 1 },                     // DNS server
  .dhcp = NETINFO_STATIC                          // DHCP enable/disable
};

/* Fridge configuration parameters */
fridgectrl_t gdevcfg = { .bCoefficient       = 0xf68,
                         .bActive            = true,
                         .bAlarmOnLow        = true,
                         .bAlarmOnHigh       = true,
                         .temp_current       = 0,
                         .temp_setpoint      = -20,
                         .temp_alarm_low     = -25,
                         .temp_alarm_high    = -10,
                         .hysteresis         = 5,
                         .temp_report_period = 60 };

/* VSCP configuration parameters */
vscp_frmw2_firmware_config_t gvscpcfg = { .m_level     = VSCP_LEVEL2,
                                          .m_puserdata = (void *) &gdevcfg,

                                          .m_probe_timeout       = VSCP_PROBE_TIMEOUT,
                                          .m_probe_timeout_count = VSCP_PROBE_TIMEOUT_COUNT,

                                          .m_interval_heartbeat = 30000,
                                          .m_interval_caps      = 240000,

                                          .m_pDm         = NULL, // Pointer to decision matrix storage (NULL if no DM).
                                          .m_nDmRows     = 0,    // Number of DM rows (0 if no DM).
                                          .m_sizeDmRow   = 0,    // Size for one DM row.
                                          .m_regOffsetDm = 0,    // Register offset for DM (normally zero)
                                          .m_pageDm      = 0,    // Register page where DM definition starts

                                          .m_pInternalMdf = NULL, // No internal MDF

                                          .m_bInterestedInAllEvents = true,
                                          .m_pEventsOfInterest      = NULL, // All events

                                          .m_guid[15] = 0xff, // Ethernet prefix
                                          .m_guid[14] = 0xff,
                                          .m_guid[13] = 0xff,
                                          .m_guid[12] = 0xff,
                                          .m_guid[11] = 0xff,
                                          .m_guid[10] = 0xff,
                                          .m_guid[9]  = 0xff,
                                          .m_guid[8]  = 0xf3,

                                          /*.m_guid[7] = g_net_info.mac[0],
                                          .m_guid[6] = g_net_info.mac[1],
                                          .m_guid[5] = g_net_info.mac[2],
                                          .m_guid[4] = g_net_info.mac[3],
                                          .m_guid[3] = g_net_info.mac[4],
                                          .m_guid[2] = g_net_info.mac[5],
                                          */
                                          .m_guid[1] = 0x00,
                                          .m_guid[0] = 0x01 };

/*
  Topic is "vscp/GUID/class/type/index" and the base "vscp/GUID/"is set here. The topic base
  is used by both subscribe and publish
    max: vscp/00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00/32767/65535/255 (length: 68)
  set to ethernet prefix + mac-addr (from g_net_info above) + 00:01
*/
static char g_mqtt_pub_topic_base[55] = { "vscp/FF:FF:FF:FF:FF:FF:FF:FE:00:08:DC:12:34:56:00:01" };

/* MQTT */
static uint8_t g_mqtt_send_buf[ETHERNET_BUF_MAX_SIZE] = {
  0,
};
static uint8_t g_mqtt_recv_buf[ETHERNET_BUF_MAX_SIZE] = {
  0,
};
static uint8_t g_mqtt_broker_ip[4] = { 192, 168, 1, 7 };
static Network g_mqtt_network;
static MQTTClient g_mqtt_client;
static MQTTPacket_connectData g_mqtt_packet_connect_data = MQTTPacket_connectData_initializer;

static MQTTMessage g_mqtt_message;

/* Timer  */
static volatile uint32_t g_msec_cnt = 0;

/* Clock */
static void
set_clock_khz(void);

/* MQTT */
static void
message_arrived(MessageData *msg_data);

/* Timer  */
static void
timer_callback(void);
static time_t
millis(void);

/* Read fridge temperature */
static int16_t
readFridgeTemperature(void);

static int
init_vscp(vscp_frmw2_firmware_config_t *pcfg);

int
main()
{
  /* Initialize */
  int rv                             = 0;
  uint32_t fridge_temp_read_start_ms = 0; // Interval for fridge temperature read
  uint32_t heart_beat_start_ms       = 0; // Timer for heart beats
  uint32_t periodic_start_ms         = 0; // Timer for periodic temperature events

  g_mqtt_packet_connect_data.username.cstring       = "vscp";
  g_mqtt_packet_connect_data.username.lenstring.len = 4;
  g_mqtt_packet_connect_data.password.cstring       = "secret";
  g_mqtt_packet_connect_data.password.lenstring.len = 6;

  set_clock_khz();
  stdio_init_all();

  if (watchdog_caused_reboot()) {
    printf("Rebooted by Watchdog!\n");
  }
  else {
    printf("Clean boot\n");
  }

  // Init compressor pin
  gpio_init(COMPRESSOR_RELAY_PIN);
  gpio_set_dir(COMPRESSOR_RELAY_PIN, GPIO_OUT);
  gpio_set_pulls(COMPRESSOR_RELAY_PIN, true, false);
  gpio_put(COMPRESSOR_RELAY_PIN, false);

  // Init NTC temperature sensor power pin
  gpio_init(NTC_POWER_PIN);
  gpio_set_pulls(NTC_POWER_PIN, true, false);
  gpio_put(NTC_POWER_PIN, false);
  gpio_set_dir(NTC_POWER_PIN, GPIO_OUT);
  gpio_put(NTC_POWER_PIN, false);

  // Enable the watchdog,
  // second arg is pause on debug which means the watchdog will pause when stepping through code
  // watchdog_enable(1000, 1);

  adc_init();

  // LCD

  // This example will use I2C0 on the default SDA and SCL pins (4, 5 on a Pico)
  i2c_init(i2c_default, 1 * 1000);
  gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
  gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

  // Make the I2C pins available to picotool
  bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

  lcd_init();

  rv = init_vscp(&gvscpcfg);
  rv = vscp_frmw2_init(&gvscpcfg);

  printf("Initialized\n");

  // Make sure GPIO is high-impedance, no pullups etc
  adc_gpio_init(26);

  // Select ADC input 0 (GPIO26)
  adc_select_input(0);

  wizchip_spi_initialize();
  wizchip_cris_initialize();

  wizchip_reset();
  wizchip_initialize();
  wizchip_check();

  // Create one millisecond callback
  wizchip_1ms_timer_initialize(timer_callback);

  // Initialize network
  network_initialize(g_net_info);

  /* Get network information */
  print_network_information(g_net_info);

  NewNetwork(&g_mqtt_network, SOCKET_MQTT);

  rv = ConnectNetwork(&g_mqtt_network, g_mqtt_broker_ip, PORT_MQTT);

  if (rv != 1) {
    printf(" Network connect failed\n");

    while (1)
      ;
  }

  /* Initialize MQTT client */
  MQTTClientInit(&g_mqtt_client,
                 &g_mqtt_network,
                 DEFAULT_MQTT_TIMEOUT,
                 g_mqtt_send_buf,
                 ETHERNET_BUF_MAX_SIZE,
                 g_mqtt_recv_buf,
                 ETHERNET_BUF_MAX_SIZE);

  /* Connect to the MQTT broker */
  g_mqtt_packet_connect_data.MQTTVersion       = 3;
  g_mqtt_packet_connect_data.cleansession      = 1;
  g_mqtt_packet_connect_data.willFlag          = 0;
  g_mqtt_packet_connect_data.keepAliveInterval = MQTT_KEEP_ALIVE;
  g_mqtt_packet_connect_data.clientID.cstring  = MQTT_CLIENT_ID;
  g_mqtt_packet_connect_data.username.cstring  = MQTT_USERNAME;
  g_mqtt_packet_connect_data.password.cstring  = MQTT_PASSWORD;

  rv = MQTTConnect(&g_mqtt_client, &g_mqtt_packet_connect_data);

  if (rv < 0) {
    printf(" MQTT connect failed : %d\n", rv);

    while (1)
      ;
  }

  printf(" MQTT connected\n");

  /* Configure publish message */
  g_mqtt_message.qos      = QOS0;
  g_mqtt_message.retained = 0;
  g_mqtt_message.dup      = 0;
  // g_mqtt_message.payload    = MQTT_PUBLISH_PAYLOAD;
  // g_mqtt_message.payloadlen = strlen(g_mqtt_message.payload);

  /* Subscribe */
  rv = MQTTSubscribe(&g_mqtt_client, g_mqtt_pub_topic_base, QOS0, message_arrived);

  if (rv < 0) {
    printf(" Subscribe failed : %d\n", rv);

    while (1)
      ;
  }

  printf(" Subscribed to topic %s\n", g_mqtt_pub_topic_base);

  heart_beat_start_ms       = 0;
  periodic_start_ms         = millis();
  fridge_temp_read_start_ms = millis();

  /* Work loop */
  while (1) {

    // watchdog_update();

    if ((rv = MQTTYield(&g_mqtt_client, g_mqtt_packet_connect_data.keepAliveInterval)) < 0) {
      printf(" Yield error : %d\n", rv);

      while (1)
        ;
    }

    rv = vscp_frmw2_work(NULL);

    // Temperature measurement
    if (millis() > (fridge_temp_read_start_ms + FRIDGE_TEMPERATURE_INTERVAL)) {

      char buf[20];

      gdevcfg.temp_current      = readFridgeTemperature();
      fridge_temp_read_start_ms = millis();

      lcd_clear();
      lcd_set_cursor(0, 0);
      sprintf(buf, "Temp: %.01f C", gdevcfg.temp_current / 100.0);
      lcd_string(buf);

      lcd_set_cursor(1, 0);
      if (gpio_get(COMPRESSOR_RELAY_PIN)) {
        sprintf(buf, "Compressor: ON");
      }
      else {
        sprintf(buf, "Compressor: OFF");
      }
      lcd_string(buf);
    }

    // Control fridge compressor
    if (gdevcfg.temp_current > (gdevcfg.temp_setpoint + gdevcfg.hysteresis)) {

      vscpEventEx ex;
      memset(&ex, 0, sizeof(ex));

      if (!gpio_get(COMPRESSOR_RELAY_PIN)) {

        // Turn on compressor
        gpio_put(COMPRESSOR_RELAY_PIN, true);

        ex.head = 0;
        memcpy(ex.GUID, gvscpcfg.m_guid, 16);
        ex.timestamp  = vscp_frmw2_callback_get_timestamp(NULL);
        ex.vscp_class = VSCP_CLASS1_INFORMATION;
        ex.vscp_type  = VSCP_TYPE_INFORMATION_ON;
        ex.sizeData   = 3;
        ex.data[0]    = 0; // Index
        ex.data[1]    = gvscpcfg.m_zone;
        ex.data[2]    = gvscpcfg.m_subzone;

        vscp_frmw2_callback_send_event_ex(NULL, &ex);
      }
    }
    else if (gdevcfg.temp_current < (gdevcfg.temp_setpoint)) {
      vscpEventEx ex;
      memset(&ex, 0, sizeof(ex));

      if (gpio_get(COMPRESSOR_RELAY_PIN)) {

        // Turn off compressor
        gpio_put(COMPRESSOR_RELAY_PIN, false);

        ex.head = 0;
        memcpy(ex.GUID, gvscpcfg.m_guid, 16);
        ex.timestamp  = vscp_frmw2_callback_get_timestamp(NULL);
        ex.vscp_class = VSCP_CLASS1_INFORMATION;
        ex.vscp_type  = VSCP_TYPE_INFORMATION_OFF;
        ex.sizeData   = 3;
        ex.data[0]    = 0; // Index
        ex.data[1]    = gvscpcfg.m_zone;
        ex.data[2]    = gvscpcfg.m_subzone;

        vscp_frmw2_callback_send_event_ex(NULL, &ex);
      }
    }

    // Heartbeat
    if (gvscpcfg.m_interval_heartbeat && (millis() > (heart_beat_start_ms + gvscpcfg.m_interval_heartbeat))) {

      /* Publish */
      vscpEventEx ex;
      memset(&ex, 0, sizeof(ex));

      ex.head = 0;
      memcpy(ex.GUID, gvscpcfg.m_guid, 16);
      ex.timestamp  = vscp_frmw2_callback_get_timestamp(NULL);
      ex.vscp_class = VSCP_CLASS1_INFORMATION;
      ex.vscp_type  = VSCP_TYPE_INFORMATION_NODE_HEARTBEAT;
      ex.sizeData   = 3;
      ex.data[0]    = 0;
      ex.data[1]    = gvscpcfg.m_zone;
      ex.data[2]    = gvscpcfg.m_subzone;

      // rv = vscp_frmw2_callback_send_event_ex(NULL, &ex);

      // printf(" Published heartbeat\n");
      heart_beat_start_ms = millis();
    }

    // Periodic temperature measurements
    if (gdevcfg.temp_report_period && (millis() > (periodic_start_ms + (gdevcfg.temp_report_period * 1000)))) {

      vscpEventEx ex;
      memset(&ex, 0, sizeof(ex));

      // Set low alarm bit
      gvscpcfg.m_alarm_status |= 1;

      printf("Temp: %d C\n", gdevcfg.temp_current);

      ex.head = 0;
      memcpy(ex.GUID, gvscpcfg.m_guid, 16);
      ex.timestamp  = vscp_frmw2_callback_get_timestamp(NULL);
      ex.vscp_class = VSCP_CLASS1_MEASUREMENT;
      ex.vscp_type  = VSCP_TYPE_MEASUREMENT_TEMPERATURE;
      ex.sizeData   = 4;
      ex.data[0]    = 0b10001000;                         // Integer | Celsius | Sensor index = 0
      ex.data[1]    = 0x82;                               // Decimal point to steps to the left
      ex.data[2]    = (gdevcfg.temp_current >> 8) & 0xff; // MSB
      ex.data[3]    = gdevcfg.temp_current & 0xff;        // LSB

      vscp_frmw2_callback_send_event_ex(NULL, &ex);

      printf(" Published temperature\n");
      periodic_start_ms = millis();
    }

    // Low temp alarm
    if (gdevcfg.bAlarmOnLow && !(gvscpcfg.m_alarm_status & ALARM_LOW_STATUS) &&
        (gdevcfg.temp_current < gdevcfg.temp_alarm_low)) {
      vscpEventEx ex;
      memset(&ex, 0, sizeof(ex));

      // Set low alarm bit (reset by read of standard alarm register)
      gvscpcfg.m_alarm_status |= ALARM_LOW_STATUS;

      ex.head = 0;
      memcpy(ex.GUID, gvscpcfg.m_guid, 16);
      ex.timestamp  = vscp_frmw2_callback_get_timestamp(NULL);
      ex.vscp_class = VSCP_CLASS1_ALARM;
      ex.vscp_type  = VSCP_TYPE_ALARM_ALARM;
      ex.sizeData   = 3;
      ex.data[0]    = gvscpcfg.m_alarm_status; // Alarm status (standard register 0x80)
      ex.data[1]    = gvscpcfg.m_zone;         // Zone
      ex.data[2]    = gvscpcfg.m_subzone;      // Subzone

      vscp_frmw2_callback_send_event_ex(NULL, &ex);
    }

    // High temp alarm
    if (gdevcfg.bAlarmOnHigh && !(gvscpcfg.m_alarm_status & ALARM_HIGH_STATUS) &&
        (gdevcfg.temp_current > gdevcfg.temp_alarm_high)) {
      vscpEventEx ex;

      // Set high alarm bit (reset by read of standard alarm register)
      gvscpcfg.m_alarm_status |= ALARM_HIGH_STATUS;

      memset(&ex, 0, sizeof(ex));
      ex.head = 0;
      memcpy(ex.GUID, gvscpcfg.m_guid, 16);
      ex.timestamp  = vscp_frmw2_callback_get_timestamp(NULL);
      ex.vscp_class = VSCP_CLASS1_ALARM;
      ex.vscp_type  = VSCP_TYPE_ALARM_ALARM;
      ex.sizeData   = 3;
      ex.data[0]    = gvscpcfg.m_alarm_status; // Alarm status (standard register 0x80)
      ex.data[1]    = gvscpcfg.m_zone;         // Zone
      ex.data[2]    = gvscpcfg.m_subzone;      // Subzone

      vscp_frmw2_callback_send_event_ex(NULL, &ex);
    }

  } // loop
}

/**
 * ----------------------------------------------------------------------------------------------------
 * Functions
 * ----------------------------------------------------------------------------------------------------
 */

///////////////////////////////////////////////////////////////////////////////
// set_sys_clock_khz
//

static void
set_clock_khz(void)
{
  // set a system clock frequency in khz
  set_sys_clock_khz(PLL_SYS_KHZ, true);

  // configure the specified clock
  clock_configure(clk_peri,
                  0,                                                // No glitchless mux
                  CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS, // System PLL on AUX mux
                  PLL_SYS_KHZ * 1000,                               // Input frequency
                  PLL_SYS_KHZ * 1000                                // Output (must be same as no divider)
  );
}

///////////////////////////////////////////////////////////////////////////////
// message_arrived
//

static void
message_arrived(MessageData *msg_data)
{
  int rv;
  vscpEventEx ex;
  memset(&ex, 0, sizeof(vscpEventEx));

  MQTTMessage *message = msg_data->message;
  printf("MQTT message received: %.*s \n", (uint32_t) message->payloadlen, (uint8_t *) message->payload);

  rv = vscp_fwhlp_parse_json_ex(&ex, message->payload);
  if (VSCP_ERROR_SUCCESS != rv) {
    printf("Failed to parse message %d", rv);
    return;
  }

  printf("VSCP Class=%d, VSCP Type = %d\n", ex.vscp_class, ex.vscp_type);

  rv = vscp_frmw2_work(&ex);
  if (VSCP_ERROR_SUCCESS != rv) {
    printf("Failed to send event to protocol woork loop. %d", rv);
  }
}

///////////////////////////////////////////////////////////////////////////////
// timer_callback
//

static void
timer_callback(void)
{
  g_msec_cnt++;
  MilliTimer_Handler();
}

///////////////////////////////////////////////////////////////////////////////
// millis
//

static time_t
millis(void)
{
  return g_msec_cnt;
}

///////////////////////////////////////////////////////////////////////////////
//                        app/fridge functionality
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// readFridgeTemperature
//

static int16_t
readFridgeTemperature(void)
{
  // Turn power on to NTC sensor
  gpio_put(NTC_POWER_PIN, true);

  // Select ADC input (0 (GPIO26))
  adc_select_input(0);

  sleep_ms(500);

  // 12-bit conversion, assume max value == ADC_VREF == 3.3 V
  const float conversion_factor = 3.3f / (1 << 12);
  uint16_t result               = adc_read();
  printf("Raw value: 0x%03x, voltage: %f V\n", result, result * conversion_factor);

  // Use B-constant
  // ==============
  // http://en.wikipedia.org/wiki/Thermistor
  // R1 = (R2V - R2V2) / V2  R2= 10K, V = 3.3V,  V2 = adc * voltage/4094
  // T = B / ln(r/Rinf)
  // Rinf = R0 e (-B/T0), R0=10K, T0 = 273.15 + 25 = 298.15

  // uint16_t B        = 3450;
  double calVoltage = 3.3;

  double Rinf = 10000.0 * exp(gdevcfg.bCoefficient / -298.15);

  // V2 = adc * voltage/4096
  double v = calVoltage * (double) result / 4096;

  // R1 = (R2V - R2V2) / V2  R2= 10K, V = 5V,  V2 = adc * voltage/1024
  double resistance = (10000.0 * (calVoltage - v)) / v;

  // itemp = r;
  double temp = ((double) gdevcfg.bCoefficient) / log(resistance / Rinf);
  // itemp = log(r/Rinf);
  temp -= 273.15; // Convert Kelvin to Celsius

  // average = testadc;
  /*  https://learn.adafruit.com/thermistor/using-a-thermistor
  average = (1023/average) - 1;
  average = 10000 / average;      // Resistance of thermistor
  //temp = average/10000;           // (R/Ro)
  temp = 10000/average;
  temp = log(tempaverage               // ln(R/Ro)
  temp /= B;                      // 1/B * ln(R/Ro)
  temp += 1.0 / (25 + 273.15);    // + (1/To)
  temp = 1.0 / temp;              // Invert
  temp -= 273.15;
  */
  uint16_t current_temp = (long) (temp * 100);
  printf("Temperature: %f C  %d\n", temp, current_temp);

  // Turn power of to the NTC sensor
  gpio_put(NTC_POWER_PIN, false);

  return current_temp;
}

///////////////////////////////////////////////////////////////////////////////
//                               VSCP
///////////////////////////////////////////////////////////////////////////////

static int
init_vscp(vscp_frmw2_firmware_config_t *pcfg)
{
  if (NULL == pcfg) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  // memset(pcfg, 0, sizeof(vscp_frmw2_firmware_config_t));

  // pcfg->m_level     = VSCP_LEVEL2;
  // pcfg->m_puserdata = (void *) &gdevcfg;

  // pcfg->m_probe_timeout       = VSCP_PROBE_TIMEOUT;
  // pcfg->m_probe_timeout_count = VSCP_PROBE_TIMEOUT_COUNT;

  // pcfg->m_interval_heartbeat = 30000;
  // pcfg->m_interval_caps      = 240000;

  // pcfg->m_pDm         = NULL; // Pointer to decision matrix storage (NULL if no DM).
  // pcfg->m_nDmRows     = 0;    // Number of DM rows (0 if no DM).
  // pcfg->m_sizeDmRow   = 0;    // Size for one DM row.
  // pcfg->m_regOffsetDm = 0;    // Register offset for DM (normally zero)
  // pcfg->m_pageDm      = 0;    // Register page where DM definition starts

  // pcfg->m_pInternalMdf = NULL; // No internal MDF

  // pcfg->m_bInterestedInAllEvents = true;
  // pcfg->m_pEventsOfInterest      = NULL; // All events

  pcfg->m_guid[0] = 0xff; // Ethernet prefix
  pcfg->m_guid[1] = 0xff;
  pcfg->m_guid[2] = 0xff;
  pcfg->m_guid[3] = 0xff;
  pcfg->m_guid[4] = 0xff;
  pcfg->m_guid[5] = 0xff;
  pcfg->m_guid[6] = 0xff;
  pcfg->m_guid[7] = 0xfe;

  pcfg->m_guid[8]  = g_net_info.mac[0];
  pcfg->m_guid[9]  = g_net_info.mac[1];
  pcfg->m_guid[10] = g_net_info.mac[2];
  pcfg->m_guid[11] = g_net_info.mac[3];
  pcfg->m_guid[12] = g_net_info.mac[4];
  pcfg->m_guid[13] = g_net_info.mac[5];
  pcfg->m_guid[14] = 0x00;
  pcfg->m_guid[15] = 0x01;

  // Set MDF url
  strncpy(pcfg->m_mdfurl, MDF_URL, 32);

  // Create MQTT subscribe/publishing base
  sprintf(g_mqtt_pub_topic_base,
          "vscp/FF:FF:FF:FF:FF:FF:FF:FE:%02X:%02X:%02X:%02X:%02X:%02X:00:01",
          g_net_info.mac[0],
          g_net_info.mac[1],
          g_net_info.mac[2],
          g_net_info.mac[3],
          g_net_info.mac[4],
          g_net_info.mac[5]);

  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
//                           VSCP Callbacks
///////////////////////////////////////////////////////////////////////////////

uint64_t
vscp_frmw2_callback_get_timestamp(void *const puserdata)
{
  absolute_time_t t;
  uint64_t tus;

  t   = get_absolute_time();
  tus = to_us_since_boot(t);
  return tus;
}

uint32_t
vscp_frmw2_callback_get_milliseconds(void *const puserdata)
{
  absolute_time_t t;

  t = get_absolute_time();
  return to_ms_since_boot(t);
}

void
vscp_frmw2_callback_enter_bootloader(void *const puserdata)
{
  // Enter bootloader mode
  reset_usb_boot(0, 0);
}

int
vscp_frmw2_callback_dm_action(void *const puserdata,
                              const vscpEventEx *const pex,
                              uint8_t action,
                              const uint8_t *const pparam)
{
  // No DM we no nothing
  return VSCP_ERROR_SUCCESS;
}

int
vscp_frmw2_callback_segment_ctrl_heartbeat(void *const puserdata, uint16_t segcrc, uint32_t time)
{
  // We are not interested in a received segment controller heartbeat
  return VSCP_ERROR_SUCCESS;
}

int
vscp_frmw2_callback_report_events_of_interest(void *const puserdata)
{
  // m_bInterestedInAllEvents is true, we are interested in all events
  return VSCP_ERROR_SUCCESS;
}

int
vscp_frmw2_callback_set_event_time(void *const puserdata, vscpEventEx *const pex)
{
  // Check event pointer
  if (NULL == pex) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  // We have no realtime clock on this device so we set the time fields
  // to zero to let the receiving end timestamp
  pex->year   = 0;
  pex->month  = 0;
  pex->day    = 0;
  pex->hour   = 0;
  pex->minute = 0;
  pex->second = 0;

  return VSCP_ERROR_SUCCESS;
}

int
vscp_frmw2_callback_restore_defaults(void *const puserdata)
{
  gdevcfg.bCoefficient       = 0xf68;
  gdevcfg.bActive            = true;
  gdevcfg.bAlarmOnLow        = true;
  gdevcfg.bAlarmOnHigh       = true;
  gdevcfg.temp_current       = 0;
  gdevcfg.temp_setpoint      = -20;
  gdevcfg.temp_alarm_low     = -25;
  gdevcfg.temp_alarm_high    = -10;
  gdevcfg.hysteresis         = 5;
  gdevcfg.temp_report_period = 60;

  return VSCP_ERROR_SUCCESS;
}

void
vscp_frmw2_callback_reset(void *const puserdata)
{
  while (1)
    ;
}

int
vscp_frmw2_callback_get_ip_addr(void *const puserdata, uint8_t *pipaddr, uint8_t size)
{
  // Check pointer
  if (NULL == pipaddr) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  // Only ip.v4 addr.
  memset(pipaddr, 0, size);
  memcpy(pipaddr, g_net_info.ip, 4);

  return VSCP_ERROR_SUCCESS;
}

int
vscp_frmw2_callback_read_reg(void *const puserdata, uint16_t page, uint32_t reg, uint8_t *pval)
{
  printf("Read VSCP register... %d \n", reg);
  switch (reg) {

    case FRIDGE_REG_ZONE:
      *pval = gvscpcfg.m_zone;
      break;

    case FRIDGE_REG_SUBZONE:
      *pval = gvscpcfg.m_subzone;
      break;

    case FRIDGE_REG_STATUS:
      *pval = gpio_get(COMPRESSOR_RELAY_PIN) ? 0x80 : 0x00;
      break;

    case FRIDGE_REG_CONFIG:
      *pval = 0;
      *pval |= gdevcfg.bActive ? 0x80 : 0;
      *pval |= gdevcfg.bAlarmOnLow ? 0x02 : 0;
      *pval |= gdevcfg.bAlarmOnHigh ? 0x01 : 0;
      break;

    case FRIDGE_REG_TEMP_EVENT_PERIOD:
      *pval = gdevcfg.temp_report_period;
      break;

    case FRIDGE_REG_TEMP_MSB:
      *pval = (gdevcfg.temp_current >> 8) & 0xff;
      break;

    case FRIDGE_REG_TEMP_LSB:
      *pval = gdevcfg.temp_current & 0xff;
      break;

    case FRIDGE_REG_B_MSB:
      *pval = (gdevcfg.temp_current >> 8) & 0xff;
      ;
      break;

    case FRIDGE_REG_B_LSB:
      *pval = gdevcfg.temp_current & 0xff;
      break;

    case FRIDGE_REG_LOW_ALARM_MSB:
      *pval = (gdevcfg.temp_alarm_low >> 8) & 0xff;
      break;

    case FRIDGE_REG_LOW_ALARM_LSB:
      *pval = gdevcfg.temp_alarm_low & 0xff;
      break;

    case FRIDGE_REG_HIGH_ALARM_MSB:
      *pval = (gdevcfg.temp_alarm_high >> 8) & 0xff;
      break;

    case FRIDGE_REG_HIGH_ALARM_LSB:
      *pval = gdevcfg.temp_alarm_high & 0xff;
      break;

    case FRIDGE_REG_HYSTERESIS:
      *pval = gdevcfg.hysteresis;
      break;

    case FRIDGE_REG_SETTEMP:
      *pval = gdevcfg.temp_setpoint;
      break;
  }

  return VSCP_ERROR_SUCCESS;
}

int
vscp_frmw2_callback_write_reg(void *const puserdata, uint16_t page, uint32_t reg, uint8_t val)
{
  printf("Write VSCP register...\n");

  switch (reg) {

    case FRIDGE_REG_ZONE:
      gvscpcfg.m_zone = val;
      break;

    case FRIDGE_REG_SUBZONE:
      gvscpcfg.m_subzone = val;
      break;

    case FRIDGE_REG_STATUS:
      // Read Only
      break;

    case FRIDGE_REG_CONFIG:
      if (val & FRIDGE_CONFIG_ENABLE) {
        gdevcfg.bActive = true;
      }
      else {
        gdevcfg.bActive = false;
      }

      if (val & FRIDGE_CONFIG_ALARM_LOW) {
        gdevcfg.bAlarmOnLow = true;
      }
      else {
        gdevcfg.bAlarmOnLow = false;
      }

      if (val & FRIDGE_CONFIG_ALARM_HIGH) {
        gdevcfg.bAlarmOnHigh = true;
      }
      else {
        gdevcfg.bAlarmOnHigh = false;
      }
      break;

    case FRIDGE_REG_TEMP_EVENT_PERIOD:
      gdevcfg.temp_report_period = val;
      break;

    case FRIDGE_REG_TEMP_MSB:
      // Read only
      break;

    case FRIDGE_REG_TEMP_LSB:
      // Read only
      break;

    case FRIDGE_REG_B_MSB:
      gdevcfg.bCoefficient = ((uint16_t) val << 8) + gdevcfg.bCoefficient;
      break;

    case FRIDGE_REG_B_LSB:
      gdevcfg.bCoefficient = (gdevcfg.bCoefficient & 0xff00) + val;
      break;

    case FRIDGE_REG_LOW_ALARM_MSB:
      gdevcfg.temp_alarm_low = ((uint16_t) val << 8) + gdevcfg.temp_alarm_low;
      break;

    case FRIDGE_REG_LOW_ALARM_LSB:
      gdevcfg.temp_alarm_low = (gdevcfg.temp_alarm_low & 0xff00) + val;
      break;

    case FRIDGE_REG_HIGH_ALARM_MSB:
      gdevcfg.temp_alarm_high = ((uint16_t) val << 8) + gdevcfg.temp_alarm_high;
      break;

    case FRIDGE_REG_HIGH_ALARM_LSB:
      gdevcfg.temp_alarm_high = (gdevcfg.bCoefficient & 0xff00) + val;
      break;

    case FRIDGE_REG_HYSTERESIS:
      gdevcfg.hysteresis = val;
      break;

    case FRIDGE_REG_SETTEMP:
      gdevcfg.temp_setpoint = val;
      break;
  }
  return VSCP_ERROR_SUCCESS;
}

int
vscp_frmw2_callback_send_event_ex(void *const puserdata, vscpEventEx *pex)
{
  int rv;
  char buf[20];
  char bufEvent[2048];
  char bufTopic[2048];

  /* Publish */
  if (VSCP_ERROR_SUCCESS != (rv = vscp_fwhlp_create_json_ex(bufEvent, sizeof(bufEvent), pex))) {
    printf("Failed to create JSON event ex %d", rv);
    return rv;
  }

  printf("Send [%s]", bufEvent);

  /* Configure publish message */
  MQTTMessage mqtt_msg;
  mqtt_msg.qos        = QOS0;
  mqtt_msg.retained   = 0;
  mqtt_msg.dup        = 0;
  mqtt_msg.payload    = bufEvent;
  mqtt_msg.payloadlen = strlen(bufEvent);

  /* Topic*/
  sprintf(buf, "/%d/%d/%d", pex->vscp_class, pex->vscp_type, pex->sizeData ? pex->data[0] & 3 : 0);
  strcpy(bufTopic, g_mqtt_pub_topic_base);
  strcat(bufTopic, buf);

  if ((rv = MQTTPublish(&g_mqtt_client, bufTopic, &mqtt_msg)) < 0) {
    printf(" Publish failed : %d\n", rv);
    return VSCP_ERROR_WRITE_ERROR;
  }

  return VSCP_ERROR_SUCCESS;
}

int
vscp_frmw2_callback_stdreg_change(void *const puserdata, uint32_t stdreg)
{
  return VSCP_ERROR_SUCCESS;
}

void
vscp_frmw2_callback_feed_watchdog(void *const puserdata)
{
  watchdog_update();
}