// w55rp20_fridge_ctrl.c
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

// clang-format off
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "hardware/adc.h"
#include "hardware/gpio.h"

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

/* Network */
static wiz_NetInfo g_net_info = {
  .mac  = { 0x00, 0x08, 0xDC, 0x12, 0x34, 0x56 }, // MAC address
  .ip   = { 192, 168, 1, 200 },                   // IP address
  .sn   = { 255, 255, 255, 0 },                   // Subnet Mask
  .gw   = { 192, 168, 1, 1 },                     // Gateway
  .dns  = { 8, 8, 8, 8 },                         // DNS server
  .dhcp = NETINFO_STATIC                          // DHCP enable/disable
};

/*
  Topic is "vscp/GUID/class/type/index" and the base "vscp/GUID/"is set here. The topic base
  is used by both subscribe and publish
    max: vscp/00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00/32767/65535/255 (length: 68)
  set to ethernet prefix + mac-addr (from g_net_info above) + 00:01
*/
static char g_mqtt_pub_topic_base[55] = {"vscp/FF:FF:FF:FF:FF:FF:FF:FE:00:08:DC:12:34:56:00:01/"};

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
repeating_timer_callback(void);
static time_t
millis(void);

/* Fridge control */
static float
readFridgeTemperature(void);

static int
init_vscp(vscp_frmw2_firmware_config_t *pcfg);

int
main()
{
  /* Initialize */
  int32_t retval    = 0;
  uint32_t start_ms = 0;
  vscp_frmw2_firmware_config_t cfg;
  int rv;

  rv = init_vscp(&cfg);
  rv = vscp_frmw2_init(&cfg);

  set_clock_khz();
  stdio_init_all();

  printf("Initialized\n");

  adc_init();

  // Make sure GPIO is high-impedance, no pullups etc
  adc_gpio_init(26);

  // Select ADC input 0 (GPIO26)
  adc_select_input(0);

  wizchip_spi_initialize();
  wizchip_cris_initialize();

  wizchip_reset();
  wizchip_initialize();
  wizchip_check();

  wizchip_1ms_timer_initialize(repeating_timer_callback);

  network_initialize(g_net_info);

  /* Get network information */
  print_network_information(g_net_info);

  NewNetwork(&g_mqtt_network, SOCKET_MQTT);

  retval = ConnectNetwork(&g_mqtt_network, g_mqtt_broker_ip, PORT_MQTT);

  if (retval != 1) {
    printf(" Network connect failed\n");

    while (1)
      ;
  }

  /* Initialize MQTT client */
  MQTTClientInit(&g_mqtt_client,
                 &g_mqtt_network,
                 DEFAULT_TIMEOUT,
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

  retval = MQTTConnect(&g_mqtt_client, &g_mqtt_packet_connect_data);

  if (retval < 0) {
    printf(" MQTT connect failed : %d\n", retval);

    while (1)
      ;
  }

  printf(" MQTT connected\n");

  /* Configure publish message */
  g_mqtt_message.qos        = QOS0;
  g_mqtt_message.retained   = 0;
  g_mqtt_message.dup        = 0;
  g_mqtt_message.payload    = MQTT_PUBLISH_PAYLOAD;
  g_mqtt_message.payloadlen = strlen(g_mqtt_message.payload);

  /* Subscribe */
  retval = MQTTSubscribe(&g_mqtt_client, g_mqtt_pub_topic_base, QOS0, message_arrived);

  if (retval < 0) {
    printf(" Subscribe failed : %d\n", retval);

    while (1)
      ;
  }

  printf(" Subscribed\n");

  start_ms = millis();

  /* Work loop */
  while (1) {

    if ((retval = MQTTYield(&g_mqtt_client, g_mqtt_packet_connect_data.keepAliveInterval)) < 0) {
      printf(" Yield error : %d\n", retval);

      while (1)
        ;
    }

    // Heartbeat
    if (millis() > (start_ms + cfg.m_interval_heartbeat)) {

      // 12-bit conversion, assume max value == ADC_VREF == 3.3 V
      const float conversion_factor = 3.3f / (1 << 12);
      uint16_t result               = adc_read();
      // printf("Raw value: 0x%03x, voltage: %f V\n", result,
      //        result * conversion_factor);
      printf("Raw value: 0x%03x, voltage: %f V\n", result, readFridgeTemperature());

      /* Publish */
      vscpEventEx ex;
      memset(&ex,0, sizeof(ex));

      ex.head = 0;
      memcpy(ex.GUID,cfg.m_guid,16);
      ex.timestamp = vscp_frmw2_callback_get_timestamp(NULL);
      ex.vscp_class = VSCP_CLASS1_INFORMATION;
      ex.vscp_type = VSCP_TYPE_INFORMATION_NODE_HEARTBEAT;
      ex.sizeData = 3;
      ex.data[0] = 0;
      ex.data[1] = cfg.m_zone;
      ex.data[2] = cfg.m_subzone;

      retval = MQTTPublish(&g_mqtt_client, MQTT_PUBLISH_TOPIC, &g_mqtt_message);

      if (retval < 0) {
        printf(" Publish failed : %d\n", retval);

        while (1)
          ;
      }

      printf(" Published\n");

      start_ms = millis();
    }
  }
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
  MQTTMessage *message = msg_data->message;

  printf("%.*s", (uint32_t) message->payloadlen, (uint8_t *) message->payload);
}

///////////////////////////////////////////////////////////////////////////////
// repeating_timer_callback
//

static void
repeating_timer_callback(void)
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

static float
readFridgeTemperature(void)
{
  // Select ADC input (0 (GPIO26))
  adc_select_input(0);

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

  uint16_t B        = 3450;
  double calVoltage = 3.3;

  double Rinf = 10000.0 * exp(B / -298.15);

  // V2 = adc * voltage/4096
  double v = calVoltage * (double) result / 4096;

  // R1 = (R2V - R2V2) / V2  R2= 10K, V = 5V,  V2 = adc * voltage/1024
  double resistance = (10000.0 * (calVoltage - v)) / v;

  // itemp = r;
  double temp = ((double) B) / log(resistance / Rinf);
  // itemp = log(r/Rinf);
  temp -= 273.15; // Convert Kelvin to Celsius

  // avarage = testadc;
  /*  https://learn.adafruit.com/thermistor/using-a-thermistor
  avarage = (1023/avarage) - 1;
  avarage = 10000 / avarage;      // Resistance of termistor
  //temp = avarage/10000;           // (R/Ro)
  temp = 10000/avarage;
  temp = log(temp);               // ln(R/Ro)
  temp /= B;                      // 1/B * ln(R/Ro)
  temp += 1.0 / (25 + 273.15);    // + (1/To)
  temp = 1.0 / temp;              // Invert
  temp -= 273.15;
  */
  uint32_t current_temp = (long) (temp * 100);
  printf("Temperature: %f C\n", temp);

  return temp;
}

///////////////////////////////////////////////////////////////////////////////
//                               VSCP
///////////////////////////////////////////////////////////////////////////////

static int
init_vscp(vscp_frmw2_firmware_config_t *pcfg)
{
  int rv;

  memset(pcfg, 0, sizeof(vscp_frmw2_firmware_config_t));

  pcfg->m_level     = VSCP_LEVEL2;
  pcfg->m_puserdata = NULL;

  pcfg->m_probe_timeout       = VSCP_PROBE_TIMEOUT;
  pcfg->m_probe_timeout_count = VSCP_PROBE_TIMEOUT_COUNT;

  pcfg->m_interval_heartbeat = 30000;
  pcfg->m_interval_caps      = 240000;

  pcfg->m_pDm         = NULL; // Pointer to decision matrix storage (NULL if no DM).
  pcfg->m_nDmRows     = 0;    // Number of DM rows (0 if no DM).
  pcfg->m_sizeDmRow   = 0;    // Size for one DM row.
  pcfg->m_regOffsetDm = 0;    // Register offset for DM (normally zero)
  pcfg->m_pageDm      = 0;    // Register page where DM definition starts

  pcfg->m_pInternalMdf = NULL; // No internal MDF

  pcfg->m_bInterestedInAllEvents = true;
  pcfg->m_pEventsOfInterest      = NULL; // All events

  pcfg->m_guid[15] = 0xff; // Ethernet prefix
  pcfg->m_guid[14] = 0xff;
  pcfg->m_guid[13] = 0xff;
  pcfg->m_guid[12] = 0xff;
  pcfg->m_guid[11] = 0xff;
  pcfg->m_guid[10] = 0xff;
  pcfg->m_guid[9]  = 0xff;
  pcfg->m_guid[8]  = 0xf3;

  pcfg->m_guid[7] = g_net_info.mac[0];
  pcfg->m_guid[6] = g_net_info.mac[1];
  pcfg->m_guid[5] = g_net_info.mac[2];
  pcfg->m_guid[4] = g_net_info.mac[3];
  pcfg->m_guid[3] = g_net_info.mac[4];
  pcfg->m_guid[2] = g_net_info.mac[5];
  pcfg->m_guid[1] = 0x00;
  pcfg->m_guid[0] = 0x01;

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
  return rv;
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
  return (uint32_t) tus;
}

void
vscp_frmw2_callback_enter_bootloader(void *const puserdata)
{
  // No bootloader, we do nothing
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
  int rv;

  return rv;
}

int
vscp_frmw2_callback_report_events_of_interest(void *const puserdata)
{
  int rv;

  return rv;
}

int
vscp_frmw2_callback_set_event_time(void *const puserdata, vscpEventEx *const pex)
{
  int rv;

  return rv;
}

int
vscp_frmw2_callback_restore_defaults(void *const puserdata)
{
  int rv;

  return rv;
}

void
vscp_frmw2_callback_reset(void *const puserdata)
{
}

int
vscp_frmw2_callback_get_ip_addr(void *const puserdata, uint8_t *pipaddr, uint8_t size)
{
  int rv;

  return rv;
}

int
vscp_frmw2_callback_read_reg(void *const puserdata, uint16_t page, uint32_t reg, uint8_t *pval)
{
}

int
vscp_frmw2_callback_write_reg(void *const puserdata, uint16_t page, uint32_t reg, uint8_t val)
{
  int rv;

  return rv;
}

int
vscp_frmw2_callback_send_event_ex(void *const puserdata, vscpEventEx *pex)
{
  int rv;
  char buf[512];
  /* Publish */
      // vscpEventEx ex;
      // memset(&ex,0, sizeof(ex));

      // ex.head = 0;
      // memcpy(ex.GUID,cfg.m_guid,16);
      // ex.timestamp = vscp_frmw2_callback_get_timestamp(NULL);
      // ex.vscp_class = VSCP_CLASS1_INFORMATION;
      // ex.vscp_type = VSCP_TYPE_INFORMATION_NODE_HEARTBEAT;
      // ex.sizeData = 3;
      // ex.data[0] = 0;
      // ex.data[1] = cfg.m_zone;
      // ex.data[2] = cfg.m_subzone;

      if (VSCP_ERROR_SUCCESS != (rv =  vscp_fwhlp_create_json_ex(buf, sizeof(buf), pex))) {
        printf("Failed to create JSON event ex %d", rv);
        return rv;
      }

      /* Configure publish message */
      MQTTMessage mqtt_msg;
      mqtt_msg.qos        = QOS0;
      mqtt_msg.retained   = 0;
      mqtt_msg.dup        = 0;
      mqtt_msg.payload    = buf;
      mqtt_msg.payloadlen = strlen(g_mqtt_message.payload);

      if ((rv = MQTTPublish(&g_mqtt_client, MQTT_PUBLISH_TOPIC, &mqtt_msg)) < 0) {
        printf(" Publish failed : %d\n", rv);
        return VSCP_ERROR_WRITE_ERROR;
      }

  return rv;
}

int
vscp_frmw2_callback_stdreg_change(void *const puserdata, uint32_t stdreg)
{
  int rv;

  return rv;
}

void
vscp_frmw2_callback_feed_watchdog(void *const puserdata)
{
}