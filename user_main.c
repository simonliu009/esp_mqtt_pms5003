/* main.c -- MQTT client example
*
* Copyright (c) 2014-2015, Tuan PM <tuanpm at live dot com>
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice,
* this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of Redis nor the names of its contributors may be used
* to endorse or promote products derived from this software without
* specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/
#include "ets_sys.h"
#include "driver/uart.h"
#include "osapi.h"
#include "mqtt.h"
#include "wifi.h"
#include "config.h"
#include "debug.h"
#include "gpio.h"
#include "user_interface.h"
#include "mem.h"
#define recvTaskPrio        1      //MQTT_TASK_PRIO taken zero already, we takes one
#define recvTaskQueueLen    4
#define MAX_TXBUFFER 512
#define MAX_UARTBUFFER (MAX_TXBUFFER/4)


MQTT_Client mqttClient;
static ETSTimer tmr0;
static uint32 refreshTime,unit,tick;
static uint8 gpsString[40];
static uint8 uartbuffer[MAX_UARTBUFFER];
static uint32 p1_0,p2_5,p10_0;
static char LASSstring[32];
static char GPSstring[40]="|gps_lat=22.9852305|gps_lon=120.1801026";
static char TIMESTAMP[10]="";
static char INFOstring[]="pms5003_esp8266_ttwang@jdaiot.com";
os_event_t recvTaskQueue[recvTaskQueueLen];


void wifiConnectCb(uint8_t status)
{
	if(status == STATION_GOT_IP){
		MQTT_Connect(&mqttClient);
	} else {
		MQTT_Disconnect(&mqttClient);
	}
}
void mqttConnectedCb(uint32_t *args)
{
	MQTT_Client* client = (MQTT_Client*)args;
	INFO("MQTT: Connected\r\n");
	MQTT_Subscribe(client, "/LASS/886/00/06/g500001/cmd", 0); //command channel
	MQTT_Subscribe(client, "/mqtt/timestamp", 1);
	MQTT_Subscribe(client, "/mqtt/echo", 2);

//	MQTT_Publish(client, "/LASS/886/00/06/g500001/pm3", "hello0", 6, 0, 0);
//	MQTT_Publish(client, "/LASS/886/00/06/g500001/alive", "hello1", 6, 1, 0);
//	MQTT_Publish(client, "/886/00/06/g500001/2", "hello2", 6, 2, 0);

}

void mqttDisconnectedCb(uint32_t *args)
{
	MQTT_Client* client = (MQTT_Client*)args;
	INFO("MQTT: Disconnected\r\n");
}

void mqttPublishedCb(uint32_t *args)
{
	MQTT_Client* client = (MQTT_Client*)args;
	INFO("MQTT: Published\r\n");
}

void mqttDataCb(uint32_t *args, const char* topic, uint32_t topic_len, const char *data, uint32_t data_len)
{
	char *topicBuf = (char*)os_zalloc(topic_len+1),
			*dataBuf = (char*)os_zalloc(data_len+1);
	uint8		*p, *q ;      

	MQTT_Client* client = (MQTT_Client*)args;

	os_memcpy(topicBuf, topic, topic_len);
	topicBuf[topic_len] = 0;

	os_memcpy(dataBuf, data, data_len);
	dataBuf[data_len] = 0;

	INFO("Receive topic: %s, data: %s \r\n", topicBuf, dataBuf);
  p=(uint8 *)os_strstr(topicBuf,"timestamp");
  q=(uint8 *)os_strstr(topicBuf,"echo");
//// modified for LASS  
  if(p){
      os_memcpy(&TIMESTAMP[0],dataBuf,10);
  }
  if(q){
     MQTT_Publish(client, "/LASS/886/00/06/clientid", "g500001", 7, 1, 0); //// echo my client id for KML builder
  }
//// define your own commands
  if(dataBuf[0]=='L'){
     os_memcpy(&GPSstring[0],(dataBuf+1),data_len-1);
  }else 
  if(dataBuf[0]=='R'){
     if(dataBuf[1]=='1') refreshTime=60; //seconds
     if(dataBuf[1]=='5') refreshTime=300; //seconds
     if(dataBuf[1]=='F') refreshTime=3600;  //1 hour
  }else
  if(dataBuf[0]=='T'){
     unit=1;
  }else
  if(dataBuf[0]=='S'){
     unit=0;
  }else
  if(dataBuf[0]=='G'){
     MQTT_Publish(client, "/LASS/886/00/06/g500001/gps", GPSstring, 40, 0, 0);
  }else
  if(dataBuf[0]=='I'){
     MQTT_Publish(client, "/LASS/886/00/06/g500001/info", INFOstring, sizeof(INFOstring), 0, 0);
  }else
  {
     os_printf("Unknown command\r\n");
  }
	os_free(topicBuf);
	os_free(dataBuf);
}
//// modified for LASS , receive data from UART0 (sensor)
static void ICACHE_FLASH_ATTR recvTask(os_event_t *events)
{
	uint8_t i;
	uint16 length = 0;
  //char* p;
  //p=&LASSstring[0];
	while (READ_PERI_REG(UART_STATUS(UART0)) & (UART_RXFIFO_CNT << UART_RXFIFO_CNT_S))
	{
		WRITE_PERI_REG(0X60000914, 0x73); //WTD
    //length=0;
		while ((READ_PERI_REG(UART_STATUS(UART0)) & (UART_RXFIFO_CNT << UART_RXFIFO_CNT_S)) && (length<MAX_UARTBUFFER))
			uartbuffer[length++] = READ_PERI_REG(UART_FIFO(UART0)) & 0xFF;
		//refer to Plantower PMS5003 
			p1_0 = (uint32)uartbuffer[4]*256+(uint32)uartbuffer[5];
			p2_5 = (uint32)uartbuffer[6]*256+(uint32)uartbuffer[7];
			p10_0 = (uint32)uartbuffer[8]*256+(uint32)uartbuffer[9];
	}

	if(UART_RXFIFO_FULL_INT_ST == (READ_PERI_REG(UART_INT_ST(UART0)) & UART_RXFIFO_FULL_INT_ST))
	{
		WRITE_PERI_REG(UART_INT_CLR(UART0), UART_RXFIFO_FULL_INT_CLR);
	}
	else if(UART_RXFIFO_TOUT_INT_ST == (READ_PERI_REG(UART_INT_ST(UART0)) & UART_RXFIFO_TOUT_INT_ST))
	{
		WRITE_PERI_REG(UART_INT_CLR(UART0), UART_RXFIFO_TOUT_INT_CLR);
	}
  
  INFO("%s\r\n",LASSstring);
	//uart_rx_intr_enable(UART0);
  ETS_UART_INTR_ENABLE();
  ///// BUG ! TX malfunctioned!
}
void ICACHE_FLASH_ATTR  tsr0(uint32_t *args) {
	MQTT_Client* client = (MQTT_Client*)args;
  if(tick>=refreshTime){
  os_sprintf(LASSstring,"%05d_%s",p1_0,TIMESTAMP);
	MQTT_Publish(client, "/LASS/886/00/06/g500001/pm1.0", LASSstring, 16, 0, 0);
  os_sprintf(LASSstring,"%05d_%s",p2_5,TIMESTAMP);
	MQTT_Publish(client, "/LASS/886/00/06/g500001/pm2.5", LASSstring, 16, 0, 0);
  os_sprintf(LASSstring,"%05d_%s",p10_0,TIMESTAMP);
	MQTT_Publish(client, "/LASS/886/00/06/g500001/pm10", LASSstring, 16, 0, 0);
  tick=0;
  }else{
    tick++;
  }
  
}




void user_init(void)
{
	//uart_init(BIT_RATE_115200, BIT_RATE_115200);
	uart_init(BIT_RATE_9600, BIT_RATE_115200);
	os_delay_us(1000000);

	CFG_Load();

	//MQTT_InitConnection(&mqttClient, sysCfg.mqtt_host, sysCfg.mqtt_port, sysCfg.security);
	MQTT_InitConnection(&mqttClient, "user.jdaiot.com", 1883, 0);

	//MQTT_InitClient(&mqttClient, sysCfg.device_id, sysCfg.mqtt_user, sysCfg.mqtt_pass, sysCfg.mqtt_keepalive, 1);
	MQTT_InitClient(&mqttClient, "g500001", "LASS", "LaSS2015", 120, 1);

	MQTT_InitLWT(&mqttClient, "/lwt", "offline", 0, 0);
	MQTT_OnConnected(&mqttClient, mqttConnectedCb);
	MQTT_OnDisconnected(&mqttClient, mqttDisconnectedCb);
	MQTT_OnPublished(&mqttClient, mqttPublishedCb);
	MQTT_OnData(&mqttClient, mqttDataCb);

	WIFI_Connect(sysCfg.sta_ssid, sysCfg.sta_pwd, wifiConnectCb);
  //// modified for LASS
	system_os_task(recvTask, recvTaskPrio, recvTaskQueue, recvTaskQueueLen);
  tick=0;
  refreshTime=10;
   // turn off timer
   os_timer_disarm(&tmr0);
   //
   // set the timer call back function
   //
   os_timer_setfn(&tmr0, tsr0, &mqttClient);
   //
   // turn timer on to trigger each .5 second
   //
   os_timer_arm(&tmr0, 5000, 1); //every 5 second


	INFO("\r\nSystem started ...\r\n");
}
