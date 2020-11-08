/*
 * server.c
 *
 *  Created on: Aug 18, 2020
 *      Author: ankayca
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "lwip/api.h"

#include "driver/uart.h"

#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_event.h"
#include "driver/ledc.h"

#include "string.h"

#include "websocket_server.h"
#include "server.h"

#define LED_PIN 18
#define AP_SSID "admin"
#define AP_PSSWD "aaaaaaaa"
#define EX_UART_NUM UART_NUM_0

#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)
static QueueHandle_t uart0_queue;
static const char *TAG = "uart_events";
uint8_t len2=0,count,count2=0;
char array[1024];

static ledc_channel_config_t ledc_channel;
QueueHandle_t client_queue;
static const int client_queue_size = 10;
// handles WiFi events
 esp_err_t event_handler(void* ctx, system_event_t* event) {
  const char* TAG = "event_handler";
  switch(event->event_id) {
    case SYSTEM_EVENT_AP_START:
      //ESP_ERROR_CHECK(tcpip_adapter_set_hostname(TCPIP_ADAPTER_IF_STA, "esp32"));
      ESP_LOGI(TAG,"Access Point Started");
      break;
    case SYSTEM_EVENT_AP_STOP:
      ESP_LOGI(TAG,"Access Point Stopped");
      break;
    case SYSTEM_EVENT_AP_STACONNECTED:
      ESP_LOGI(TAG,"STA Connected, MAC=%02x:%02x:%02x:%02x:%02x:%02x AID=%i",
        event->event_info.sta_connected.mac[0],event->event_info.sta_connected.mac[1],
        event->event_info.sta_connected.mac[2],event->event_info.sta_connected.mac[3],
        event->event_info.sta_connected.mac[4],event->event_info.sta_connected.mac[5],
        event->event_info.sta_connected.aid);
      break;
    case SYSTEM_EVENT_AP_STADISCONNECTED:
      ESP_LOGI(TAG,"STA Disconnected, MAC=%02x:%02x:%02x:%02x:%02x:%02x AID=%i",
        event->event_info.sta_disconnected.mac[0],event->event_info.sta_disconnected.mac[1],
        event->event_info.sta_disconnected.mac[2],event->event_info.sta_disconnected.mac[3],
        event->event_info.sta_disconnected.mac[4],event->event_info.sta_disconnected.mac[5],
        event->event_info.sta_disconnected.aid);
      break;
    case SYSTEM_EVENT_AP_PROBEREQRECVED:
      ESP_LOGI(TAG,"AP Probe Received");
      break;
    case SYSTEM_EVENT_AP_STA_GOT_IP6:
      ESP_LOGI(TAG,"Got IP6=%01x:%01x:%01x:%01x",
        event->event_info.got_ip6.ip6_info.ip.addr[0],event->event_info.got_ip6.ip6_info.ip.addr[1],
        event->event_info.got_ip6.ip6_info.ip.addr[2],event->event_info.got_ip6.ip6_info.ip.addr[3]);
      break;
    default:
      ESP_LOGI(TAG,"Unregistered event=%i",event->event_id);
      break;
  }
  return ESP_OK;
}
// sets up WiFi
void wifi_setup() {
  const char* TAG = "wifi_setup";

  ESP_LOGI(TAG,"starting tcpip adapter");

  ESP_ERROR_CHECK(tcpip_adapter_dhcps_stop(TCPIP_ADAPTER_IF_AP));
  //tcpip_adapter_set_hostname(TCPIP_ADAPTER_IF_AP,"esp32");
  tcpip_adapter_ip_info_t info;
  memset(&info, 0, sizeof(info));
  IP4_ADDR(&info.ip, 192, 168, 4, 1);
  IP4_ADDR(&info.gw, 192, 168, 4, 1);
  IP4_ADDR(&info.netmask, 255, 255, 255, 0);
  ESP_LOGI(TAG,"setting gateway IP");
  ESP_ERROR_CHECK(tcpip_adapter_set_ip_info(TCPIP_ADAPTER_IF_AP, &info));
  //ESP_ERROR_CHECK(tcpip_adapter_set_hostname(TCPIP_ADAPTER_IF_AP,"esp32"));
  //ESP_LOGI(TAG,"set hostname to \"%s\"",hostname);
  ESP_LOGI(TAG,"starting DHCPS adapter");
  ESP_ERROR_CHECK(tcpip_adapter_dhcps_start(TCPIP_ADAPTER_IF_AP));
  //ESP_ERROR_CHECK(tcpip_adapter_set_hostname(TCPIP_ADAPTER_IF_AP,hostname));
  ESP_LOGI(TAG,"starting event loop");
  ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));

  ESP_LOGI(TAG,"initializing WiFi");
  wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_config));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));

  wifi_config_t wifi_config = {
    .ap = {
      .ssid = AP_SSID,
      .password= AP_PSSWD,
      .channel = 0,
      .authmode = WIFI_AUTH_WPA2_PSK,
      .ssid_hidden = 0,
      .max_connection = 4,
      .beacon_interval = 100
    }
  };

  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());
  ESP_LOGI(TAG,"WiFi set up");
}
// sets up the led for pwm
void led_duty(uint16_t duty) {
  static uint16_t val;
  static uint16_t max = (1L<<10)-1;
  if(duty > 100) return;
  val = (duty * max) / 100;
  ledc_set_duty(ledc_channel.speed_mode,ledc_channel.channel,val);
  ledc_update_duty(ledc_channel.speed_mode,ledc_channel.channel);
}

void led_setup() {
  const static char* TAG = "led_setup";

  ledc_timer_config_t ledc_timer = {
    .duty_resolution = LEDC_TIMER_10_BIT,
    .freq_hz         = 5000,
    .speed_mode      = 0,
    .timer_num       = LEDC_TIMER_0
  };

  ledc_channel.channel = LEDC_CHANNEL_0;
  ledc_channel.duty = 0;
  ledc_channel.gpio_num = LED_PIN,
  ledc_channel.speed_mode = 0;
  ledc_channel.timer_sel = LEDC_TIMER_0;

  ledc_timer_config(&ledc_timer);
  ledc_channel_config(&ledc_channel);
  //led_duty(0);
  ESP_LOGI(TAG,"led is off and ready, 10 bits");
}

void uart_init(void){
	 uart_config_t uart_config = {
	        .baud_rate = 9600,
	        .data_bits = UART_DATA_8_BITS,
	        .parity = UART_PARITY_DISABLE,
	        .stop_bits = UART_STOP_BITS_1,
	        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
	    };
	    uart_param_config(EX_UART_NUM, &uart_config);

	    // Install UART driver, and get the queue.
	    uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 100, &uart0_queue, 0);
}
void uart_handler(void* pvParameters) {
	char out[4];
	uint8_t count=0,count2=0;
    uart_event_t event;

    uint8_t *dtmp = (uint8_t *) malloc(RD_BUF_SIZE);

    for (;;) {
        // Waiting for UART event.
        if (xQueueReceive(uart0_queue, (void *)&event, (portTickType)portMAX_DELAY)) {
            //bzero(dtmp, RD_BUF_SIZE);
            //ESP_LOGI(TAG, "uart[%d] event:", EX_UART_NUM);

            switch (event.type) {
                // Event of UART receving data
                // We'd better handler data event fast, there would be much more data events than
                // other types of events. If we take too much time on data event, the queue might be full.
                case UART_DATA:
                    //ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
                    //printf(" basliyorr\n\r");
                    uart_flush_input(EX_UART_NUM);
                    uart_read_bytes(EX_UART_NUM, dtmp, event.size, portMAX_DELAY);




                    for(count=0;count <= event.size;count++){
                    	if( ((dtmp[count]== 83) || (dtmp[count] == 65) ) &&  (dtmp[count+5] == 72) ){

								for(count2=0;count2 <=4; count2++){
									out[count2] = dtmp[count+count2];

								}
								 ws_server_send_text_all(out,5);
								 //ESP_LOGI(TAG, "fordan ciktim %s",out);

							}


						}
                    uart_flush_input(EX_UART_NUM);
                    break;

                // Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGI(TAG, "hw fifo overflow");
                    // If fifo overflow happened, you should consider adding flow control for your application.
                    // The ISR has already reset the rx FIFO,
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(EX_UART_NUM);
                    xQueueReset(uart0_queue);
                    break;

                // Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGI(TAG, "ring buffer full");
                    // If buffer full happened, you should consider encreasing your buffer size
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(EX_UART_NUM);
                    xQueueReset(uart0_queue);
                    break;

                case UART_PARITY_ERR:
                    ESP_LOGI(TAG, "uart parity error");
                    break;

                // Event of UART frame error
                case UART_FRAME_ERR:
                    ESP_LOGI(TAG, "uart frame error");
                    break;

                // Others
                default:
                    ESP_LOGI(TAG, "uart event type: %d", event.type);
                    break;
            }
        }
    }

    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}
void websocket_callback(uint8_t num,WEBSOCKET_TYPE_t type,char* msg,uint64_t len) {
  const static char* TAG = "websocket_callback";
  int value;

  switch(type) {
    case WEBSOCKET_CONNECT:
      ESP_LOGI(TAG,"client %i connected!",num);


      break;
    case WEBSOCKET_DISCONNECT_EXTERNAL:

      ESP_LOGI(TAG,"client %i sent a disconnect message",num);

      break;
    case WEBSOCKET_DISCONNECT_INTERNAL:
      ESP_LOGI(TAG,"client %i was disconnected",num);
      break;
    case WEBSOCKET_DISCONNECT_ERROR:
      ESP_LOGI(TAG,"client %i was disconnected due to an error",num);
      break;
    case WEBSOCKET_TEXT:

      ESP_LOGI(TAG, "bisey var gibi ");
      break;
    case WEBSOCKET_BIN:
      ESP_LOGI(TAG,"client %i sent binary message of size %i:\n%s",num,(uint32_t)len,msg);
      break;
    case WEBSOCKET_PING:
      ESP_LOGI(TAG,"client %i pinged us with message of size %i:\n%s",num,(uint32_t)len,msg);
      break;
    case WEBSOCKET_PONG:
      ESP_LOGI(TAG,"client %i responded to the ping",num);
      break;
  }
}
// serves any clients
void http_serve(struct netconn *conn) {
  const static char* TAG = "http_server";
  const static char HTML_HEADER[] = "HTTP/1.1 200 OK\nContent-type: text/html\n\n";

  const static char JS_HEADER[] = "HTTP/1.1 200 OK\nContent-type: text/javascript\n\n";
  const static char CSS_HEADER[] = "HTTP/1.1 200 OK\nContent-type: text/css\n\n";
  struct netbuf* inbuf;
  static char* buf;
  static uint16_t buflen;
  static err_t err;

  // default page
  extern const uint8_t root_html_start[] asm("_binary_root_html_start");
  extern const uint8_t root_html_end[] asm("_binary_root_html_end");
  const uint32_t root_html_len = root_html_end - root_html_start;

  // test.js
  extern const uint8_t test_js_start[] asm("_binary_test_js_start");
  extern const uint8_t test_js_end[] asm("_binary_test_js_end");
  const uint32_t test_js_len = test_js_end - test_js_start;

  // test.css
  extern const uint8_t test_css_start[] asm("_binary_test_css_start");
  extern const uint8_t test_css_end[] asm("_binary_test_css_end");
  const uint32_t test_css_len = test_css_end - test_css_start;


  netconn_set_recvtimeout(conn,1000); // allow a connection timeout of 1 second
  ESP_LOGI(TAG,"reading from client...");
  err = netconn_recv(conn, &inbuf);
  ESP_LOGI(TAG,"read from client");
  if(err==ERR_OK) {
    netbuf_data(inbuf, (void**)&buf, &buflen);
    if(buf) {

      // default page
      if     (strstr(buf,"GET / ")
          && !strstr(buf,"Upgrade: websocket")) {
        ESP_LOGI(TAG,"Sending /");
        netconn_write(conn, HTML_HEADER, sizeof(HTML_HEADER)-1,NETCONN_NOCOPY);
        netconn_write(conn, root_html_start,root_html_len,NETCONN_NOCOPY);
        netconn_close(conn);
        netconn_delete(conn);
        netbuf_delete(inbuf);
      }

      // default page websocket
      else if(strstr(buf,"GET / ")
           && strstr(buf,"Upgrade: websocket")) {
        ESP_LOGI(TAG,"Requesting websocket on /");
        ws_server_add_client(conn,buf,buflen,"/",websocket_callback);

        netbuf_delete(inbuf);
      }

      else if(strstr(buf,"GET /test.js ")) {
        ESP_LOGI(TAG,"Sending /test.js");
        netconn_write(conn, JS_HEADER, sizeof(JS_HEADER)-1,NETCONN_NOCOPY);
        netconn_write(conn, test_js_start, test_js_len,NETCONN_NOCOPY);
        netconn_close(conn);
        netconn_delete(conn);
        netbuf_delete(inbuf);
      }

      else if(strstr(buf,"GET /test.css ")) {
        ESP_LOGI(TAG,"Sending /test.css");
        netconn_write(conn, CSS_HEADER, sizeof(CSS_HEADER)-1,NETCONN_NOCOPY);
        netconn_write(conn, test_css_start, test_css_len,NETCONN_NOCOPY);
        netconn_close(conn);
        netconn_delete(conn);
        netbuf_delete(inbuf);
      }



      else {
        ESP_LOGI(TAG,"Unknown request");
        netconn_close(conn);
        netconn_delete(conn);
        netbuf_delete(inbuf);
      }
    }
    else {
      ESP_LOGI(TAG,"Unknown request (empty?...)");
      netconn_close(conn);
      netconn_delete(conn);
      netbuf_delete(inbuf);
    }
  }
  else { // if err==ERR_OK
    ESP_LOGI(TAG,"error on read, closing connection");
    netconn_close(conn);
    netconn_delete(conn);
    netbuf_delete(inbuf);
  }
}
// handles clients when they first connect. passes to a queue
void server_task(void* pvParameters) {
  const static char* TAG = "server_task";
  struct netconn *conn, *newconn;
  static err_t err;
  client_queue = xQueueCreate(client_queue_size,sizeof(struct netconn*));

  conn = netconn_new(NETCONN_TCP);
  netconn_bind(conn,NULL,80);
  netconn_listen(conn);
  ESP_LOGI(TAG,"server listening");
  do {
    err = netconn_accept(conn, &newconn);
    ESP_LOGI(TAG,"new client");
    if(err == ERR_OK) {
    	ESP_LOGI(TAG,"server_taskin icindeyim 1");
      xQueueSendToBack(client_queue,&newconn,portMAX_DELAY);
      ESP_LOGI(TAG,"server_taskin icindeyim 2");
      //http_serve(newconn);
    }
  } while(err == ERR_OK);
  netconn_close(conn);
  netconn_delete(conn);
  ESP_LOGE(TAG,"task ending, rebooting board");
  esp_restart();
}
// receives clients from queue, handles them
void server_handle_task(void* pvParameters) {
  const static char* TAG = "server_handle_task";
  struct netconn* conn;
  ESP_LOGI(TAG,"task starting");
  for(;;) {
    xQueueReceive(client_queue,&conn,portMAX_DELAY);
    if(!conn) continue;
    http_serve(conn);
  }
  vTaskDelete(NULL);
}
