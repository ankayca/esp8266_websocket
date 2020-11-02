/*
An example showing the ESP32 as a
WebSocket server.

Demonstrates:
the ESP32 as a WiFi Access Point,
embedding files (html, js, etc.) for a server,
WebSockets,
LED control.

All example options are in "Example Options"

All WebSocket Server options are in:
Component config ---> WebSocket Server

Connect an LED to pin 2 (default)
Connect to the Access Point,
default name: "ESP32 Test"
password: "hello world"

go to 192.168.4.1 in a browser

Note that there are two regular server tasks.
The first gets incoming clients, then passes them
to a queue for the second task to actually handle.
I found that connections were dropped much less frequently
this way, especially when handling large requests. It
does use more RAM though.
*/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "lwip/api.h"

#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "driver/ledc.h"

#include "string.h"

#include "websocket_server.h"

#include "server.h"
#include "driver/uart.h"
#define EX_UART_NUM UART_NUM_0

#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)
static QueueHandle_t uart0_queue;
static const char *TAG = "uart_events";
static void count_task(void* pvParameters) {
    uart_event_t event;
    uint8_t *dtmp = (uint8_t *) malloc(RD_BUF_SIZE);

    // int len;
    // int clients;
    for (;;) {
        // Waiting for UART event.
        if (xQueueReceive(uart0_queue, (void *)&event, (portTickType)portMAX_DELAY)) {
            bzero(dtmp, RD_BUF_SIZE);
            ESP_LOGI(TAG, "uart[%d] event:", EX_UART_NUM);

            switch (event.type) {
                // Event of UART receving data
                // We'd better handler data event fast, there would be much more data events than
                // other types of events. If we take too much time on data event, the queue might be full.
                case UART_DATA:
                    ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
                    uart_read_bytes(EX_UART_NUM, dtmp, event.size, portMAX_DELAY);
    
                    // I can read and write dtmp with uart_read_bytes() , but I want to send data via web socket
                    // clients = ws_server_send_text_all(out,len);
                   
                    // ws_server_send_text_all() declatered  from components/websocket/websocket.c
                    // uart_read_bytes() declareted 

                                        /**
                     * @brief UART read bytes from UART buffer
                     *
                     * @param uart_num Uart port number.
                     * @param buf     pointer to the buffer.
                     * @param length  data length
                     * @param ticks_to_wait sTimeout, count in RTOS ticks
                     *
                     * @return
                     *     - (-1) Error
                     *     - OTHERS (>=0) The number of bytes read from UART FIFO
                     */
                    //int uart_read_bytes(uart_port_t uart_num, uint8_t *buf, uint32_t length, TickType_t ticks_to_wait);
                    

                    ESP_LOGI(TAG, "[DATA EVT]:");
                    uart_write_bytes(EX_UART_NUM, (const char *) dtmp, event.size);
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





void app_main() {


  tcpip_adapter_init();
  nvs_flash_init();
  wifi_setup();
  led_setup();
  ws_server_start();
  xTaskCreate(&server_task,"server_task",3000,NULL,9,NULL);
  xTaskCreate(&server_handle_task,"server_handle_task",4000,NULL,6,NULL);
  xTaskCreate(&count_task,"count_task",6000,NULL,2,NULL);
}
