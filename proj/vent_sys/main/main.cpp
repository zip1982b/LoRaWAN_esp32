/*******************************************************************************
 * 
 * ttn-esp32 - The Things Network device library for ESP-IDF / SX127x
 * 
 * Copyright (c) 2018 Manuel Bleichenbacher
 * 
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 *
 * Sample program showing how to send a test message every 30 second.
 *******************************************************************************/

#include "freertos/FreeRTOS.h"
#include "esp_event.h"
#include "nvs_flash.h"

#include "driver/gpio.h"

#include "TheThingsNetwork.h"


// NOTE:
// The LoRaWAN frequency and the radio chip must be configured by running 'make menuconfig'.
// Go to Components / The Things Network, select the appropriate values and save.

// Copy the below hex string from the "Device EUI" field
// on your device's overview page in the TTN console.
const char *devEui = "000123456abc3125";

// Copy the below two lines from bottom of the same page
const char *appEui = "0b8b460dc0c3cbb4";
const char *appKey = "3d23df2458d8fda3aadb1c3f2a6e4fe8";

// Pins and other resources
#define TTN_SPI_HOST      HSPI_HOST
#define TTN_SPI_DMA_CHAN  1
#define TTN_PIN_SPI_SCLK  18
#define TTN_PIN_SPI_MOSI  23
#define TTN_PIN_SPI_MISO  19
#define TTN_PIN_NSS       5
#define TTN_PIN_RXTX      TTN_NOT_CONNECTED
#define TTN_PIN_RST       TTN_NOT_CONNECTED
#define TTN_PIN_DIO0      26
#define TTN_PIN_DIO1      33


/**
 * Brief:
 * 
 * GPIO status:
 * GPIO32: output - FAN control
 * GPIO34:  input, interrupt from rising edge and falling edge - zero sensor
 * 
 */
#define FAN				32
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<FAN))
#define ZERO_SENSOR			34
#define GPIO_INPUT_PIN_SEL  ((1ULL<<ZERO_SENSOR))
#define ESP_INTR_FLAG_DEFAULT 0

xQueueHandle xQueueDIM;
xQueueHandle xQueueISR;


static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
	if(gpio_num == ZERO_SENSOR){
		gpio_set_intr_type(ZERO_SENSOR, GPIO_INTR_DISABLE);
		xQueueSendFromISR(xQueueISR, &gpio_num, 0);
	}
}








static TheThingsNetwork ttn;

const unsigned TX_INTERVAL = 30;
static uint8_t msgData[] = "Hello, world";


void sendMessages(void* pvParameter)
{
    while (1) {
        printf("Sending message...\n");
        TTNResponseCode res = ttn.transmitMessage(msgData, sizeof(msgData) - 1);
        printf(res == kTTNSuccessfulTransmission ? "Message sent.\n" : "Transmission failed.\n");

        vTaskDelay(TX_INTERVAL * 1000 / portTICK_PERIOD_MS);
    }
}








extern "C" void app_main(void)
{
    esp_err_t err;
    // Initialize the GPIO ISR handler service
    err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    ESP_ERROR_CHECK(err);

    // Initialize the NVS (non-volatile storage) for saving and restoring the keys
    err = nvs_flash_init();
    ESP_ERROR_CHECK(err);

    // Initialize SPI bus
    spi_bus_config_t spi_bus_config;
    spi_bus_config.miso_io_num = TTN_PIN_SPI_MISO;
    spi_bus_config.mosi_io_num = TTN_PIN_SPI_MOSI;
    spi_bus_config.sclk_io_num = TTN_PIN_SPI_SCLK;
    spi_bus_config.quadwp_io_num = -1;
    spi_bus_config.quadhd_io_num = -1;
    spi_bus_config.max_transfer_sz = 0;
    err = spi_bus_initialize(TTN_SPI_HOST, &spi_bus_config, TTN_SPI_DMA_CHAN);
    ESP_ERROR_CHECK(err);

    // Configure the SX127x pins
    ttn.configurePins(TTN_SPI_HOST, TTN_PIN_NSS, TTN_PIN_RXTX, TTN_PIN_RST, TTN_PIN_DIO0, TTN_PIN_DIO1);

    // The below line can be commented after the first run as the data is saved in NVS
    ttn.provision(devEui, appEui, appKey);

    printf("Joining...\n");
    if (ttn.join())
    {
        printf("Joined.\n");
        xTaskCreate(sendMessages, "send_messages", 1024 * 4, (void* )0, 3, nullptr);
		xTaskCreate(venting, "fan_work", 1024 * 4,  NULL, 11, NULL);
    }
    else
    {
        printf("Join failed. Goodbye\n");
    }
}
