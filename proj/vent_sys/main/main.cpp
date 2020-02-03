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
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <unistd.h>
#include "esp_timer.h"

#include "esp_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"

#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "driver/periph_ctrl.h"

#include "TheThingsNetwork.h"

//#include "DHT.h"



#define DHT_OK 0
#define DHT_CHECKSUM_ERROR -1
#define DHT_TIMEOUT_ERROR -2

// == function prototypes =======================================

void setDHTgpio(gpio_num_t gpio);
void errorHandler(int response);
int readDHT();
float getHumidity();
float getTemperature();
int getSignalLevel(int usTimeOut, bool state);

gpio_num_t DHTgpio = GPIO_NUM_33; // my default DHT pin = 4
float humidity = 0.;
float temperature = 0.;



static void startingTRIAC_timer_callback(void* arg);
static void delay_timer_callback(void* arg);

static const char* TAG_lora = "LoRaWAN";
static const char* TAG_vent = "VentSys";
static const char* TAG_dht = "DHT22";

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
 
#ifdef __cplusplus
extern "C"{
#endif

#define FAN				32
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<FAN))
#define ZERO_SENSOR			34
#define GPIO_INPUT_PIN_SEL  ((1ULL<<ZERO_SENSOR))
#define ESP_INTR_FLAG_DEFAULT 0

#ifdef __cplusplus
} // extern "C"
#endif





typedef struct {
    uint8_t speed; //0, 1 ,2
} fan_event_t;


portBASE_TYPE xStatus_venting;
portBASE_TYPE xStatus_task_isr_handler_ZS;

xQueueHandle xQueueDIM;
xSemaphoreHandle xBinSemaphoreZS;




 static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
	static portBASE_TYPE xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	if(gpio_num == ZERO_SENSOR){
		xSemaphoreGiveFromISR(xBinSemaphoreZS, &xHigherPriorityTaskWoken);
		if(xHigherPriorityTaskWoken)
		{
			portYIELD_FROM_ISR();
		}
	}
}





/* very high priority task*/
static void  task_isr_handler_ZS(void* arg)
{
	UBaseType_t uxPriority;
	uxPriority = uxTaskPriorityGet(NULL);
	ESP_LOGI(TAG_vent, "[task_isr_handler_ZS] Priority get = [%d]",  (uint8_t)uxPriority);
	
	/* Create a one-shot timer for starting TRIAC */
	const esp_timer_create_args_t startingTRIAC_timer_args = {
            .callback = &startingTRIAC_timer_callback,
            /* name is optional, but may help identify the timer when debugging */
            .name = "starting TRIAC"
    };
    esp_timer_handle_t startingTRIAC_timer;
    ESP_ERROR_CHECK(esp_timer_create(&startingTRIAC_timer_args, &startingTRIAC_timer));
	
	
	
	/* Create a one-shot timer for delay RMS */
	const esp_timer_create_args_t delay_timer_args = {
            .callback = &delay_timer_callback,
			.arg = (void*) startingTRIAC_timer,
            .name = "delay timer"
    };
	esp_timer_handle_t delay_timer;
    ESP_ERROR_CHECK(esp_timer_create(&delay_timer_args, &delay_timer));
	
	
	uint32_t io_num;
	fan_event_t received_data;
	uint8_t speed = 0;
	ESP_LOGI(TAG_vent, "[task_isr_handler_ZS]esp_timer is configured");
	
	for(;;){
		//ESP_LOGI(TAG, "[task_isr_handler_ZS] - Cicle -");
		
		xSemaphoreTake(xBinSemaphoreZS, portMAX_DELAY);
		//xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY);
			//ESP_LOGI(TAG, "[task_isr_handler_ZS] ionum = %d", io_num);
		
		
		xQueueReceive(xQueueDIM, &received_data, 0);
		speed = received_data.speed;
		
		//ESP_LOGI(TAG, "[task_isr_handler_ZS] speed = %d", speed);
		switch(speed){
		case 0:
			gpio_set_level(GPIO_NUM_32, 0); //FAN switch off
			//ESP_LOGI(TAG, "[task_isr_handler_ZS] Fan off");
			break;
		case 1:
			/* Start the one-shot timer */
			esp_timer_start_once(delay_timer, 5000);
			//ESP_LOGI(TAG, "[task_isr_handler_ZS] Started timer, time since boot: %lld us", esp_timer_get_time());
			break;
		case 2:
			gpio_set_level(GPIO_NUM_32, 1); //FAN switch on - 100% speed
			//ESP_LOGI(TAG, "[task_isr_handler_ZS] FAN ON speed = 2");
			break;
		}
	}
}








void DHT_task(void *pvParameter)
{
	fan_event_t send_data;
    setDHTgpio(GPIO_NUM_33);
    ESP_LOGI(TAG_dht, "Starting DHT Task\n\n");
	float H;
	float T;
	float target_humidity = 30.2;// %
	uint8_t delta = 2;
	
	
    while (1)
    {
        //ESP_LOGI(TAG_dht, "=== Reading DHT ===\n");
        int ret = readDHT();

        errorHandler(ret);
		
		H = getHumidity();
		T = getTemperature();
		
        ESP_LOGI(TAG_dht, "Hum: %.1f Tmp: %.1f", H, T);
		
		if(H <= target_humidity - delta){
			ESP_LOGI(TAG_dht, "[DHT_task] Fan speed = 1");
			send_data.speed = 1;
			xQueueSendToBack(xQueueDIM, &send_data, portMAX_DELAY);
		}
		else if(H > target_humidity + delta){
			ESP_LOGI(TAG_dht, "[DHT_task] Fan speed = 0");
			send_data.speed = 0;
			xQueueSendToBack(xQueueDIM, &send_data, portMAX_DELAY);
		}
        // -- wait at least 2 sec before reading again ------------
        // The interval of whole process must be beyond 2 seconds !!
        vTaskDelay(3000 / portTICK_RATE_MS);
    }
}




static TheThingsNetwork ttn;

const unsigned TX_INTERVAL = 30;
static uint8_t msgData[] = "Hello, world";


void sendMessages(void* pvParameter)
{
    while (1) {
		ESP_LOGI(TAG_lora, "[sendMessages] Sending message...");
        //printf("Sending message...\n");
        TTNResponseCode res = ttn.transmitMessage(msgData, sizeof(msgData) - 1);
        printf(res == kTTNSuccessfulTransmission ? "Message sent.\n" : "Transmission failed.\n");
        vTaskDelay(TX_INTERVAL * 1000 / portTICK_PERIOD_MS);
    }
}






extern "C" void app_main(void)
{
	gpio_config_t io_conf;
	/*** triac control ***/
    io_conf.intr_type = GPIO_INTR_DISABLE; //disable interrupt
    io_conf.mode = GPIO_MODE_OUTPUT; //set as output mode
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL; //bit mask of the pins that you want to set,e.g.GPIO32/33
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE; //disable pull-down mode
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE; //disable pull-up mode
    gpio_config(&io_conf); //configure GPIO with the given settings
	/*********************/
	
	/*** zero sensor ***/
    io_conf.intr_type = GPIO_INTR_ANYEDGE; //interrupt ANYEDGE
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL; //bit mask of the pins, use GPIO34 here
    io_conf.mode = GPIO_MODE_INPUT; //set as input mode
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE; //enable pull-up mode
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE; //disable pull-down_cw mode - отключитли подтяжку к земле
    gpio_config(&io_conf);
	/*********************/
	
	//install gpio isr service
    //gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    //gpio_isr_handler_add(ZERO_SENSOR, gpio_isr_handler, (void*) ZERO_SENSOR);
	
    esp_err_t err;
    // Initialize the GPIO ISR handler service
    err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
	gpio_isr_handler_add(GPIO_NUM_32, gpio_isr_handler, (void*) GPIO_NUM_32);
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
		ESP_LOGI(TAG_lora, "Joined");
        
        xTaskCreate(sendMessages, "send_messages", 1024 * 4, (void* )0, 3, nullptr);
		vSemaphoreCreateBinary(xBinSemaphoreZS);
		xQueueDIM = xQueueCreate(5, sizeof(fan_event_t));
	
	
		xStatus_task_isr_handler_ZS = xTaskCreate(task_isr_handler_ZS, "task isr handler Zero Sensor", 1024 * 4,  NULL, 8, NULL); //&xZS_Handle
		if(xStatus_task_isr_handler_ZS == pdPASS)
			ESP_LOGI(TAG_vent, "[app_main] Task [task isr handler Zero Sensor] is created");
		else
			ESP_LOGI(TAG_vent, "[app_main] Task [task isr handler Zero Sensor] is not created");

		xTaskCreate(&DHT_task, "DHT_task", 2048, NULL, 5, NULL);
    }
    else
    {
		ESP_LOGE(TAG_lora, "Join failed. Goodbye");
    }
}


static void delay_timer_callback(void* arg)
{
    //int64_t time_since_boot = esp_timer_get_time();
    esp_timer_handle_t startingTRIAC_timer_handle = (esp_timer_handle_t) arg;
	gpio_set_level(GPIO_NUM_32, 1); //FAN switch on 
	//ESP_LOGI(TAG, "[task_isr_handler_ZS] Fan on half");
    /* To start the timer which is running, need to stop it first */
    ESP_ERROR_CHECK(esp_timer_start_once(startingTRIAC_timer_handle, 50));
    //ESP_LOGI(TAG, "[delay_timer_callback] startingTRIAC_timer start once");
}


static void startingTRIAC_timer_callback(void* arg)
{
	gpio_set_level(GPIO_NUM_32, 0); //FAN switch off
}


// == set the DHT used pin=========================================

void setDHTgpio(gpio_num_t gpio)
{
    DHTgpio = gpio;
}

// == get temp & hum =============================================

float getHumidity() { return humidity; }
float getTemperature() { return temperature; }

// == error handler ===============================================

void errorHandler(int response)
{
    switch (response)
    {

    case DHT_TIMEOUT_ERROR:
        ESP_LOGE(TAG_dht, "Sensor Timeout\n");
        break;

    case DHT_CHECKSUM_ERROR:
        ESP_LOGE(TAG_dht, "CheckSum error\n");
        break;

    case DHT_OK:
        break;

    default:
        ESP_LOGE(TAG_dht, "Unknown error\n");
    }
}

/*-------------------------------------------------------------------------------
;
;	get next state 
;
;	I don't like this logic. It needs some interrupt blocking / priority
;	to ensure it runs in realtime.
;
;--------------------------------------------------------------------------------*/

int getSignalLevel(int usTimeOut, bool state)
{

    int uSec = 0;
    while (gpio_get_level(DHTgpio) == state)
    {

        if (uSec > usTimeOut)
            return -1;

        ++uSec;
        ets_delay_us(1); // uSec delay
    }

    return uSec;
}

/*----------------------------------------------------------------------------
;
;	read DHT22 sensor

copy/paste from AM2302/DHT22 Docu:

DATA: Hum = 16 bits, Temp = 16 Bits, check-sum = 8 Bits

Example: MCU has received 40 bits data from AM2302 as
0000 0010 1000 1100 0000 0001 0101 1111 1110 1110
16 bits RH data + 16 bits T data + check sum

1) we convert 16 bits RH data from binary system to decimal system, 0000 0010 1000 1100 → 652
Binary system Decimal system: RH=652/10=65.2%RH

2) we convert 16 bits T data from binary system to decimal system, 0000 0001 0101 1111 → 351
Binary system Decimal system: T=351/10=35.1°C

When highest bit of temperature is 1, it means the temperature is below 0 degree Celsius. 
Example: 1000 0000 0110 0101, T= minus 10.1°C: 16 bits T data

3) Check Sum=0000 0010+1000 1100+0000 0001+0101 1111=1110 1110 Check-sum=the last 8 bits of Sum=11101110

Signal & Timings:

The interval of whole process must be beyond 2 seconds.

To request data from DHT:

1) Sent low pulse for > 1~10 ms (MILI SEC)
2) Sent high pulse for > 20~40 us (Micros).
3) When DHT detects the start signal, it will pull low the bus 80us as response signal, 
   then the DHT pulls up 80us for preparation to send data.
4) When DHT is sending data to MCU, every bit's transmission begin with low-voltage-level that last 50us, 
   the following high-voltage-level signal's length decide the bit is "1" or "0".
	0: 26~28 us
	1: 70 us

;----------------------------------------------------------------------------*/

#define MAXdhtData 5 // to complete 40 = 5*8 Bits

int readDHT()
{
    int uSec = 0;

    uint8_t dhtData[MAXdhtData];
    uint8_t byteInx = 0;
    uint8_t bitInx = 7;

    for (int k = 0; k < MAXdhtData; k++)
        dhtData[k] = 0;

    // == Send start signal to DHT sensor ===========

    gpio_set_direction(DHTgpio, GPIO_MODE_OUTPUT);

    // pull down for 3 ms for a smooth and nice wake up
    gpio_set_level(DHTgpio, 0);
    ets_delay_us(3000);

    // pull up for 25 us for a gentile asking for data
    gpio_set_level(DHTgpio, 1);
    ets_delay_us(25);

    gpio_set_direction(DHTgpio, GPIO_MODE_INPUT); // change to input mode

    // == DHT will keep the line low for 80 us and then high for 80us ====

    uSec = getSignalLevel(85, 0);
    ESP_LOGD(TAG_dht, "Response = %d", uSec);
    if (uSec < 0)
        return DHT_TIMEOUT_ERROR;

    // -- 80us up ------------------------

    uSec = getSignalLevel(85, 1);
    ESP_LOGD(TAG_dht, "Response = %d", uSec);
    if (uSec < 0)
        return DHT_TIMEOUT_ERROR;

    // == No errors, read the 40 data bits ================

    for (int k = 0; k < 40; k++)
    {

        // -- starts new data transmission with >50us low signal

        uSec = getSignalLevel(56, 0);
        if (uSec < 0)
            return DHT_TIMEOUT_ERROR;

        // -- check to see if after >70us rx data is a 0 or a 1

        uSec = getSignalLevel(75, 1);
        if (uSec < 0)
            return DHT_TIMEOUT_ERROR;

        // add the current read to the output data
        // since all dhtData array where set to 0 at the start,
        // only look for "1" (>28us us)

        if (uSec > 40)
        {
            dhtData[byteInx] |= (1 << bitInx);
        }

        // index to next byte

        if (bitInx == 0)
        {
            bitInx = 7;
            ++byteInx;
        }
        else
            bitInx--;
    }

    // == get humidity from Data[0] and Data[1] ==========================

    humidity = dhtData[0];
    humidity *= 0x100; // >> 8
    humidity += dhtData[1];
    humidity /= 10; // get the decimal

    // == get temp from Data[2] and Data[3]

    temperature = dhtData[2] & 0x7F;
    temperature *= 0x100; // >> 8
    temperature += dhtData[3];
    temperature /= 10;

    if (dhtData[2] & 0x80) // negative temp, brrr it's freezing
        temperature *= -1;

    // == verify if checksum is ok ===========================================
    // Checksum is the sum of Data 8 bits masked out 0xFF

    if (dhtData[4] == ((dhtData[0] + dhtData[1] + dhtData[2] + dhtData[3]) & 0xFF))
        return DHT_OK;

    else
        return DHT_CHECKSUM_ERROR;
}

