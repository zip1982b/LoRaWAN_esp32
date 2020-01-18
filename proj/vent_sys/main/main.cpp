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
#include "freertos/queue.h"
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


#define TIMER_DIVIDER         16  //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds
#define TIMER1_INTERVAL_SWITCH_ON_TRIAC   (0.00005) // for switch on TRIAC (sec) - 50uS
#define SPEED_1   (0.005)   // for firing angle to be 90' for 220V 50Hz AC signal, we need to have a delay of 5 ms
#define WITHOUT_RELOAD   0        
#define WITH_RELOAD      1       

/*
 * A sample structure to pass events
 * from the timer interrupt handler to the main program.
 */
typedef struct {
    int type;  // the type of timer's event
    int timer_group;
    int timer_idx;
    uint64_t timer_counter_value;
} timer_event_t;

typedef struct {
    uint8_t speed; //0, 1 ,2
} fan_event_t;



xQueueHandle xQueueDIM;
xQueueHandle xQueueISR;
xSemaphoreHandle xBinSemaphoreZS;
xSemaphoreHandle xBinSemaphoreT0;
xSemaphoreHandle xBinSemaphoreT1;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
	static portBASE_TYPE xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	if(gpio_num == ZERO_SENSOR){
		xSemaphoreGiveFromISR(xBinSemaphoreZS, &xHigherPriorityTaskWoken);
		if(xHigherPriorityTaskWoken)
		{
			taskYIELD_YIELD_FROM_ISR();
		}
	}
}




/*
 * Timer group0 ISR handler
 * switch on TRIAC
 * switch off TRIAC
 */
void IRAM_ATTR timer_group0_isr(void *para)
{
    timer_spinlock_take(TIMER_GROUP_0);
    int timer_idx = (int) para;
	uint32_t timer_intr = timer_group_get_intr_status_in_isr(TIMER_GROUP_0);
	
	static portBASE_TYPE xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	
	
	if (timer_intr & TIMER_INTR_T0) {
		timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_0);
		xSemaphoreGiveFromISR(xBinSemaphoreT0, &xHigherPriorityTaskWoken);
		if(xHigherPriorityTaskWoken)
		{
			taskYIELD_YIELD_FROM_ISR();
		}
	}
        
    } 
	else if (timer_intr & TIMER_INTR_T1) {
        timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_1);
		xSemaphoreGiveFromISR(xBinSemaphoreT1, &xHigherPriorityTaskWoken);
		if(xHigherPriorityTaskWoken)
		{
			taskYIELD_YIELD_FROM_ISR();
		}
    } 
    /* After the alarm has been triggered
      we need enable it again, so it is triggered the next time */
    timer_group_enable_alarm_in_isr(TIMER_GROUP_0, timer_idx);
    timer_spinlock_give(TIMER_GROUP_0);
}




/* very high priority task*/
static void  t_isr_handler_ZS(void* arg)
{
	/*
	0 - off
	1 - 50 % = 0.005mS
	2 - 100 % = 0.01mS
	*/
	
	/* Select and initialize basic parameters of the timer */
    timer_config_t config;
    config.divider = TIMER_DIVIDER;
    config.counter_dir = TIMER_COUNT_UP;
    config.counter_en = TIMER_PAUSE;
    config.alarm_en = TIMER_ALARM_EN;
    config.intr_type = TIMER_INTR_LEVEL;
    config.auto_reload = WITH_RELOAD;
#ifdef TIMER_GROUP_SUPPORTS_XTAL_CLOCK
    config.clk_src = TIMER_SRC_CLK_APB;
#endif
    timer_init(TIMER_GROUP_0, TIMER_0, &config);

    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL);

    /* Configure the alarm value and the interrupt on alarm. */
    //timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, TIMER0_INTERVAL_DELAY * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);
    timer_isr_register(TIMER_GROUP_0, TIMER_0, timer_group0_isr, (void *) TIMER_0, ESP_INTR_FLAG_IRAM, NULL);

	fan_event_t received_data;
	uint64_t alarm_value;
	received_data.speed = 0;
	
	for(;;){
		printf("Timer0 is configured  - Cicle - \n");
		xSemaphoreTake(xBinSemaphoreZS, portMAX_DELAY);
		xQueueReceive(xQueueDIM, &received_data, 0);
		switch(received_data.speed){
		case 0:
			gpio_set_level(FAN, 0); //FAN switch off
			break;
		case 1:
			alarm_value = (uint64_t) SPEED_1 * TIMER_SCALE; // FAN switch on - 50% speed
			printf('Alarm value [%d] ',  alarm_value);
			timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, alarm_value); // а до скольки считать?	
			timer_start(TIMER_GROUP_0, TIMER_0); //T0 start
			break;
		case 2:
			gpio_set_level(FAN, 1); //FAN switch on - 100% speed
			break;
		}
	}
}


/* very high priority task*/
static void  t_isr_handler_T0(void* arg)
{
	/*
	1 - gpio_set_level(FAN, 1);
	2 - T1 start;
	*/
	
	/* Select and initialize basic parameters of the timer */
    timer_config_t config;
    config.divider = TIMER_DIVIDER;
    config.counter_dir = TIMER_COUNT_UP;
    config.counter_en = TIMER_PAUSE;
    config.alarm_en = TIMER_ALARM_EN;
    config.intr_type = TIMER_INTR_LEVEL;
    config.auto_reload = WITH_RELOAD;
#ifdef TIMER_GROUP_SUPPORTS_XTAL_CLOCK
    config.clk_src = TIMER_SRC_CLK_APB;
#endif
    timer_init(TIMER_GROUP_0, TIMER_1, &config);

    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL);

    /* Configure the alarm value and the interrupt on alarm. */
    //timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, TIMER0_INTERVAL_DELAY * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, TIMER_1);
    timer_isr_register(TIMER_GROUP_0, TIMER_1, timer_group0_isr, (void *) TIMER_1, ESP_INTR_FLAG_IRAM, NULL);

	uint64_t alarm_value;
	
	for(;;){
		printf("[t_isr_handler_T0] Timer1 is configured  - Cicle - \n");
		xSemaphoreTake(xBinSemaphoreT0, portMAX_DELAY);
		alarm_value = (uint64_t) TIMER1_INTERVAL_SWITCH_ON_TRIAC * TIMER_SCALE; // FAN switch on - 50% speed
		printf('Alarm value [%d] ',  alarm_value);
		timer_set_alarm_value(TIMER_GROUP_0, TIMER_1, alarm_value); // а до скольки считать?	
		timer_start(TIMER_GROUP_0, TIMER_1); //T1 start
		gpio_set_level(FAN, 1); //FAN switch on 
		}
	}
}


/* very high priority task*/
static void  t_isr_handler_T1(void* arg)
{
	/*
	1 - gpio_set_level(FAN, 0);
	*/
	gpio_set_level(FAN, 0); //FAN switch off
}













static void venting(void* arg)
{
	fan_event_t send_data;
	printf("[venting task] send_data.timer_counter_value = [%d]\n", send_data.speed);
    send_data.speed = 0;
	uint8_t i = 0;
    for(;;) {
		while(i <= 2)
		{
			vTaskDelay(5000 / portTICK_RATE_MS);
			xQueueSendToBack(xQueueDIM, &send_data, portMAX_DELAY);
			send_data.speed = send_data.speed + i;
			i++;
		}
		send_data.speed = 0;
		i = 0;
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
		//xTaskCreate(venting, "fan_work", 1024 * 4,  NULL, 11, NULL);
    }
    else
    {
        printf("Join failed. Goodbye\n");
    }
	
	xQueueDIM = xQueueCreate(5, sizeof(fan_event_t));
	//tg0_timer_init(TIMER_0, WITH_RELOAD, TIMER0_INTERVAL_SWITCH_ON_TRIAC);
    //tg0_timer_init(TIMER_1, WITH_RELOAD, TIMER1_INTERVAL_DELAY);
	xTaskCreate(venting, "test_fan_work", 1024 * 4,  NULL, 11, NULL);
	
	vSemaphoreCreateBinary(xBinarySemaphore1);
	
	xTaskCreate(t_isr_handler_ZS, "task isr handler Zero Sensor", 1024 * 4,  NULL, 12, NULL);
	xTaskCreate(t_isr_handler_T0, "task isr handler T0", 1024 * 4,  NULL, 12, NULL);
	xTaskCreate(t_isr_handler_T1, "task isr handler T1", 1024 * 4,  NULL, 12, NULL);
}
