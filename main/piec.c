
// przekaźnik plusa czarny - IO21
// przekaźnik minusa biały - IO20
// VCC - brązowy 5 V
// masa - szary

// SCL-23
// SDA - 19


#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <math.h>
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "esp_log.h"

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"

//#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "sdkconfig.h"

#include "bme280.h"

#define TAG_BME280 "BME280"

#define SDA_PIN 19
#define SCL_PIN 23
#define BUTTON_GPIO GPIO_NUM_9


#define I2C_MASTER_ACK 0
#define I2C_MASTER_NACK 1

#define MIN_TEM_CHECK 20
#define DEBOUNCE_TIME_MS 50   
#define LONG_PRESS_TIME_MS 3000

#define BUTTON_GPIO0 GPIO_NUM_0
#define BUTTON_GPIO1 GPIO_NUM_1
#define BUTTON_GPIO2 GPIO_NUM_2
#define LONG_PRESS_MS 3000
#define DEBOUNCE_MS 50
#define DEFAULT_VALUE_T 18.0

typedef enum {
    SHORT_PRESS,
    LONG_PRESS
} ButtonEventType;

typedef struct {
    uint8_t gpio_num;
    ButtonEventType eventType;
} ButtonEvent;

typedef struct {
    float temp;
    float pres;
    float hum;
} SensorDataStruct;

volatile bool ble_advertise_status = true;
static float threshold_temperature; 
char *TAG = "BLE-Piec";
uint8_t ble_addr_type;
static float latest_temperature=0;
static float lowest_temperature=0;
static float highest_temperature=0;
//SemaphoreHandle_t temp_semaphore;
void ble_app_advertise(void);

QueueHandle_t buttonEventQueue =NULL;
TimerHandle_t debounceTimer0 =NULL, longPressTimer=NULL;
TaskHandle_t MeasureTaskHandle = NULL;
TaskHandle_t HeaterControlHandle = NULL;
TaskHandle_t DataSaveHandle = NULL;
QueueHandle_t gpio_evt_queue = NULL;
TaskHandle_t handler1=NULL;
QueueHandle_t Mailbox1 = NULL;
SemaphoreHandle_t buttonSemaphore = NULL;
//SemaphoreHandle_t button_sem;

// zamiana wartości w Float ieee11073 z aplikacji Nordic do Float zwykłego 
float ieee11073ToFloat32(uint32_t value) {
    // Rozbicie wartości na część ułamkową i całkowitą
    uint32_t fractionalPart = value & 0x000000FF; 
    uint32_t wholePart = (value >> 24) & 0x000000FF;  
    float result = wholePart + (fractionalPart / 100.0f); // Dzielimy przez 100, bo mamy dwie cyfry po przecinku
    return result;
}

// write and read data to flash memory
static int save_data(float *threshold_temperature, float *lowest_temperature, float *highest_temperature) {
    nvs_handle my_handle;
    esp_err_t err;

    err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK){
        return 1;
    } 
    else {
        //printf("SUCCESS OPEN DATA FROM NVS\n\r");
        ESP_LOGI(TAG , "SUCCESS SAVED DATA TO NVS");
    }

    uint32_t threshhold = *(uint32_t*)threshold_temperature;
    uint32_t lowest = *(uint32_t*)lowest_temperature;
    uint32_t highest = *(uint32_t*)highest_temperature;

    err = nvs_set_u32(my_handle, "threshhold", threshhold);
    if(err!=ESP_OK)
    {
        
        ///printf("FAILED TO WRITE threshhold\n\r");
        ESP_LOGE(TAG , "FAILED TO WRITE threshhold");
    }
    else {
        //printf("SUCCESS OPEN DATA FROM NVS\n\r");
        ESP_LOGI(TAG , "SUCCESS SAVED  threshhold DATA TO NVS");
    }

    err = nvs_set_u32(my_handle, "lowest", lowest);
     if(err!=ESP_OK)
    {
        //printf("FAILED TO WRITE lowest\n\r");
        ESP_LOGE(TAG , "FAILED TO WRITE lowest");
    }
    else {
        //printf("SUCCESS OPEN DATA FROM NVS\n\r");
        ESP_LOGI(TAG , "SUCCESS SAVED  lowest DATA TO NVS");
    }
    err = nvs_set_u32(my_handle, "highest", highest);
     if(err!=ESP_OK)
    {
        //printf("FAILED TO WRITE highest\n\r");
        ESP_LOGE(TAG , "FAILED TO WRITE threshhold");
    }
     else {
        //printf("SUCCESS OPEN DATA FROM NVS\n\r");
        ESP_LOGI(TAG , "SUCCESS SAVED highest DATA TO NVS");
    }
    nvs_commit(my_handle);
    nvs_close(my_handle);
    return 0;
 // druga opcja memcpy
    // memcpy(&threshhold, threshold_temperature, sizeof(float));
    // memcpy(&lowest, lowest_temperature, sizeof(float));
    // memcpy(&highest, highest_temperature, sizeof(float));

}

static int read_data(float *threshold_temperature, float *lowest_temperature, float *highest_temperature)
{   
    esp_err_t err;
    nvs_handle my_handle;
    float threshhold = DEFAULT_VALUE_T;
    float highest = DEFAULT_VALUE_T;
    float lowest = DEFAULT_VALUE_T;
    err = nvs_open("storage", NVS_READONLY, &my_handle);
    if(err!=ESP_OK){
        //printf("FAILED TO open NVS\n\r");
        ESP_LOGE(TAG , "CAN'T OPEN DATA FROM NVS");
    }
    else {
        //printf("SUCCESS OPEN DATA FROM NVS\n\r");
        ESP_LOGI(TAG , "SUCCESS OPEN DATA FROM NVS");
    }
    err = nvs_get_u32(my_handle, "threshhold", &threshhold);
    if(err!=ESP_OK){
        //printf("CAN'T READ threshhold FROM NVS\n\r");
        ESP_LOGE(TAG , "CAN'T READ threshhold FROM NVS");
    }
    else {
        //printf("CAN'T READ threshhold FROM NVS\n\r");
        ESP_LOGI(TAG , "SUCESS READ threshhold FROM NVS");
    }
    err = nvs_get_u32(my_handle, "highest", &highest);
    if(err!=ESP_OK){
        //printf("CAN'T READ highest FROM NVS\n\r");
        ESP_LOGE(TAG , "CAN'T READ highest FROM NVS");
    }
    else {
        //printf("SUCESS READ  highest FROM NVS\n\r");
        ESP_LOGI(TAG , "SUCESS READ highest FROM NVS");
    }
    err = nvs_get_u32(my_handle, "lowest", &lowest);
    if(err!=ESP_OK){
        //printf("CAN'T READ lowest  FROM NVS\n\r");
        ESP_LOGE(TAG , "CAN'T READ lowest FROM NVS");
    }
    else {
        //printf("SUCESS READ lowest  FROM NVS\n\r");
        ESP_LOGI(TAG , "SUCESS READ lowest FROM NVS");
    }
    *threshold_temperature = threshhold;
    *lowest_temperature = lowest;
    *highest_temperature = highest;
    nvs_close(my_handle);
    return 0;
}

// Write data to ESP32 defined as server
static int threshold_temperature_write(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    // Ensure the data size is correct (e.g., sizeof(float))
    if (ctxt->om->om_len == sizeof(uint32_t )) {
        uint32_t new_threshold;
        memcpy(&new_threshold, ctxt->om->om_data, ctxt->om->om_len);
        //ESP_LOGE(TAG, "New threshold temperature set: 0X%X", (unsigned int)new_threshold);
        threshold_temperature = ieee11073ToFloat32(new_threshold);
        ESP_LOGI(TAG, "New threshold temperature set: %.2f", threshold_temperature);
        return 0;
    } else {
        ESP_LOGE(TAG, "Invalid data size for threshold temperature");
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }
}


// Read data from ESP32 defined as server
static int device_read(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    char temp_str_l[200];
    char temp_str_lo[50];
    char temp_str_h[50];
    char temp_str_tre[50];
    //xSemaphoreTake(temp_semaphore, portMAX_DELAY);
    snprintf(temp_str_l, sizeof(temp_str_l), "Rec temp: %.2f ", latest_temperature);
    snprintf(temp_str_lo, sizeof(temp_str_lo), "Lowest temp: %.2f ", lowest_temperature);
    snprintf(temp_str_h, sizeof(temp_str_h), "Highest temp: %.2f ", highest_temperature);
    snprintf(temp_str_tre, sizeof(temp_str_tre), " Threshold temp: %.2f ",threshold_temperature);
    strcat(temp_str_l,temp_str_lo);
    strcat(temp_str_l,temp_str_h);
    strcat(temp_str_l,temp_str_tre);
    //xSemaphoreGive(temp_semaphore);
    os_mbuf_append(ctxt->om, temp_str_l, strlen(temp_str_l));
    return 0;
}

// Array of pointers to other service definitions
// UUID - Universal Unique Identifier
static const struct ble_gatt_svc_def gatt_svcs[] = {
    {.type = BLE_GATT_SVC_TYPE_PRIMARY,
     .uuid = BLE_UUID16_DECLARE(0x180),                 // Define UUID for device type
     .characteristics = (struct ble_gatt_chr_def[]){
         {.uuid = BLE_UUID16_DECLARE(0xFEF4),           // Define UUID for reading
          .flags = BLE_GATT_CHR_F_READ,
          .access_cb = device_read},
         {.uuid = BLE_UUID16_DECLARE(0xDEAD),           // Define UUID for writing
          .flags = BLE_GATT_CHR_F_WRITE,
          .access_cb = threshold_temperature_write},
         {0}}},
    {0}};

// BLE event handling
static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type)
    {
    // Advertise if connected
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI("GAP", "BLE GAP EVENT CONNECT %s", event->connect.status == 0 ? "OK!" : "FAILED!");
        if (event->connect.status != 0)
        {
            ble_advertise_status=true;
            ble_app_advertise();
        }
        break;
    // Advertise again after completion of the event
    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI("GAP", "BLE GAP EVENT");
        ble_advertise_status=true;
        ble_app_advertise();
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI("GAP", "BLE GAP EVENT DISCONNECTED");
        ble_advertise_status=true;
        ble_app_advertise();
        break;
    default:
        break;
    }
    return 0;
}

// Define the BLE connection
void ble_app_advertise(void)
{
    // GAP - device name definition
    struct ble_hs_adv_fields fields;
    const char *device_name;
    memset(&fields, 0, sizeof(fields));
    device_name = ble_svc_gap_device_name(); // Read the BLE device name
    fields.name = (uint8_t *)device_name;
    fields.name_len = strlen(device_name);
    fields.name_is_complete = 1;
    ble_gap_adv_set_fields(&fields);

    // GAP - device connectivity definition
    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    if(ble_advertise_status==true){
        adv_params.conn_mode = BLE_GAP_CONN_MODE_UND; // connectable
        adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN; // discoverable
        ble_gap_adv_start(ble_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
    }
    else
    {
        adv_params.conn_mode = BLE_GAP_CONN_MODE_NON; // non-connectable
        adv_params.disc_mode = BLE_GAP_DISC_MODE_NON ; // non-discoverable
    }
    
}

// The application
void ble_app_on_sync(void)
{
    ble_hs_id_infer_auto(0, &ble_addr_type); // Determines the best address type automatically
    ble_app_advertise();                     // Define the BLE connection
}

// The infinite task
void host_task(void *param)
{
    nimble_port_run(); // This function will return only when nimble_port_stop() is executed
}

#if 1 //#######################################################################################################################

// Initialize I2C communication parameters
void i2c_master_init()
{
	i2c_config_t i2c_config = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = SDA_PIN,
		.scl_io_num = SCL_PIN,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = 1000000};
	i2c_param_config(I2C_NUM_0, &i2c_config);
	i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
}

// BME280 I2C write function
s8 BME280_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 iError = BME280_INIT_VALUE;

	esp_err_t espRc;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);

	i2c_master_write_byte(cmd, reg_addr, true);
	i2c_master_write(cmd, reg_data, cnt, true);
	i2c_master_stop(cmd);

	espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
	if (espRc == ESP_OK)
	{
		iError = SUCCESS;
	}
	else
	{
		iError = ERROR;
	}
	i2c_cmd_link_delete(cmd);

	return (s8)iError;
}

// BME280 I2C read function
s8 BME280_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 iError = BME280_INIT_VALUE;
	esp_err_t espRc;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, reg_addr, true);

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);

	if (cnt > 1)
	{
		i2c_master_read(cmd, reg_data, cnt - 1, I2C_MASTER_ACK);
	}
	i2c_master_read_byte(cmd, reg_data + cnt - 1, I2C_MASTER_NACK);
	i2c_master_stop(cmd);

	espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
	if (espRc == ESP_OK)
	{
		iError = SUCCESS;
	}
	else
	{
		iError = ERROR;
	}

	i2c_cmd_link_delete(cmd);

	return (s8)iError;
}

// BME280 I2C delay function
void BME280_delay_msek(u32 msek)
{
	vTaskDelay(msek / portTICK_PERIOD_MS);
}

#endif
//##########################################################################################################################
static void IRAM_ATTR button_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t) arg;
    if(gpio_num == BUTTON_GPIO0) {
        xTimerStartFromISR(debounceTimer0, NULL);
        xTimerStartFromISR(longPressTimer, NULL);
    }
}

void gpio_configure1(void)
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = ((1<<20) | (1<<21) ); //relays
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1<<BUTTON_GPIO);
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    gpio_isr_handler_add(BUTTON_GPIO, button_isr_handler, NULL);
}


static void TimerCallback(TimerHandle_t xTimer) {
    uint32_t timer_id = (uint32_t) pvTimerGetTimerID(xTimer);
    ButtonEvent event;

    if ((timer_id == 0) && (gpio_get_level(BUTTON_GPIO) == 0)) {  
        event.eventType = SHORT_PRESS;
        }
    else if ((timer_id == 1)&&(gpio_get_level(BUTTON_GPIO) == 0)) { 
            event.eventType = LONG_PRESS;
        } 
        xSemaphoreGiveFromISR(buttonSemaphore, NULL);
        xQueueSendToBackFromISR(buttonEventQueue, &event, portMAX_DELAY);
}

static void buttonTask(void* arg) {
    QueueHandle_t buttonEventQueue = (QueueHandle_t )arg;
    ButtonEvent event;
    while(1) {
        if(xSemaphoreTake(buttonSemaphore, portMAX_DELAY)){
            xQueueReceive(buttonEventQueue, &event, portMAX_DELAY);
            if(event.eventType == LONG_PRESS) {
                ble_advertise_status = true;
                printf("DLUGIE WCISNIECIE,BLE ACTIVE \n\r");
                
            } else if(event.eventType == SHORT_PRESS) {
                printf("KROTKIE WCISNIECIE WCISNIECIE\n\r");
            }
        }
        }
    }

// ##### OLD VERSION
// void button_task(void* arg) {
//     int64_t start_time = 0;
//     int64_t end_time;
//     int64_t duration;
//     while (1) {
//         if (xQueueReceive(gpio_evt_queue, &end_time, portMAX_DELAY)) {
//             if (gpio_get_level(BUTTON_GPIO) == 0) {
//                 printf("wcisnieto przycisk\n\r");
//                 start_time = end_time;
//             } else {
//                 // Button released
//                 duration = end_time - start_time;
//                 if (duration > 3000000) {
//                      ble_advertise_status = true;
//                      printf("ustawiono bluetooth\n\r");
//                 } else {
//                     ble_advertise_status = false;
//                     printf("bt status - false");
//                 }
//             }
//         }
//     }

// }


void powerON(void)
{
    gpio_set_level(21,1);
    gpio_set_level(20,1);
}

void powerOFF(void)
{
    gpio_set_level(21,0);
    gpio_set_level(20,0);
}
#if 0
void MeasureTask(void *arg)
{
    
    QueueHandle_t Mailbox1 = (QueueHandle_t)arg;
    SensorDataStruct Data;
    BaseType_t xStatus;

    struct bme280_t bme280 = {
		.bus_write = BME280_I2C_bus_write,
		.bus_read = BME280_I2C_bus_read,
		.dev_addr = BME280_I2C_ADDRESS1,
		.delay_msec = BME280_delay_msek};

	s32 com_rslt;
	s32 v_uncomp_pressure_s32;
	s32 v_uncomp_temperature_s32;
	s32 v_uncomp_humidity_s32;

	// Initialize BME280 sensor and set internal parameters
	com_rslt = bme280_init(&bme280);
	printf("com_rslt %d\n", com_rslt);

	com_rslt += bme280_set_oversamp_pressure(BME280_OVERSAMP_16X);
	com_rslt += bme280_set_oversamp_temperature(BME280_OVERSAMP_2X);
	com_rslt += bme280_set_oversamp_humidity(BME280_OVERSAMP_1X);

	com_rslt += bme280_set_standby_durn(BME280_STANDBY_TIME_1_MS);
	com_rslt += bme280_set_filter(BME280_FILTER_COEFF_16);

	com_rslt += bme280_set_power_mode(BME280_NORMAL_MODE);
	
    if (com_rslt == SUCCESS)
	{
		while (true)
		{
			vTaskDelay(1000 / portTICK_PERIOD_MS);

			// Read BME280 data
			com_rslt = bme280_read_uncomp_pressure_temperature_humidity(
				&v_uncomp_pressure_s32, &v_uncomp_temperature_s32, &v_uncomp_humidity_s32);

			Data.temp = bme280_compensate_temperature_double(v_uncomp_temperature_s32);

			Data.pres = bme280_compensate_pressure_double(v_uncomp_pressure_s32) / 100; // Pa -> hPa

			Data.hum = bme280_compensate_humidity_double(v_uncomp_humidity_s32);
            
			// Print BME data
			if (com_rslt == SUCCESS)
			{
				printf("Temperature %f\n\r",Data.temp);
                //printf("Humidity %f/n/r",Data.hum); // BMP280 don't have hum available
                printf("Pressure %f\n\r",Data.pres);
			}
			else
			{
				ESP_LOGE(TAG_BME280, "measure error. code: %d", com_rslt);
			}

            xStatus =xQueueOverwrite(Mailbox1,&Data);
            if(xStatus!=pdPASS){
            printf("Can't send data\n\r");
            }
            else{
            printf("Data has been sent\n\r");
            }
			// 
			latest_temperature = Data.temp;
    		//xSemaphoreGive(temp_semaphore); 
            if(latest_temperature < lowest_temperature){
            lowest_temperature = latest_temperature;
            }
            if(latest_temperature > highest_temperature){
            highest_temperature = latest_temperature;
            }
            save_data(&threshold_temperature, &lowest_temperature, &highest_temperature);
            vTaskDelay(1000*60*MIN_TEM_CHECK/portTICK_PERIOD_MS);
		}

	}
	else
	{
		ESP_LOGE(TAG_BME280, "init or setting error. code: %d", com_rslt);
	}
	
}
#endif

///// dummy MeasureTask
#if 1
void MeasureTask(void *arg)
{
    
    QueueHandle_t Mailbox1 = (QueueHandle_t)arg;
    SensorDataStruct Data;
while(1){
    Data.temp = 20.0;
    Data.hum = 30.0;
    Data.pres = 1000.0;
    xQueueOverwrite(Mailbox1,&Data);
    latest_temperature = Data.temp;
    if(latest_temperature < lowest_temperature){
        lowest_temperature = latest_temperature;
    }
    if(latest_temperature > highest_temperature){
        highest_temperature = latest_temperature;
    }
    //xSemaphoreGive(temp_semaphore); 
    save_data(&threshold_temperature, &lowest_temperature, &highest_temperature);
    vTaskDelay(1000*60*1/portTICK_PERIOD_MS);
}
}
#endif

void HeaterControl(void *arg)
{
    QueueHandle_t Mailbox1 = (QueueHandle_t)arg;
    SensorDataStruct Data;
    BaseType_t xStatus;

    while(1){
         xStatus = xQueuePeek(Mailbox1,&Data,portMAX_DELAY);
                if(xStatus!=pdPASS)
                {
                    printf("ERROR RECEIVE DATA !\n\r");
                }
                else 
                {
                    printf("SUCCESFULLY RECEIVED\n\r");
                    if(Data.temp>(threshold_temperature+1))
                    {
                        powerOFF();
                        printf("TEMP TOO HIGH, POWER OFF\n\r");
                    }
                    else if(Data.temp<(threshold_temperature-1))
                    {
                        powerON();
                        printf("TEMP TOO LOW, POWER ON\n\r");
                    }
                    else
                    {
                        printf("TEMP THRESHOLD ACHIEVED %f \n\r",Data.temp);
                    }
                }
                vTaskDelay(1000*60*1/portTICK_PERIOD_MS);
            }
        
}

// void Task3(void *arg)
// {
//     QueueHandle_t xQueue1 = (QueueHandle_t)arg;
//     int i;
    
//     while(1){
        
//         xQueueSendToBack(xQueue1,&i,0);
//         printf("Wysłano %d z taska3\n\r",i);
        
//         vTaskDelay(333/portTICK_PERIOD_MS);
//         i--;
//     }

// }

// void Task4(void *arg)
// {
//     QueueHandle_t xQueue1 = (QueueHandle_t)arg;
//     int i=10000;
//     while(1){
//         xQueueSendToBack(xQueue1,&i,0);
//         printf("Wysłano %d z taska4\n\r",i);
//         vTaskDelay(1000/portTICK_PERIOD_MS);
//     }

// }


void app_main(void)
{

    //esp_err_t ret = 
    nvs_flash_init();
	// if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
	// {
	// 	ESP_ERROR_CHECK(nvs_flash_erase());
	// 	ret = nvs_flash_init();
	// }
	//temp_semaphore = xSemaphoreCreateMutex();
    read_data(&threshold_temperature, &lowest_temperature, &highest_temperature);
	//ESP_ERROR_CHECK(ret);
	//esp_nimble_hci_and_controller_init();      // 2 - Initialize ESP controller
    nimble_port_init();                        // 3 - Initialize the host stack
    ble_svc_gap_device_name_set("BLE-Piec"); // 4 - Initialize NimBLE configuration - server name
    ble_svc_gap_init();                        // 4 - Initialize NimBLE configuration - gap service
    ble_svc_gatt_init();                       // 4 - Initialize NimBLE configuration - gatt service
    ble_gatts_count_cfg(gatt_svcs);            // 4 - Initialize NimBLE configuration - config gatt services
    ble_gatts_add_svcs(gatt_svcs);             // 4 - Initialize NimBLE configuration - queues gatt services.
    ble_hs_cfg.sync_cb = ble_app_on_sync;      // 5 - Initialize application
    nimble_port_freertos_init(host_task);      // 6 - Run the thread
	gpio_configure1();

	#if 0
    i2c_master_init();
    #endif
    //button_sem = xSemaphoreCreateBinary();
    //gpio_evt_queue = xQueueCreate(10, sizeof(int64_t));
    buttonEventQueue = xQueueCreate(10, sizeof(ButtonEvent));
    buttonSemaphore = xSemaphoreCreateBinary();
    xTaskCreate(buttonTask, "button_task", 2048, (void *)buttonEventQueue, 10, &handler1);
    Mailbox1 = xQueueCreate(1,sizeof(int));
    if(Mailbox1!=NULL){
        printf("Mailbox1 created\n\r");
        xTaskCreate(MeasureTask,"MeasureTask",1024*5,(void *)Mailbox1,2,&MeasureTaskHandle);
        xTaskCreate(HeaterControl,"HeaterControl",1024*5,(void *)Mailbox1,1,&HeaterControlHandle);
        
        // xTaskCreate(Task2,"Task2",1024*5,(void *)xQueue1,1,&Task2Handle); TBA
    
    }
    else{
        printf("Mailbox1 NOT CREATED\n\r");
    }
        
       debounceTimer0 = xTimerCreate("debounceTimer", pdMS_TO_TICKS(DEBOUNCE_MS), pdFALSE, (void *)0, TimerCallback);
       longPressTimer = xTimerCreate("longPressTimer", pdMS_TO_TICKS(LONG_PRESS_MS), pdFALSE, (void *)1, TimerCallback);
}
