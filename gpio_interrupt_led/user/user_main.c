/*
 * ESPRSSIF MIT License
 *
 * Copyright (c) 2015 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 *
 * Permission is hereby granted for use on ESPRESSIF SYSTEMS ESP8266 only, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#include "esp_common.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "../include/gpio.h"


xTaskHandle key_handler_task_handle;

/******************************************************************************
 * FunctionName : user_rf_cal_sector_set
 * Description  : SDK just reversed 4 sectors, used for rf init data and paramters.
 *                We add this function to force users to set rf cal sector, since
 *                we don't know which sector is free in user's application.
 *                sector map for last several sectors : ABCCC
 *                A : rf cal
 *                B : rf init data
 *                C : sdk parameters
 * Parameters   : none
 * Returns      : rf cal sector
*******************************************************************************/
uint32 user_rf_cal_sector_set(void)
{
    flash_size_map size_map = system_get_flash_size_map();
    uint32 rf_cal_sec = 0;

    switch (size_map) {
        case FLASH_SIZE_4M_MAP_256_256:
            rf_cal_sec = 128 - 5;
            break;

        case FLASH_SIZE_8M_MAP_512_512:
            rf_cal_sec = 256 - 5;
            break;

        case FLASH_SIZE_16M_MAP_512_512:
        case FLASH_SIZE_16M_MAP_1024_1024:
            rf_cal_sec = 512 - 5;
            break;

        case FLASH_SIZE_32M_MAP_512_512:
        case FLASH_SIZE_32M_MAP_1024_1024:
            rf_cal_sec = 1024 - 5;
            break;
        case FLASH_SIZE_64M_MAP_1024_1024:
            rf_cal_sec = 2048 - 5;
            break;
        case FLASH_SIZE_128M_MAP_1024_1024:
            rf_cal_sec = 4096 - 5;
            break;
        default:
            rf_cal_sec = 0;
            break;
    }

    return rf_cal_sec;
}

void led_init(void){
	GPIO_ConfigTypeDef gpio_in_cfg;    //Define GPIO Init Structure
	gpio_in_cfg.GPIO_IntrType = GPIO_PIN_INTR_NEGEDGE;    //
	gpio_in_cfg.GPIO_Mode = GPIO_Mode_Output;    //Input mode
	gpio_in_cfg.GPIO_Pullup = GPIO_PullUp_EN;
	gpio_in_cfg.GPIO_Pin = GPIO_Pin_0;    // Enable GPIO
	gpio_config(&gpio_in_cfg);    //Initialization function
}

void led_toggle(void){
	uint32_t bit;
	bit = GPIO_INPUT_GET(0);
	GPIO_OUTPUT(GPIO_Pin_0, bit ^ 0x0000001 );
	if ((bit & 0x0000001) == 0x01){
		printf("led on \n");
	}else{
		printf("led of \n");
	}
}

void led_toggle_task(void  *pvParameters)
{
	led_init();
    for(; ; ){
    	led_toggle();
    	printf("led toggle \n");
    	vTaskDelay(100);
    }
    vTaskDelete(NULL);
}

void key_interrupt(void){

	uint32 gpio_status;

	gpio_status = GPIO_REG_READ( GPIO_STATUS_ADDRESS );

	GPIO_REG_WRITE( GPIO_STATUS_W1TC_ADDRESS , gpio_status );

	if( gpio_status & (BIT(15)) )
	{
		xTaskResumeFromISR( key_handler_task_handle );
	}
}

void key_init(void){
	GPIO_ConfigTypeDef gpio_in_cfg;    //Define GPIO Init Structure
	gpio_in_cfg.GPIO_IntrType = GPIO_PIN_INTR_POSEDGE;    //
	gpio_in_cfg.GPIO_Mode = GPIO_Mode_Input;    //Input mode
	gpio_in_cfg.GPIO_Pullup = GPIO_PullUp_DIS;
	gpio_in_cfg.GPIO_Pin = GPIO_Pin_15;    // Enable GPIO
	gpio_config(&gpio_in_cfg);    //Initialization function

	gpio_intr_handler_register(key_interrupt, NULL); // Register the interrupt function
	_xt_isr_unmask(1 << ETS_GPIO_INUM);    //Enable the GPIO interrupt

}

void key_handler_task(void  *pvParameters){
	uint32_t TickCountPre = 0 , TickCountCur = 0;

	key_init();

	for( ;; )
	{
		vTaskSuspend( NULL );

		TickCountPre = TickCountCur;

		TickCountCur = xTaskGetTickCount( );
		if( TickCountCur - TickCountPre > 7 )
		{
			uint8_t i;

			for( i = 0; i < 10 ; i ++ )
			{
				vTaskDelay( 30 / portTICK_RATE_MS );

				uint32_t gpio_value;

				gpio_value = gpio_input_get( );
				if( ( gpio_value & BIT(15) ) == BIT(0) )
				{
					break;
				}
			}

			if( i == 10 )
			{
				if (GPIO_INPUT_GET(15) & 0X0000001){
					led_toggle();
				}

			}

		}
	}

	vTaskDelete(NULL);
}


/******************************************************************************
 * FunctionName : user_init
 * Description  : entry of user application, init user function here
 * Parameters   : none
 * Returns      : none
*******************************************************************************/
void user_init(void)
{
    printf("SDK version:%s\n", system_get_sdk_version());

    led_init();

    //key_init();

    //xTaskCreate(led_toggle_task, "led_toggle_task", 256, NULL,1, NULL);
    xTaskCreate(key_handler_task, "key_handler_task", 256, NULL, 3, &key_handler_task_handle);
}

