//Copyright 2015, 2018 <>< Charles Lohr, Adam Feinstein see LICENSE file.

/*==============================================================================
 * Includes
 *============================================================================*/

#include "mem.h"
#include "c_types.h"
#include "user_interface.h"
#include "ets_sys.h"
#include "uart.h"
#include "osapi.h"
#include "espconn.h"
#include "esp82xxutil.h"
#include "commonservices.h"
#include "vars.h"


typedef void (*isr_callback) (void);

struct button_info {
    uint8 is_down;
};

struct button_info g_button;

/*==============================================================================
 * Process Defines
 *============================================================================*/

#define procTaskPrio        0
#define procTaskQueueLen    1
os_event_t    procTaskQueue[procTaskQueueLen];

// Pin defines

// pin D6 on NodeMCU
#define LED_PIN FUNC_GPIO12 
#define LED_NUM 12

// pin D5 on NodeMCU
#define BUTTON_PIN FUNC_GPIO14
#define BUTTON_NUM 14

/*==============================================================================
 * Variables
 *============================================================================*/

static os_timer_t some_timer;

/*==============================================================================
 * Functions
 *============================================================================*/

void buttonISR(void *data)
{
    struct button_info *button = (struct button_info*) data;
    uint32 gpio_status = GPIO_REG_READ(GPIO_STATUS_ADDRESS);

    if (gpio_status & BIT(BUTTON_NUM)) {
        gpio_pin_intr_state_set(GPIO_ID_PIN(BUTTON_NUM), GPIO_PIN_INTR_DISABLE);
        // copied the next line from example at driver/key.c, but it should be sufficient
        // to just write BIT(BUTTON_NUM), is it not?
        GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, gpio_status & BIT(BUTTON_NUM));

        if (button->is_down == 0) {
            button->is_down = 1;
            os_printf("down\n");

            gpio_pin_intr_state_set(GPIO_ID_PIN(BUTTON_NUM), GPIO_PIN_INTR_NEGEDGE);

            GPIO_OUTPUT_SET(LED_NUM, 1);
        } else {
            button->is_down = 0;
            os_printf("up\n");
            gpio_pin_intr_state_set(GPIO_ID_PIN(BUTTON_NUM), GPIO_PIN_INTR_POSEDGE);

            GPIO_OUTPUT_SET(LED_NUM, 0);
        }
    } else {
        os_printf("wadafak\n");
    }
}

void ICACHE_FLASH_ATTR
button_isr_init()
{
    g_button.is_down = 0;
    ETS_GPIO_INTR_ATTACH(buttonISR, (void*) &g_button);
    ETS_GPIO_INTR_DISABLE();

    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTMS_U, BUTTON_PIN); // pin D5 on NodeMCU

    gpio_output_set(0, 0, 0, GPIO_ID_PIN(BUTTON_NUM));

    gpio_register_set(GPIO_PIN_ADDR(BUTTON_NUM), 
            GPIO_PIN_INT_TYPE_SET(GPIO_PIN_INTR_DISABLE) 
            | GPIO_PIN_PAD_DRIVER_SET(GPIO_PAD_DRIVER_DISABLE)
            | GPIO_PIN_SOURCE_SET(GPIO_AS_PIN_SOURCE));

    // clear gpio14 status???????? - comment from driver/key.c from esp8266 nonos sdk examples
    // seems like W1TC and W1TS are used for setting pins to 1 and 0, being faster than the set macros 
    GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, BIT(BUTTON_NUM));
    
    // enable interrupt
    gpio_pin_intr_state_set(GPIO_ID_PIN(BUTTON_NUM), GPIO_PIN_INTR_NEGEDGE);

    ETS_GPIO_INTR_ENABLE();
}

/**
 * UART RX handler, called by the uart task. Currently does nothing
 *
 * @param c The char received on the UART
 */
void ICACHE_FLASH_ATTR charrx( uint8_t c )
{
    //Called from UART.
}

/**
 * This is called on boot for versions ESP8266_NONOS_SDK_v1.5.2 to
 * ESP8266_NONOS_SDK_v2.2.1. system_phy_set_rfoption() may be called here
 */
void user_rf_pre_init(void)
{
    ; // nothing
}


void ICACHE_FLASH_ATTR user_pre_init(void)
{
    LoadDefaultPartitionMap(); //You must load the partition table so the NONOS SDK can find stuff.
}

/**
 * The default method, equivalent to main() in other environments. Handles all
 * initialization
 */
void ICACHE_FLASH_ATTR user_init(void)
{
    // Initialize the UART
    uart_init(BIT_RATE_115200, BIT_RATE_115200);

    os_printf("\r\nesp82XX \r\n%s\b", VERSSTR);

    button_isr_init();

    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDI_U, LED_PIN); // pin D6 on NodeMCU

    os_printf( "Boot Ok.\n" );
}

/**
 * This will be called to disable any interrupts should the firmware enter a
 * section with critical timing. There is no code in this project that will
 * cause reboots if interrupts are disabled.
 */
void ICACHE_FLASH_ATTR EnterCritical(void)
{
    ;
}

/**
 * This will be called to enable any interrupts after the firmware exits a
 * section with critical timing.
 */
void ICACHE_FLASH_ATTR ExitCritical(void)
{
    ;
}
