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

/**
 * This task is called constantly. The ESP can't handle infinite loops in tasks,
 * so this task will post to itself when finished, in essence looping forever
 *
 * @param events unused
 */
static void ICACHE_FLASH_ATTR procTask(os_event_t *events)
{
	CSTick( 0 );

	// Post the task in order to have it called again
	system_os_post(procTaskPrio, 0, 0 );
}

/**
 * This is a timer set up in user_main() which is called every 100ms, forever
 * @param arg unused
 */
static void ICACHE_FLASH_ATTR timer100ms(void *arg)
{
	CSTick( 1 ); // Send a one to uart

    uint32 button_status = 0;
    button_status = GPIO_INPUT_GET(BUTTON_NUM);

    // GPIO_OUTPUT_SET(LED_NUM, button_status);
    // GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, BIT(LED_NUM)); // - didn't work
}

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


void buttonChange() {
    uint32 in_reg = GPIO_REG_READ(GPIO_IN_ADDRESS);
    os_printf("%u", in_reg);

    GPIO_OUTPUT_SET(LED_NUM, GPIO_INPUT_GET(BUTTON_NUM));
}

/**
 * This callback is registered with espconn_regist_recvcb and is called whenever
 * a UDP packet is received
 *
 * @param arg pointer corresponding structure espconn. This pointer may be
 *            different in different callbacks, please don’t use this pointer
 *            directly to distinguish one from another in multiple connections,
 *            use remote_ip and remote_port in espconn instead.
 * @param pusrdata received data entry parameters
 * @param len      received data length
 */
static void ICACHE_FLASH_ATTR udpserver_recv(void *arg, char *pusrdata, unsigned short len)
{
	struct espconn *pespconn = (struct espconn *)arg;

	uart0_sendStr("X");
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

	os_printf("\r\nesp82XX Web-GUI\r\n%s\b", VERSSTR);

	//Uncomment this to force a system restore.
	//	system_restore();

	// Load settings and pre-initialize common services
	CSSettingsLoad( 0 );
	CSPreInit();
	// Initialize common settings
	CSInit( 0 );


    button_isr_init();

    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDI_U, LED_PIN); // pin D6 on NodeMCU
    // GPIO_DIS_OUTPUT(BUTTON_PIN);
    // gpio_output_set(0, 0, 0, BIT14);

	// Set timer100ms to be called every 100ms
	os_timer_disarm(&some_timer);
	os_timer_setfn(&some_timer, (os_timer_func_t *)timer100ms, NULL);
	os_timer_arm(&some_timer, 100, 1);

	os_printf( "Boot Ok.\n" );

	// Set the wifi sleep type
	wifi_set_sleep_type(LIGHT_SLEEP_T);
	wifi_fpm_set_sleep_type(LIGHT_SLEEP_T);

	// Add a process and start it
	system_os_task(procTask, procTaskPrio, procTaskQueue, procTaskQueueLen);
	system_os_post(procTaskPrio, 0, 0 );
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
