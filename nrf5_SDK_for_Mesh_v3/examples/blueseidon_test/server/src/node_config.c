/**
 * node_config.c 
 * Copyright (C) 2019 blueseidon

 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

/* HAL */
//#include "app_timer.h"
#include "boards.h"
#include "simple_hal.h"

/* Provisioning and configuration */
#include "mesh_app_utils.h"

/* Models */
#include "generic_level_server.h"
#include "generic_onoff_client.h"
#include "generic_onoff_server.h"

/* Logging and RTT */
#include "log.h"

/* Example specific includes */
#include "app_level_userdefined.h"
#include "app_switch.h"
#include "example_common.h"
#include "flash_write.h"
#include "node_config.h"

/************************Initialisation for node config level server model**************************/
#define APP_LEVEL_CONFIG_ELEMENT_INDEX (0)

static void app_level_config_server_set_cb(const app_level_server_t *p_server, uint32_t present_level);
static void app_level_config_server_get_cb(const app_level_server_t *p_server, uint32_t *p_present_level);


/* Application level generic level server structure definition and initialization */
APP_LEVEL_SERVER_DEF(m_level_server_0,
    APP_CONFIG_FORCE_SEGMENTATION,
    APP_CONFIG_MIC_SIZE,
    NULL,
    app_level_config_server_set_cb,
    app_level_config_server_get_cb);

/* Callback for updating the hardware state */
static void app_level_config_server_set_cb(const app_level_server_t *p_server, uint32_t present_level)
{
    /* Resolve the server instance here if required, this example uses only 1 instance. */
    set_node_config(present_level);
}

/* Callback for reading the hardware state */
static void app_level_config_server_get_cb(const app_level_server_t *p_server, uint32_t *p_present_level)
{
    /* Resolve the server instance here if required, this example uses only 1 instance. */
    *p_present_level = get_node_config();
}
/*******************************************END***************************************************/
/******************************Initialisation of SWITCH server MODEL******************************/
#define SWITCH_SERVER_1_PIN (BSP_LED_0)
#define SWITCH_SERVER_2_PIN (BSP_LED_1)
#define SWITCH_SERVER_3_PIN (BSP_LED_2)
#define SWITCH_SERVER_4_PIN (BSP_LED_3)

#define APP_ELEMENT_1_INDEX (1)
#define APP_ELEMENT_2_INDEX (2)
#define APP_ELEMENT_3_INDEX (3)
#define APP_ELEMENT_4_INDEX (4)

static void app_switch_server_1_set_cb(const app_switch_server_t *p_server, bool switch_state);
static void app_switch_server_1_get_cb(const app_switch_server_t *p_server, bool *p_present_switch);

static void app_switch_server_2_set_cb(const app_switch_server_t *p_server, bool switch_state);
static void app_switch_server_2_get_cb(const app_switch_server_t *p_server, bool *p_present_switch);

static void app_switch_server_3_set_cb(const app_switch_server_t *p_server, bool switch_state);
static void app_switch_server_3_get_cb(const app_switch_server_t *p_server, bool *p_present_switch);

static void app_switch_server_4_set_cb(const app_switch_server_t *p_server, bool switch_state);
static void app_switch_server_4_get_cb(const app_switch_server_t *p_server, bool *p_present_switch);

/* Generic switch server structure definition and initialization */
APP_SWITCH_SERVER_DEF(m_switch_server_1,
    APP_CONFIG_FORCE_SEGMENTATION,
    APP_CONFIG_MIC_SIZE,
    app_switch_server_1_set_cb,
    app_switch_server_1_get_cb)

APP_SWITCH_SERVER_DEF(m_switch_server_2,
    APP_CONFIG_FORCE_SEGMENTATION,
    APP_CONFIG_MIC_SIZE,
    app_switch_server_2_set_cb,
    app_switch_server_2_get_cb)

APP_SWITCH_SERVER_DEF(m_switch_server_3,
    APP_CONFIG_FORCE_SEGMENTATION,
    APP_CONFIG_MIC_SIZE,
    app_switch_server_3_set_cb,
    app_switch_server_3_get_cb)

APP_SWITCH_SERVER_DEF(m_switch_server_4,
    APP_CONFIG_FORCE_SEGMENTATION,
    APP_CONFIG_MIC_SIZE,
    app_switch_server_4_set_cb,
    app_switch_server_4_get_cb)

/* Callback for updating the hardware state */
static void app_switch_server_1_set_cb(const app_switch_server_t *p_server, bool switch_state)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Setting GPIO value: %d\n", switch_state)

    hal_led_pin_set(SWITCH_SERVER_1_PIN, switch_state);
}

/* Callback for updating the hardware state */
static void app_switch_server_2_set_cb(const app_switch_server_t *p_server, bool switch_state)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Setting GPIO value: %d\n", switch_state)

    hal_led_pin_set(SWITCH_SERVER_2_PIN, switch_state);
}

/* Callback for updating the hardware state */
static void app_switch_server_3_set_cb(const app_switch_server_t *p_server, bool switch_state)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Setting GPIO value: %d\n", switch_state)

    hal_led_pin_set(SWITCH_SERVER_3_PIN, switch_state);
}

/* Callback for updating the hardware state */
static void app_switch_server_4_set_cb(const app_switch_server_t *p_server, bool switch_state)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Setting GPIO value: %d\n", switch_state)

    hal_led_pin_set(SWITCH_SERVER_4_PIN, switch_state);
}

/* Callback for reading the hardware state */
static void app_switch_server_1_get_cb(const app_switch_server_t *p_server, bool *p_present_switch)
{
    *p_present_switch = hal_led_pin_get(SWITCH_SERVER_1_PIN);
}

/* Callback for reading the hardware state */
static void app_switch_server_2_get_cb(const app_switch_server_t *p_server, bool *p_present_switch)
{
    *p_present_switch = hal_led_pin_get(SWITCH_SERVER_2_PIN);
}

/* Callback for reading the hardware state */
static void app_switch_server_3_get_cb(const app_switch_server_t *p_server, bool *p_present_switch)
{
    *p_present_switch = hal_led_pin_get(SWITCH_SERVER_3_PIN);
}

/* Callback for reading the hardware state */
static void app_switch_server_4_get_cb(const app_switch_server_t *p_server, bool *p_present_switch)
{
    *p_present_switch = hal_led_pin_get(SWITCH_SERVER_4_PIN);
}
/*******************************************END***************************************************/
/******************************Initialisation of LEVEL server MODEL*******************************/
static void app_level_server_1_set_cb(const app_level_server_t *p_server, uint32_t present_level);
static void app_level_server_1_get_cb(const app_level_server_t *p_server, uint32_t *present_level);

static void app_level_server_2_set_cb(const app_level_server_t *p_server, uint32_t present_level);
static void app_level_server_2_get_cb(const app_level_server_t *p_server, uint32_t *present_level);

static void app_level_server_3_set_cb(const app_level_server_t *p_server, uint32_t present_level);
static void app_level_server_3_get_cb(const app_level_server_t *p_server, uint32_t *present_level);

static void app_level_server_4_set_cb(const app_level_server_t *p_server, uint32_t present_level);
static void app_level_server_4_get_cb(const app_level_server_t *p_server, uint32_t *present_level);

/* Generic switch server structure definition and initialization */
APP_LEVEL_SERVER_DEF(m_level_server_1,
    APP_CONFIG_FORCE_SEGMENTATION,
    APP_CONFIG_MIC_SIZE,
    NULL,
    app_level_server_1_set_cb,
    app_level_server_1_get_cb)

APP_LEVEL_SERVER_DEF(m_level_server_2,
    APP_CONFIG_FORCE_SEGMENTATION,
    APP_CONFIG_MIC_SIZE,
    NULL,
    app_level_server_2_set_cb,
    app_level_server_2_get_cb)

APP_LEVEL_SERVER_DEF(m_level_server_3,
    APP_CONFIG_FORCE_SEGMENTATION,
    APP_CONFIG_MIC_SIZE,
    NULL,
    app_level_server_3_set_cb,
    app_level_server_3_get_cb)

APP_LEVEL_SERVER_DEF(m_level_server_4,
    APP_CONFIG_FORCE_SEGMENTATION,
    APP_CONFIG_MIC_SIZE,
    NULL,
    app_level_server_4_set_cb,
    app_level_server_4_get_cb)

/* Callback for updating the hardware state */
static void app_level_server_1_set_cb(const app_level_server_t *p_server, uint32_t present_level)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Setting GPIO value: %d\n", present_level)
}

/* Callback for updating the hardware state */
static void app_level_server_2_set_cb(const app_level_server_t *p_server, uint32_t present_level)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Setting GPIO value: %d\n", present_level)
}

/* Callback for updating the hardware state */
static void app_level_server_3_set_cb(const app_level_server_t *p_server, uint32_t present_level)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Setting GPIO value: %d\n", present_level)
}

/* Callback for updating the hardware state */
static void app_level_server_4_set_cb(const app_level_server_t *p_server, uint32_t present_level)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Setting GPIO value: %d\n", present_level)
}

/* Callback for reading the hardware state */
static void app_level_server_1_get_cb(const app_level_server_t *p_server, uint32_t *present_level)
{
    *present_level = 1;
}

/* Callback for reading the hardware state */
static void app_level_server_2_get_cb(const app_level_server_t *p_server, uint32_t *present_level)
{
    *present_level = 2;
}

/* Callback for reading the hardware state */
static void app_level_server_3_get_cb(const app_level_server_t *p_server, uint32_t *present_level)
{
    *present_level = 3;
}

/* Callback for reading the hardware state */
static void app_level_server_4_get_cb(const app_level_server_t *p_server, uint32_t *present_level)
{
    *present_level = 4;
}
/*******************************************END***************************************************/
/******************************Initialisation of SWITCH client MODEL******************************/
//#define APP_STATE_OFF (0)
//#define APP_STATE_ON (1)
//
//#define APP_UNACK_MSG_REPEAT_COUNT (2)
//
//static generic_onoff_client_t m_clients;
//
///* Forward declaration */
//static void app_gen_onoff_client_publish_interval_cb(access_model_handle_t handle, void *p_self);
//static void app_generic_onoff_client_status_cb(const generic_onoff_client_t *p_self,
//    const access_message_rx_meta_t *p_meta,
//    const generic_onoff_status_params_t *p_in);
//static void app_gen_onoff_client_transaction_status_cb(access_model_handle_t model_handle,
//    void *p_args,
//    access_reliable_status_t status);
//
//const generic_onoff_client_callbacks_t client_cbs =
//    {
//        .onoff_status_cb = app_generic_onoff_client_status_cb,
//        .ack_transaction_status_cb = app_gen_onoff_client_transaction_status_cb,
//        .periodic_publish_cb = app_gen_onoff_client_publish_interval_cb};
//
///* This callback is called periodically if model is configured for periodic publishing */
//static void app_gen_onoff_client_publish_interval_cb(access_model_handle_t handle, void *p_self)
//{
//    __LOG(LOG_SRC_APP, LOG_LEVEL_WARN, "Publish desired message here.\n");
//}
//
///* Acknowledged transaction status callback, if acknowledged transfer fails, application can
//* determine suitable course of action (e.g. re-initiate previous transaction) by using this
//* callback.
//*/
//static void app_gen_onoff_client_transaction_status_cb(access_model_handle_t model_handle,
//    void *p_args,
//    access_reliable_status_t status)
//{
//    switch (status)
//    {
//    case ACCESS_RELIABLE_TRANSFER_SUCCESS:
//        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Acknowledged transfer success.\n");
//        break;
//
//    case ACCESS_RELIABLE_TRANSFER_TIMEOUT:
//        hal_led_blink_ms(LEDS_MASK, LED_BLINK_SHORT_INTERVAL_MS, LED_BLINK_CNT_NO_REPLY);
//        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Acknowledged transfer timeout.\n");
//        break;
//
//    case ACCESS_RELIABLE_TRANSFER_CANCELLED:
//        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Acknowledged transfer cancelled.\n");
//        break;
//
//    default:
//        ERROR_CHECK(NRF_ERROR_INTERNAL);
//        break;
//    }
//}
//
///* Generic OnOff client model interface: Process the received status message in this callback */
//static void app_generic_onoff_client_status_cb(const generic_onoff_client_t *p_self,
//    const access_message_rx_meta_t *p_meta,
//    const generic_onoff_status_params_t *p_in)
//{
//    if (p_in->remaining_time_ms > 0)
//    {
//        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "OnOff server: 0x%04x, Present OnOff: %d, Target OnOff: %d, Remaining Time: %d ms\n",
//            p_meta->src.value, p_in->present_on_off, p_in->target_on_off, p_in->remaining_time_ms);
//    }
//    else
//    {
//        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "OnOff server: 0x%04x, Present OnOff: %d\n",
//            p_meta->src.value, p_in->present_on_off);
//    }
//}
/*******************************************END***************************************************/
/***********************************Initialisation of ALL MODELS**********************************/
#define SWITCH_SERVER_CONFIG (1)
#define SWITCH_CLIENT_CONFIG (2)
#define LEVEL_SERVER_CONFIG (3)
#define LEVEL_CLIENT_CONFIG (4)

void app_model_init(void)
{
    node_config_flash_init();
    uint32_t config = get_node_config();
    uint8_t CHANNEL_1 = (uint8_t)(config >> 28);
    uint8_t CHANNEL_2 = (((uint8_t)(config >> 24)) & 0b00001111);
    uint8_t CHANNEL_3 = (uint8_t)(config >> 20);
    uint8_t CHANNEL_4 = (uint8_t)(config >> 16);
    uint8_t LPN_ONOFF = (uint8_t)(config >> 12);
    uint8_t LPN_POLL =  (uint8_t)(config >>  8);
    uint8_t LPN_DELAY = (uint8_t)(config >>  4);
    uint8_t LPN_RX =    (uint8_t)(config >>  0);

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "\n Channel 1: %u | Channel 2: %u | Channel 3: %u | Channel 4: %u \n LPN State: %u | LPN Poll time: %u | LPN Delay: %u | LPN Receive Time: %u \n", CHANNEL_1, CHANNEL_2, CHANNEL_3, CHANNEL_4, LPN_ONOFF, LPN_POLL, LPN_DELAY, LPN_RX);

    /* Instantiate level server on element index 0 */
    ERROR_CHECK(app_level_init(&m_level_server_0, APP_LEVEL_CONFIG_ELEMENT_INDEX));
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "App level Model 0 Handle: %d\n", m_level_server_0.server.model_handle);

    if (CHANNEL_1 == SWITCH_SERVER_CONFIG)
    {
        /* Instantiate switch server on element index APP_SWITCH_ELEMENT_1 */
        ERROR_CHECK(app_switch_init(&m_switch_server_1, APP_ELEMENT_1_INDEX));
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "App switch Model 1 Handle: %d\n", m_switch_server_1.server.model_handle);
    }

    if (CHANNEL_2 == SWITCH_SERVER_CONFIG)
    {
        /* Instantiate switch server on element index APP_SWITCH_ELEMENT_2 */
        ERROR_CHECK(app_switch_init(&m_switch_server_2, APP_ELEMENT_2_INDEX));
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "App switch Model 2 Handle: %d\n", m_switch_server_2.server.model_handle);
    }

    if (CHANNEL_3 == SWITCH_SERVER_CONFIG)
    {
        /* Instantiate switch server on element index APP_SWITCH_ELEMENT_3 */
        ERROR_CHECK(app_switch_init(&m_switch_server_3, APP_ELEMENT_3_INDEX));
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "App switch Model 3 Handle: %d\n", m_switch_server_3.server.model_handle);
    }

    if (CHANNEL_4 == SWITCH_SERVER_CONFIG)
    {
        /* Instantiate switch server on element index APP_SWITCH_ELEMENT_4 */
        ERROR_CHECK(app_switch_init(&m_switch_server_4, APP_ELEMENT_4_INDEX));
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "App switch Model 4 Handle: %d\n", m_switch_server_4.server.model_handle);
    }

    //    if (CHANNEL_1 == SWITCH_CLIENT_CONFIG)
    //    {
    //        m_clients.settings.p_callbacks = &client_cbs;
    //        m_clients.settings.timeout = 0;
    //        m_clients.settings.force_segmented = APP_CONFIG_FORCE_SEGMENTATION;
    //        m_clients.settings.transmic_size = APP_CONFIG_MIC_SIZE;
    //
    //        ERROR_CHECK(generic_onoff_client_init(&m_clients, APP_ELEMENT_1_INDEX));
    //        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "App level Model client 1 Handle: %d\n", m_clients.model_handle);
    //    }

    if (CHANNEL_1 == LEVEL_SERVER_CONFIG)
    {
        /* Instantiate level server on element index APP_LEVEL_ELEMENT_1 */
        ERROR_CHECK(app_level_init(&m_level_server_1, APP_ELEMENT_1_INDEX));
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "App level Model 1 Handle: %d\n", m_level_server_1.server.model_handle);
    }

    if (CHANNEL_2 == LEVEL_SERVER_CONFIG)
    {
        /* Instantiate level server on element index APP_LEVEL_ELEMENT_2 */
        ERROR_CHECK(app_level_init(&m_level_server_2, APP_ELEMENT_2_INDEX));
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "App level Model 2 Handle: %d\n", m_level_server_2.server.model_handle);
    }

    if (CHANNEL_3 == LEVEL_SERVER_CONFIG)
    {
        /* Instantiate level server on element index APP_LEVEL_ELEMENT_3 */
        ERROR_CHECK(app_level_init(&m_level_server_3, APP_ELEMENT_3_INDEX));
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "App level Model 3 Handle: %d\n", m_level_server_3.server.model_handle);
    }

    if (CHANNEL_4 == LEVEL_SERVER_CONFIG)
    {
        /* Instantiate level server on element index APP_LEVEL_ELEMENT_4 */
        ERROR_CHECK(app_level_init(&m_level_server_4, APP_ELEMENT_4_INDEX));
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "App level Model 4 Handle: %d\n", m_level_server_4.server.model_handle);
    }
}
/*******************************************END***************************************************/