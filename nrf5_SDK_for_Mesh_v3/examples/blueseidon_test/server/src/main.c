/**
 * main.h 
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

/** LPN feature */
#define MESH_FEATURE_LPN_ENABLED 1

// Did not enter Sleep ?
// Set bit 7 and bits 4..0 in the mask to one (0x ...00 1001 1111)
#define FPU_EXCEPTION_MASK 0x0000009F

#include <stdint.h>
#include <string.h>

/* HAL */
#include "app_timer.h"
#include "boards.h"
#include "simple_hal.h"

/* Core */
#include "access_config.h"
#include "device_state_manager.h"
#include "mesh_stack.h"
#include "nrf_mesh.h"
#include "nrf_mesh_config_core.h"
#include "nrf_mesh_configure.h"
#include "nrf_mesh_gatt.h"
#include "proxy.h"

#if MESH_FEATURE_LPN_ENABLED
/* LPN */
#include "mesh_friendship_types.h"
#include "mesh_lpn.h"

/* nRF5 SDK */
#include "nrf_pwr_mgmt.h"
#include "nrf_soc.h"
#endif

/* Provisioning and configuration */
#include "mesh_app_utils.h"
#include "mesh_provisionee.h"

/* Models */
//#include "generic_onoff_server.h"

/* Logging and RTT */
#include "log.h"
#include "rtt_input.h"

/* Example specific includes */
#include "app_config.h"
#include "app_switch.h"
#include "ble_softdevice_support.h"
#include "blueseidon_test_example_common.h"
#include "example_common.h"
#include "node_config.h"
#include "nrf_mesh_config_examples.h"

#if MESH_FEATURE_LPN_ENABLED
/** The maximum duration to scan for incoming Friend Offers. */
#define FRIEND_REQUEST_TIMEOUT_MS (MESH_LPN_FRIEND_REQUEST_TIMEOUT_MAX_MS)
/** The upper limit for two subsequent Friend Polls. */
#define POLL_TIMEOUT_MS (SEC_TO_MS(10))
/** The time between LPN sending a request and listening for a response. */
#define RECEIVE_DELAY_MS (100)
#endif

static bool m_device_provisioned;
static void start(void);

static void node_reset(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- Node reset  -----\n");
    hal_led_blink_ms(LEDS_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_RESET);
    /* This function may return if there are ongoing flash operations. */
    mesh_stack_device_reset();
}

static void config_server_evt_cb(const config_server_evt_t *p_evt)
{
    if (p_evt->type == CONFIG_SERVER_EVT_NODE_RESET)
    {
        node_reset();
    }
}

#if MESH_FEATURE_LPN_ENABLED
static void initiate_friendship()
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initiating the friendship establishment procedure.\n");

    mesh_lpn_friend_request_t freq;
    freq.friend_criteria.friend_queue_size_min_log = MESH_FRIENDSHIP_MIN_FRIEND_QUEUE_SIZE_16;
    freq.friend_criteria.receive_window_factor = MESH_FRIENDSHIP_RECEIVE_WINDOW_FACTOR_1_0;
    freq.friend_criteria.rssi_factor = MESH_FRIENDSHIP_RSSI_FACTOR_2_0;
    freq.poll_timeout_ms = POLL_TIMEOUT_MS;
    freq.receive_delay_ms = RECEIVE_DELAY_MS;

    uint32_t status = mesh_lpn_friend_request(freq, FRIEND_REQUEST_TIMEOUT_MS);
    switch (status)
    {
    case NRF_SUCCESS:
        break;

    case NRF_ERROR_INVALID_STATE:
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Already in an active friendship\n");
#if SIMPLE_HAL_LEDS_ENABLED
        hal_led_blink_ms(LEDS_MASK, LED_BLINK_SHORT_INTERVAL_MS, LED_BLINK_CNT_ERROR);
#endif
        break;

    case NRF_ERROR_INVALID_PARAM:
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Friend request parameters outside of valid ranges.\n");
#if SIMPLE_HAL_LEDS_ENABLED
        hal_led_blink_ms(LEDS_MASK, LED_BLINK_SHORT_INTERVAL_MS, LED_BLINK_CNT_ERROR);
#endif
        break;

    default:
        ERROR_CHECK(status);
        break;
    }
}

static void terminate_friendship()
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Terminating the active friendship\n");

    uint32_t status = mesh_lpn_friendship_terminate();
    switch (status)
    {
    case NRF_SUCCESS:
        break;

    case NRF_ERROR_INVALID_STATE:
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Not in an active friendship\n");
#if SIMPLE_HAL_LEDS_ENABLED
        hal_led_blink_ms(LEDS_MASK, LED_BLINK_SHORT_INTERVAL_MS, LED_BLINK_CNT_ERROR);
#endif
        break;

    default:
        ERROR_CHECK(status);
        break;
    }
}
#endif

static void button_event_handler(uint32_t button_number)
{
    //__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Button %u pressed\n", button_number);
    switch (button_number)
    {
    /* Pressing SW1 on the Development Kit will result in LED state to toggle and trigger
        the STATUS message to inform client about the state change. This is a demonstration of
        state change publication due to local event. */
    case 0:
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Button 1 was pressed \n");
#if MESH_FEATURE_LPN_ENABLED
        if (!mesh_lpn_is_in_friendship())
        {
            initiate_friendship();
        }
        else /* In a friendship */
        {
            terminate_friendship();
        }
#endif
        break;
    }

    case 1:
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Button 2 was pressed \n");
        break;
    }

    case 2:
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Button 3 was pressed \n");
        uint32_t config = get_node_config();
        uint8_t CHANNEL_1 = (((uint8_t)(config >> 28)) & 0b00001111);
        uint8_t CHANNEL_2 = (((uint8_t)(config >> 24)) & 0b00001111);
        uint8_t CHANNEL_3 = (((uint8_t)(config >> 20)) & 0b00001111);
        uint8_t CHANNEL_4 = (((uint8_t)(config >> 16)) & 0b00001111);
        uint8_t LPN_ONOFF = (((uint8_t)(config >> 12)) & 0b00001111);
        uint8_t LPN_POLL =  (((uint8_t)(config >> 8)) & 0b00001111);
        uint8_t LPN_DELAY = (((uint8_t)(config >> 4)) & 0b00001111);
        uint8_t LPN_RX =    (((uint8_t)(config >> 0)) & 0b00001111);

        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "\n Channel 1: %u | Channel 2: %u | Channel 3: %u | Channel 4: %u \n LPN State: %u | LPN Poll time: %u | LPN Delay: %u | LPN Receive Time: %u \n", CHANNEL_1, CHANNEL_2, CHANNEL_3, CHANNEL_4, LPN_ONOFF, LPN_POLL, LPN_DELAY, LPN_RX);
        break;
    }

    /* Initiate node reset */
    case 3:
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Button 4 was pressed \n");
        /* Clear all the states to reset the node. */
        if (!mesh_stack_is_device_provisioned())
        {
#if MESH_FEATURE_GATT_PROXY_ENABLED
            (void)proxy_stop();
#endif
            mesh_stack_config_clear();
            node_reset();
        }
        else
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "The device is unprovisioned. Resetting has no effect.\n");
        }
        break;
    }

    default:
        break;
    }
}

static void app_rtt_input_handler(int key)
{
    if (key >= '0' && key <= '4')
    {
        uint32_t button_number = key - '0';
        button_event_handler(button_number);
    }
}

static void device_identification_start_cb(uint8_t attention_duration_s)
{
    hal_led_mask_set(LEDS_MASK, false);
    hal_led_blink_ms(BSP_LED_2_MASK | BSP_LED_3_MASK,
        LED_BLINK_ATTENTION_INTERVAL_MS,
        LED_BLINK_ATTENTION_COUNT(attention_duration_s));
}

static void provisioning_aborted_cb(void)
{
    hal_led_blink_stop();
}

static void provisioning_complete_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Successfully provisioned\n");

#if MESH_FEATURE_GATT_ENABLED
    /* Restores the application parameters after switching from the Provisioning
     * service to the Proxy  */
    gap_params_init();
    conn_params_init();
#endif

    dsm_local_unicast_address_t node_address;
    dsm_local_unicast_addresses_get(&node_address);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Node Address: 0x%04x \n", node_address.address_start);

    hal_led_blink_stop();
    hal_led_mask_set(LEDS_MASK, LED_MASK_STATE_OFF);
    hal_led_blink_ms(LEDS_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_PROV);
}

#if MESH_FEATURE_LPN_ENABLED
static void app_mesh_core_event_cb(const nrf_mesh_evt_t *p_evt)
{
    /* USER_NOTE: User can insert mesh core event proceesing here */
    switch (p_evt->type)
    {
    case NRF_MESH_EVT_LPN_FRIEND_OFFER:
    {
        const nrf_mesh_evt_lpn_friend_offer_t *p_offer = &p_evt->params.friend_offer;

        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
            "Received friend offer from 0x%04X\n",
            p_offer->src);

        uint32_t status = mesh_lpn_friend_accept(p_offer);
        switch (status)
        {
        case NRF_SUCCESS:
            break;

        case NRF_ERROR_INVALID_STATE:
        case NRF_ERROR_INVALID_PARAM:
        case NRF_ERROR_NULL:
            __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR,
                "Cannot accept friendship: %d\n",
                status);
#if SIMPLE_HAL_LEDS_ENABLED
            hal_led_blink_ms(LEDS_MASK, LED_BLINK_SHORT_INTERVAL_MS,
                LED_BLINK_CNT_ERROR);
#endif
            break;

        default:
            ERROR_CHECK(status);
            break;
        }

        break;
    }

    case NRF_MESH_EVT_LPN_FRIEND_POLL_COMPLETE:
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Friend poll procedure complete\n");
        break;

    case NRF_MESH_EVT_LPN_FRIEND_REQUEST_TIMEOUT:
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Friend Request timed out\n");
#if SIMPLE_HAL_LEDS_ENABLED
        hal_led_blink_ms(LEDS_MASK, LED_BLINK_SHORT_INTERVAL_MS, LED_BLINK_CNT_ERROR);
#endif
        break;

    case NRF_MESH_EVT_FRIENDSHIP_ESTABLISHED:
    {
        const nrf_mesh_evt_friendship_established_t *p_est =
            &p_evt->params.friendship_established;
        (void)p_est;

        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
            "Friendship established with: 0x%04X\n",
            p_est->friend_src);

#if SIMPLE_HAL_LEDS_ENABLED
        hal_led_pin_set(BSP_LED_1, true);
#endif
        break;
    }

    case NRF_MESH_EVT_FRIENDSHIP_TERMINATED:
    {
        const nrf_mesh_evt_friendship_terminated_t *p_term = &p_evt->params.friendship_terminated;
        UNUSED_VARIABLE(p_term);

        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
            "Friendship with 0x%04X terminated. Reason: %d\n",
            p_term->friend_src, p_term->reason);

#if SIMPLE_HAL_LEDS_ENABLED
        hal_led_pin_set(BSP_LED_1, false);
#endif

        //ERROR_CHECK(app_timer_stop(m_state_on_timer));
        break;
    }

    default:
        break;
    }
}

static nrf_mesh_evt_handler_t m_mesh_core_event_handler = {.evt_cb = app_mesh_core_event_cb};
#endif

static void models_init_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initializing and adding models\n");
    app_model_init();
}

static void mesh_init(void)
{
    mesh_stack_init_params_t init_params =
        {
            .core.irq_priority = NRF_MESH_IRQ_PRIORITY_LOWEST,
            .core.lfclksrc = DEV_BOARD_LF_CLK_CFG,
            .core.p_uuid = NULL,
            .models.models_init_cb = models_init_cb,
            .models.config_server_cb = config_server_evt_cb};
    ERROR_CHECK(mesh_stack_init(&init_params, &m_device_provisioned));

#if MESH_FEATURE_LPN_ENABLED
    /* Register event handler to receive LPN and friendship events. */
    nrf_mesh_evt_handler_add(&m_mesh_core_event_handler);
#endif
}

static void initialize(void)
{
    __LOG_INIT(LOG_SRC_APP | LOG_SRC_FRIEND, LOG_LEVEL_DBG1, LOG_CALLBACK_DEFAULT);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- BLE Mesh Blueseidon Server Test -----\n");

    ERROR_CHECK(app_timer_init());
    hal_leds_init();

#if BUTTON_BOARD
    ERROR_CHECK(hal_buttons_init(button_event_handler));
#endif
    ble_stack_init();

#if MESH_FEATURE_GATT_ENABLED
    gap_params_init();
    conn_params_init();
#endif

    mesh_init();

    ERROR_CHECK(sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE));

#if MESH_FEATURE_LPN_ENABLED
    mesh_lpn_init();
#endif
}

static void start(void)
{
    rtt_input_enable(app_rtt_input_handler, RTT_INPUT_POLL_PERIOD_MS);

    if (!m_device_provisioned)
    {
        static const uint8_t static_auth_data[NRF_MESH_KEY_SIZE] = STATIC_AUTH_DATA;
        mesh_provisionee_start_params_t prov_start_params =
            {
                .p_static_data = static_auth_data,
                .prov_complete_cb = provisioning_complete_cb,
                .prov_device_identification_start_cb = device_identification_start_cb,
                .prov_device_identification_stop_cb = NULL,
                .prov_abort_cb = provisioning_aborted_cb,
                .p_device_uri = EX_URI_LS_SERVER};
        ERROR_CHECK(mesh_provisionee_prov_start(&prov_start_params));
    }

    mesh_app_uuid_print(nrf_mesh_configure_device_uuid_get());

    ERROR_CHECK(mesh_stack_start());

    hal_led_mask_set(LEDS_MASK, LED_MASK_STATE_OFF);
    hal_led_blink_ms(LEDS_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_START);
}

int main(void)
{
    initialize();
    start();

    for (;;)
    {
        /* Clear exceptions and PendingIRQ from the FPU unit */
        __set_FPSCR(__get_FPSCR() & ~(FPU_EXCEPTION_MASK));
        (void)__get_FPSCR();
        NVIC_ClearPendingIRQ(FPU_IRQn);
        (void)sd_app_evt_wait();
    }
}

//static void button_event_handler(uint32_t button_number)
//{
//    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Button %u pressed\n", button_number);
//
//    uint32_t status = NRF_SUCCESS;
//    generic_onoff_set_params_t set_params;
//    model_transition_t transition_params;
//    static uint8_t tid = 0;
//
//    /* Button 1: On, Button 2: Off, Client[0]
//     * Button 2: On, Button 3: Off, Client[1]
//     */
//
//    switch(button_number)
//    {
//        case 0:
//        case 2:
//            set_params.on_off = APP_STATE_ON;
//            break;
//
//        case 1:
//        case 3:
//            set_params.on_off = APP_STATE_OFF;
//            break;
//    }
//
//    set_params.tid = tid++;
//    transition_params.delay_ms = APP_CONFIG_ONOFF_DELAY_MS;
//    transition_params.transition_time_ms = APP_CONFIG_ONOFF_TRANSITION_TIME_MS;
//    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Sending msg: ONOFF SET %d\n", set_params.on_off);
//
//    switch (button_number)
//    {
//        case 0:
//        case 1:
//            /* Demonstrate acknowledged transaction, using 1st client model instance */
//            /* In this examples, users will not be blocked if the model is busy */
//            (void)access_model_reliable_cancel(m_clients[0].model_handle);
//            status = generic_onoff_client_set(&m_clients[0], &set_params, &transition_params);
//            hal_led_pin_set(BSP_LED_0, set_params.on_off);
//            break;
//
//        case 2:
//        case 3:
//            /* Demonstrate un-acknowledged transaction, using 2nd client model instance */
//            status = generic_onoff_client_set_unack(&m_clients[1], &set_params,
//                                                    &transition_params, APP_UNACK_MSG_REPEAT_COUNT);
//            hal_led_pin_set(BSP_LED_1, set_params.on_off);
//            break;
//    }
//
//    switch (status)
//    {
//        case NRF_SUCCESS:
//            break;
//
//        case NRF_ERROR_NO_MEM:
//        case NRF_ERROR_BUSY:
//        case NRF_ERROR_INVALID_STATE:
//            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Client %u cannot send\n", button_number);
//            hal_led_blink_ms(LEDS_MASK, LED_BLINK_SHORT_INTERVAL_MS, LED_BLINK_CNT_NO_REPLY);
//            break;
//
//        case NRF_ERROR_INVALID_PARAM:
//            /* Publication not enabled for this client. One (or more) of the following is wrong:
//             * - An application key is missing, or there is no application key bound to the model
//             * - The client does not have its publication state set
//             *
//             * It is the provisioner that adds an application key, binds it to the model and sets
//             * the model's publication state.
//             */
//            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Publication not configured for client %u\n", button_number);
//            break;
//
//        default:
//            ERROR_CHECK(status);
//            break;
//    }
//}