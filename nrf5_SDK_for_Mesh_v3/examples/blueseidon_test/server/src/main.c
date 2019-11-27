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

/* Provisioning and configuration */
#include "mesh_app_utils.h"
#include "mesh_provisionee.h"

/* Models */
#include "generic_onoff_server.h"

/* Logging and RTT */
#include "log.h"
#include "rtt_input.h"

/* Example specific includes */
#include "app_config.h"
#include "app_switch.h"
#include "ble_softdevice_support.h"
#include "blueseidon_test_example_common.h"
#include "example_common.h"
#include "nrf_mesh_config_examples.h"
#include "node_config.h"

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
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "state: %u \n", get_node_config());

        uint32_t node_config = get_node_config();
        uint8_t CHANNEL_1 = (uint8_t)(node_config >> 0);
        uint8_t CHANNEL_2 = (uint8_t)(node_config >> 8);
        uint8_t CHANNEL_3 = (uint8_t)(node_config >> 16);
        uint8_t CHANNEL_4 = (uint8_t)(node_config >> 24);

        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Channel 1: %u \n", CHANNEL_1);
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Channel 2: %u \n", CHANNEL_2);
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Channel 3: %u \n", CHANNEL_3);
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Channel 4: %u \n", CHANNEL_4);
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
        (void)sd_app_evt_wait();
    }
}