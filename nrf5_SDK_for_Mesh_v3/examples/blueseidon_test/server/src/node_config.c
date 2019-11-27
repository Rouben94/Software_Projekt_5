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

#include <stdbool.h>
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
#include "generic_level_server.h"
#include "generic_onoff_server.h"

/* Logging and RTT */
#include "log.h"
#include "rtt_input.h"

/* Example specific includes */
#include "app_config.h"
#include "app_level_userdefined.h"
#include "app_switch.h"
#include "ble_softdevice_support.h"
#include "blueseidon_test_example_common.h"
#include "example_common.h"
#include "nrf_mesh_config_examples.h"

#include "node_config.h"

/* Logging */
#include "flash_manager.h" // For storing custom data in flash.

typedef struct
{
    uint32_t config_bit_mask;
} node_configuration_t;

static node_configuration_t node_config;

#define NODE_CONFIGURATION_ENTRY_HANDLE (0x0500)
#define FLASH_PAGE_COUNT (1)

#define SWITCH_SERVER_CONFIG (1)
#define SWITCH_CLIENT_CONFIG (2)
#define LEVEL_SERVER_CONFIG (3)
#define LEVEL_CLIENT_CONFIG (4)

/**** Flash handling ****/

static flash_manager_t m_flash_manager;

static void app_flash_manager_add(void);

static void flash_write_complete(const flash_manager_t *p_manager, const fm_entry_t *p_entry, fm_result_t result)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Flash write complete\n");

    /* If we get an AREA_FULL then our calculations for flash space required are buggy. */
    NRF_MESH_ASSERT(result != FM_RESULT_ERROR_AREA_FULL);

    /* We do not invalidate in this module, so a NOT_FOUND should not be received. */
    NRF_MESH_ASSERT(result != FM_RESULT_ERROR_NOT_FOUND);
    if (result == FM_RESULT_ERROR_FLASH_MALFUNCTION)
    {
        ERROR_CHECK(NRF_ERROR_NO_MEM);
    }
}

static void flash_invalidate_complete(const flash_manager_t *p_manager, fm_handle_t handle, fm_result_t result)
{
    /* This application does not expect invalidate complete calls. */
    ERROR_CHECK(NRF_ERROR_INTERNAL);
}

typedef void (*flash_op_func_t)(void);
static void flash_manager_mem_available(void *p_args)
{
    ((flash_op_func_t)p_args)(); /*lint !e611 Suspicious cast */
}

static void flash_remove_complete(const flash_manager_t *p_manager)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Flash remove complete\n");
}

static void app_flash_manager_add(void)
{

    static fm_mem_listener_t flash_add_mem_available_struct = {
        .callback = flash_manager_mem_available,
        .p_args = app_flash_manager_add};

    const uint32_t *start_address;
    uint32_t allocated_area_size;
    ERROR_CHECK(mesh_stack_persistence_flash_usage(&start_address, &allocated_area_size));

    flash_manager_config_t manager_config;
    manager_config.write_complete_cb = flash_write_complete;
    manager_config.invalidate_complete_cb = flash_invalidate_complete;
    manager_config.remove_complete_cb = flash_remove_complete;
    manager_config.min_available_space = WORD_SIZE;
    manager_config.p_area = (const flash_manager_page_t *)((uint32_t)start_address - PAGE_SIZE * FLASH_PAGE_COUNT);
    manager_config.page_count = FLASH_PAGE_COUNT;
    uint32_t status = flash_manager_add(&m_flash_manager, &manager_config);
    if (NRF_SUCCESS != status)
    {
        flash_manager_mem_listener_register(&flash_add_mem_available_struct);
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Unable to add flash manager for app data\n");
    }
}

static bool load_app_data(void)
{
    uint32_t length = sizeof(node_config);
    uint32_t status = flash_manager_entry_read(&m_flash_manager,
        NODE_CONFIGURATION_ENTRY_HANDLE,
        &node_config,
        &length);
    if (status != NRF_SUCCESS)
    {
        memset(&node_config, 0x00, sizeof(node_config));
    }

    return (status == NRF_SUCCESS);
}

static uint32_t store_app_data(void)
{
    fm_entry_t *p_entry = flash_manager_entry_alloc(&m_flash_manager, NODE_CONFIGURATION_ENTRY_HANDLE, sizeof(node_config));
    static fm_mem_listener_t flash_add_mem_available_struct = {
        .callback = flash_manager_mem_available,
        .p_args = store_app_data};

    if (p_entry == NULL)
    {
        flash_manager_mem_listener_register(&flash_add_mem_available_struct);
    }
    else
    {
        node_configuration_t *p_nw_state = (node_configuration_t *)p_entry->data;
        memcpy(p_nw_state, &node_config, sizeof(node_config));
        flash_manager_entry_commit(p_entry);
    }

    return NRF_SUCCESS;
}

static void clear_app_data(void)
{
    memset(&node_config, 0x00, sizeof(node_config));

    if (flash_manager_remove(&m_flash_manager) != NRF_SUCCESS)
    {
        /* Register the listener and wait for some memory to be freed up before we retry. */
        static fm_mem_listener_t mem_listener = {.callback = flash_manager_mem_available,
            .p_args = clear_app_data};
        flash_manager_mem_listener_register(&mem_listener);
    }
}

static void app_data_store_cb(void)
{
    ERROR_CHECK(store_app_data());
}

void set_node_config(uint32_t config)
{
    node_config.config_bit_mask = config;
    ERROR_CHECK(store_app_data());
}

uint32_t get_node_config(void)
{
    load_app_data();
    return node_config.config_bit_mask;
}

void node_config_flash_init(void)
{
    app_flash_manager_add();
}

/*************************************************************************************************/
#define SWITCH_SERVER_0_PIN (BSP_LED_0)
#define SWITCH_SERVER_1_PIN (BSP_LED_1)
#define SWITCH_SERVER_2_PIN (BSP_LED_2)
#define SWITCH_SERVER_3_PIN (BSP_LED_3)

#define APP_SWITCH_0_ELEMENT_INDEX (0)
#define APP_SWITCH_1_ELEMENT_INDEX (1)
#define APP_SWITCH_2_ELEMENT_INDEX (2)
#define APP_SWITCH_3_ELEMENT_INDEX (3)
#define APP_SWITCH_4_ELEMENT_INDEX (4)

static void app_switch_server_0_set_cb(const app_switch_server_t *p_server, bool switch_state);
static void app_switch_server_0_get_cb(const app_switch_server_t *p_server, bool *p_present_switch);

static void app_switch_server_1_set_cb(const app_switch_server_t *p_server, bool switch_state);
static void app_switch_server_1_get_cb(const app_switch_server_t *p_server, bool *p_present_switch);

static void app_switch_server_2_set_cb(const app_switch_server_t *p_server, bool switch_state);
static void app_switch_server_2_get_cb(const app_switch_server_t *p_server, bool *p_present_switch);

static void app_switch_server_3_set_cb(const app_switch_server_t *p_server, bool switch_state);
static void app_switch_server_3_get_cb(const app_switch_server_t *p_server, bool *p_present_switch);

static void app_level_server_set_cb(const app_level_server_t *p_server, uint32_t present_level);
static void app_level_server_get_cb(const app_level_server_t *p_server, uint32_t *p_present_level);

/* Application level generic level server structure definition and initialization */
APP_LEVEL_SERVER_DEF(m_level_server_0,
    APP_CONFIG_FORCE_SEGMENTATION,
    APP_CONFIG_MIC_SIZE,
    NULL,
    app_level_server_set_cb,
    app_level_server_get_cb);

/* Generic switch server structure definition and initialization */
APP_SWITCH_SERVER_DEF(m_switch_server_0,
    APP_CONFIG_FORCE_SEGMENTATION,
    APP_CONFIG_MIC_SIZE,
    app_switch_server_0_set_cb,
    app_switch_server_0_get_cb)

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

/* Callback for updating the hardware state */
static void app_level_server_set_cb(const app_level_server_t *p_server, uint32_t present_level)
{
    /* Resolve the server instance here if required, this example uses only 1 instance. */
    set_node_config(present_level);
}

/* Callback for updating the hardware state */
static void app_switch_server_0_set_cb(const app_switch_server_t *p_server, bool switch_state)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Setting GPIO value: %d\n", switch_state)

    hal_led_pin_set(SWITCH_SERVER_0_PIN, switch_state);
}

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

/* Callback for reading the hardware state */
static void app_level_server_get_cb(const app_level_server_t *p_server, uint32_t *p_present_level)
{
    /* Resolve the server instance here if required, this example uses only 1 instance. */
    *p_present_level = get_node_config();
}

/* Callback for reading the hardware state */
static void app_switch_server_0_get_cb(const app_switch_server_t *p_server, bool *p_present_switch)
{
    *p_present_switch = hal_led_pin_get(SWITCH_SERVER_0_PIN);
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

/*************************************************************************************************/
void app_model_init(void)
{
    node_config_flash_init();
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

    /* Instantiate level server on element index 0 */
    ERROR_CHECK(app_level_init(&m_level_server_0, APP_SWITCH_0_ELEMENT_INDEX));
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "App level Model 0 Handle: %d\n", m_level_server_0.server.model_handle);

    if (CHANNEL_1 == SWITCH_SERVER_CONFIG)
    {
        /* Instantiate switch server on element index APP_SWITCH_ELEMENT_0_INDEX */
        ERROR_CHECK(app_switch_init(&m_switch_server_0, APP_SWITCH_1_ELEMENT_INDEX));
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "App switch Model 0 Handle: %d\n", m_switch_server_0.server.model_handle);
    }

    if (CHANNEL_2 == SWITCH_SERVER_CONFIG)
    {
        /* Instantiate switch server on element index APP_SWITCH_ELEMENT_1_INDEX */
        ERROR_CHECK(app_switch_init(&m_switch_server_1, APP_SWITCH_2_ELEMENT_INDEX));
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "App switch Model 1 Handle: %d\n", m_switch_server_1.server.model_handle);
    }

    if (CHANNEL_3 == SWITCH_SERVER_CONFIG)
    {
        /* Instantiate switch server on element index APP_SWITCH_ELEMENT_2_INDEX */
        ERROR_CHECK(app_switch_init(&m_switch_server_2, APP_SWITCH_3_ELEMENT_INDEX));
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "App switch Model 2 Handle: %d\n", m_switch_server_2.server.model_handle);
    }

    if (CHANNEL_4 == SWITCH_SERVER_CONFIG)
    {
        /* Instantiate switch server on element index APP_SWITCH_ELEMENT_3_INDEX */
        ERROR_CHECK(app_switch_init(&m_switch_server_3, APP_SWITCH_4_ELEMENT_INDEX));
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "App switch Model 3 Handle: %d\n", m_switch_server_3.server.model_handle);
    }
}

/*************************************************************************************************/