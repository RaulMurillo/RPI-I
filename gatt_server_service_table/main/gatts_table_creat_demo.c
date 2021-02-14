/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/****************************************************************************
*
* This demo showcases creating a GATT database using a predefined attribute table.
* It acts as a GATT server and can send adv data, be connected by client.
* Run the gatt_client demo, the client demo will automatically connect to the gatt_server_service_table demo.
* Client demo will enable GATT server's notify after connection. The two devices will then exchange
* data.
*
****************************************************************************/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "gatts_table_creat_demo.h"
#include "esp_gatt_common_api.h"

/* Wi-Fi Provisioning Configuration */
#include <esp_wifi.h>
#include <esp_event.h>
#include <nvs_flash.h>

#include <lwip/err.h>
#include <lwip/sys.h>

#include "app_prov.h"

#include "uthash.h"
#include <math.h>

#define EXAMPLE_AP_RECONN_ATTEMPTS CONFIG_EXAMPLE_AP_RECONN_ATTEMPTS

#define WIFI_PROV_TAG "RAUL_WIFI_PROV_APP"

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    static int s_retry_num = 0;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (s_retry_num < EXAMPLE_AP_RECONN_ATTEMPTS)
        {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(WIFI_PROV_TAG, "retry to connect to the AP");
        }
        ESP_LOGI(WIFI_PROV_TAG, "connect to the AP fail");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(WIFI_PROV_TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
    }
}

static void wifi_init_sta(void)
{
    /* Set our event handling */
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, event_handler, NULL));

    /* Start Wi-Fi in station mode with credentials set during provisioning */
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
}

static void start_softap_provisioning(void)
{
    /* Security version */
    int security = 0;
    /* Proof of possession */
    const protocomm_security_pop_t *pop = NULL;

#ifdef CONFIG_EXAMPLE_USE_SEC_1
    security = 1;
#endif

    /* Having proof of possession is optional */
#ifdef CONFIG_EXAMPLE_USE_POP
    const static protocomm_security_pop_t app_pop = {
        .data = (uint8_t *)CONFIG_EXAMPLE_POP,
        .len = (sizeof(CONFIG_EXAMPLE_POP) - 1)};
    pop = &app_pop;
#endif

    const char *ssid = NULL;

#ifdef CONFIG_EXAMPLE_SSID
    ssid = CONFIG_EXAMPLE_SSID;
#else
    uint8_t eth_mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, eth_mac);

    char ssid_with_mac[33];
    snprintf(ssid_with_mac, sizeof(ssid_with_mac), "PROV_%02X%02X%02X",
             eth_mac[3], eth_mac[4], eth_mac[5]);

    ssid = ssid_with_mac;
#endif

    ESP_ERROR_CHECK(app_prov_start_softap_provisioning(
        ssid, CONFIG_EXAMPLE_PASS, security, pop));
}

/* End Wi-Fi Provisioning Configuration */

#define GATTS_TABLE_TAG "GATTS_TABLE_RAUL"
#define GATTC_TAG "GATT_SCANNER_RAUL"

#define PROFILE_NUM 1
#define PROFILE_APP_IDX 0
#define ESP_APP_ID 0x55
#define SAMPLE_DEVICE_NAME "ESP_GATTS_RAUL"
#define SVC_INST_ID 0

/* The max length of characteristic value. When the GATT client performs a write or prepare write operation,
*  the data length must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX. 
*/
#define GATTS_DEMO_CHAR_VAL_LEN_MAX 500
#define PREPARE_BUF_MAX_SIZE 1024
#define CHAR_DECLARATION_SIZE (sizeof(uint8_t))

#define ADV_CONFIG_FLAG (1 << 0)
#define SCAN_RSP_CONFIG_FLAG (1 << 1)

static uint8_t adv_config_done = 0;

uint16_t gatt_db_handle_table[CEAQ_IDX_NB];

// static const int measuredPower = -57;
#define MEASURED_POWER CONFIG_MEASURED_POWER
// #define AMBIENTAL_PARAM CONFIG_AMBIENTAL_PARAM


struct device_rssi {
    uint8_t bd_addr[ESP_BD_ADDR_LEN]; /* key */
    int rssi;
    UT_hash_handle hh;         /* makes this structure hashable */
};

typedef struct
{
    uint8_t *prepare_buf;
    int prepare_len;
} prepare_type_env_t;

static prepare_type_env_t prepare_write_env;

#define CONFIG_SET_RAW_ADV_DATA
#ifdef CONFIG_SET_RAW_ADV_DATA
// RPI-I  --  ESP_GATTS_RAUL
static uint8_t raw_adv_data[] = {
    /* flags */
    0x02, 0x01, 0x06,
    /* tx power*/
    0x02, 0x0a, 0xeb,
    /* service uuid */
    0x03, 0x03, 0xFF, 0x00,
    /* device name */
    0x0f, 0x09, 'E', 'S', 'P', '_', 'G', 'A', 'T', 'T', 'S', '_', 'R', 'A', 'U', 'L'};
static uint8_t raw_scan_rsp_data[] = {
    /* flags */
    0x02, 0x01, 0x06,
    /* tx power */
    0x02, 0x0a, 0xeb,
    /* service uuid */
    0x03, 0x03, 0xFF, 0x00};

#else
static uint8_t service_uuid[16] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb,
    0x34,
    0x9b,
    0x5f,
    0x80,
    0x00,
    0x00,
    0x80,
    0x00,
    0x10,
    0x00,
    0x00,
    0xFF,
    0x00,
    0x00,
    0x00,
};

/* The length of adv data must be less than 31 bytes */
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = 0x00,
    .manufacturer_len = 0,       //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL, //test_manufacturer,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(service_uuid),
    .p_service_uuid = service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

// scan response data
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp = true,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006,
    .max_interval = 0x0010,
    .appearance = 0x00,
    .manufacturer_len = 0,       //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL, //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(service_uuid),
    .p_service_uuid = service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};
#endif /* CONFIG_SET_RAW_ADV_DATA */

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50,
    .scan_window            = 0x30,
    .scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE
};

struct gatts_profile_inst
{
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

static struct device_rssi *devices = NULL;    /* important! initialize to NULL */

static float distance(int dev_rssi) {
#ifdef CONFIG_ESTIMATION_TECHNIQUE_1 //General log-distance
#define AMBIENTAL_PARAM strtof(CONFIG_AMBIENTAL_PARAM, NULL)
    return exp10f( (float)(MEASURED_POWER - dev_rssi)/(float)(10 * AMBIENTAL_PARAM) );
#elif CONFIG_ESTIMATION_TECHNIQUE_2 //AltBeacon
    float ratio = (float)dev_rssi/(float)MEASURED_POWER;
    if(ratio<1){
    	return powf(ratio, 10.0);
    }
    return 0.89976 * powf(ratio, 7.7095) + 0.111;
#endif
}

static void add_device(uint8_t *dev_addr, int dev_rssi) {
    struct device_rssi *s;

    // ESP_LOGI(GATTC_TAG, "Adding device with MAC:");
    // esp_log_buffer_hex(GATTC_TAG, dev_addr, 6);

    HASH_FIND_INT(devices, dev_addr, s);  /* dev_addr already in the hash? */
    if (s==NULL) {
        s = (struct device_rssi *)malloc(sizeof *s);
        memcpy(s->bd_addr, dev_addr, ESP_BD_ADDR_LEN);
        HASH_ADD_INT( devices, bd_addr, s );  /* bd_addr: name of key field */
    }
    s->rssi = dev_rssi;
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
                                        esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst gatt_db_profile_tab[PROFILE_NUM] = {
    [PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE, /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

/* Service */
// RPI-I
static const uint16_t GATTS_SERVICE_UUID_CEAQ       = 0x181A;   // Environmental Sensing
static const uint16_t GATTS_CHAR_UUID_FREQ          = 0xFF01;   // Measurement Interval 0x2A21 - 0xFF01
static const uint16_t GATTS_CHAR_UUID_DIST          = 0xFF02;
static const uint16_t GATTS_CHAR_UUID_CE_ACTIVE     = 0xFF03;
static const uint16_t GATTS_CHAR_UUID_CO2_ACTIVE    = 0xFF04;
static const uint16_t GATTS_CHAR_UUID_TMP_ACTIVE    = 0xFF05;   // Temperature Measurement 0x2A1C
static const uint16_t GATTS_CHAR_UUID_TEST_CE       = 0xFF06;
static const uint16_t GATTS_CHAR_UUID_TEST_CO2      = 0xFF07;
static const uint16_t GATTS_CHAR_UUID_TEST_TMP      = 0xFF07;   // Temperature 0x2A6E

static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint16_t character_user_description = ESP_GATT_UUID_CHAR_DESCRIPTION;
static const uint16_t char_format_uuid = ESP_GATT_UUID_CHAR_PRESENT_FORMAT;
// static const uint8_t char_prop_read                =  ESP_GATT_CHAR_PROP_BIT_READ;
// static const uint8_t char_prop_write               = ESP_GATT_CHAR_PROP_BIT_WRITE;
static const uint8_t char_prop_read_write = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t char_prop_read_notify = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
// static const uint8_t char_prop_read_write_notify   = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;

/* First, we define the variables we want to share via GATT.*/

/* Characteristic/descriptor values: Measurement Frequency */
static const uint8_t freq_user_descr[] = "Time interval between measurements (seconds)";
static uint8_t freq_value[] = CONFIG_MEASUREMENT_FREQUENCY; //"2.0";
static float f_freq_value; // strtof(CONFIG_MEASUREMENT_FREQUENCY, NULL); //2.0;
// Characteristic Presentation Format: 0x19: UTF-8 string; 0x00: no exponent; 0x2703: unit = time (second); 0x01: Bluetooth SIG namespace; 0x0000: No description
static const uint8_t freq_present_str[7] = {0x19, 0x00, 0x03, 0x27, 0x01, 0x00, 0x00};

/* Characteristic/descriptor values: Distance */
static const uint8_t dist_user_descr[] = "Distance for people detection (meters)";
static uint8_t dist_value[] = CONFIG_DISTANCE; //"1.50";
static float f_dist_value; // strtof((char *)dist_value, NULL); //1.5;
// Characteristic Presentation Format: 0x19: UTF-8 string; 0x00: no exponent; 0x2701: unit = length (metre); 0x01: Bluetooth SIG namespace; 0x0000: No description
static const uint8_t dist_present_str[7] = {0x19, 0x00, 0x01, 0x27, 0x01, 0x00, 0x00};

/* Characteristic/descriptor values: CE-Active */
static const uint8_t ce_active_user_descr[] = "Activate/deactivate people estimation measurement";
#ifdef CONFIG_ACTIVATE_CE_MEASUREMENT
    static uint8_t ce_active_value[1] = {true};
#else
    static uint8_t ce_active_value[1] = {false};
#endif
// Characteristic Presentation Format: 0x01: boolean; 0x00: no exponent; 0x2700: unit = unitless; 0x01: Bluetooth SIG namespace; 0x0000: No description
static const uint8_t ce_active_present_str[7] = {0x01, 0x00, 0x00, 0x27, 0x01, 0x00, 0x00};

/* Characteristic/descriptor values: CO2-Active */
static const uint8_t co2_active_user_descr[] = "Activate/deactivate CO2 measurement";
#ifdef CONFIG_ACTIVATE_CO2_MEASUREMENT
    static uint8_t co2_active_value[1] = {true};
#else
    static uint8_t co2_active_value[1] = {false};
#endif
// Characteristic Presentation Format: 0x01: boolean; 0x00: no exponent; 0x2700: unit = unitless; 0x01: Bluetooth SIG namespace; 0x0000: No description
static const uint8_t co2_active_present_str[7] = {0x01, 0x00, 0x00, 0x27, 0x01, 0x00, 0x00};

/* Characteristic/descriptor values: TMP-Active */
static const uint8_t tmp_active_user_descr[] = "Activate/deactivate temperature measurement";
#ifdef CONFIG_ACTIVATE_TMP_MEASUREMENT
    static uint8_t tmp_active_value[1] = {true};
#else
    static uint8_t tmp_active_value[1] = {false};
#endif
// Characteristic Presentation Format: 0x01: boolean; 0x00: no exponent; 0x2700: unit = unitless; 0x01: Bluetooth SIG namespace; 0x0000: No description
static const uint8_t tmp_active_present_str[7] = {0x01, 0x00, 0x00, 0x27, 0x01, 0x00, 0x00};

/* Characteristic/descriptor values: Capacity Estimation */
static const uint8_t ce_user_descr[] = "Estimation of people in the room";
static uint8_t ce_value[2] = {0x00, 0x00};
static uint8_t ce_ccc[2] = {0x00, 0x00};
// Characteristic Presentation Format: 0x06: unsigned 16 bit integer; 0x00: no exponent; 0x2700: unit = unitless; 0x01: Bluetooth SIG namespace; 0x0000: No description
static const uint8_t ce_present_str[7] = {0x06, 0x00, 0x00, 0x27, 0x01, 0x00, 0x00};

/* Characteristic/descriptor values: CO2 */
static const uint8_t co2_user_descr[] = "CO2 level (ppm)";
static uint8_t co2_value[2] = {0x00, 0x00};
static uint8_t co2_ccc[2] = {0x00, 0x00};
// Characteristic Presentation Format: 0x06: unsigned 16 bit integer; 0x00: no exponent; 0x2700: unit = unitless; 0x01: Bluetooth SIG namespace; 0x0000: No description
static const uint8_t co2_present_str[7] = {0x06, 0x00, 0x00, 0x27, 0x01, 0x00, 0x00};

/* Characteristic/descriptor values: Temperature */
static const uint8_t tmp_user_descr[] = "Temperature in the room (degree Celsius)";
static uint8_t tmp_value[2] = {0x00, 0x00};
static uint8_t tmp_ccc[2] = {0x00, 0x00};
// Characteristic Presentation Format: 0x0E: signed 16-bit integer; 0x00: no exponent; 0x2700: unit = degree Celsius; 0x01: Bluetooth SIG namespace; 0x0000: No description
static const uint8_t tmp_present_str[7] = {0x0E, 0x00, 0x2F, 0x27, 0x01, 0x00, 0x00};

/* Other */
static TaskHandle_t measure_notify_task = NULL;

/* Full Database Description - Used to add attributes into the database */
static const esp_gatts_attr_db_t gatt_db[CEAQ_IDX_NB] =
    {
        // Capacity Estimation & Air Quality Service
        // Service Declaration
        [CEAQ_IDX_SVC] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ, 
             sizeof(uint16_t), sizeof(GATTS_SERVICE_UUID_CEAQ), (uint8_t *)&GATTS_SERVICE_UUID_CEAQ}},

        /* Measurement Frequency */
        /* Characteristic Declaration */
        [IDX_MEAS_FREQ_CHAR] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, 
             CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

        /* Characteristic Value */
        [IDX_MEAS_FREQ_VAL] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_FREQ, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
             GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(freq_value), (uint8_t *)freq_value}},

        /* Characteristic User Description */
        [IDX_MEAS_FREQ_DESC] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_user_description, ESP_GATT_PERM_READ,
             sizeof(freq_user_descr), sizeof(freq_user_descr), (uint8_t *)freq_user_descr}},

        /* Characteristic Presentation Format */
        [IDX_MEAS_FREQ_FORMAT] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&char_format_uuid, ESP_GATT_PERM_READ, 
             sizeof(freq_present_str), sizeof(freq_present_str), (uint8_t *)freq_present_str}},

        /* Distance */
        /* Characteristic Declaration */
        [IDX_DIST_CHAR] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, 
             CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

        /* Characteristic Value */
        [IDX_DIST_VAL] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_DIST, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
             GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(dist_value), (uint8_t *)dist_value}},

        /* Characteristic User Description */
        [IDX_DIST_DESC] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_user_description, ESP_GATT_PERM_READ,
             sizeof(dist_user_descr), sizeof(dist_user_descr), (uint8_t *)dist_user_descr}},

        /* Characteristic Presentation Format */
        [IDX_DIST_FORMAT] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&char_format_uuid, ESP_GATT_PERM_READ,
             sizeof(dist_present_str), sizeof(dist_present_str), (uint8_t *)dist_present_str}},

        /* CE-Active */
        /* Characteristic Declaration */
        [IDX_CE_ACTIVE_CHAR] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
             CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

        /* Characteristic Value */
        [IDX_CE_ACTIVE_VAL] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_CE_ACTIVE, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
             sizeof(uint8_t), sizeof(ce_active_value), (uint8_t *)ce_active_value}},

        /* Characteristic User Description */
        [IDX_CE_ACTIVE_DESC] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_user_description, ESP_GATT_PERM_READ,
             sizeof(ce_active_user_descr), sizeof(ce_active_user_descr), (uint8_t *)ce_active_user_descr}},

        /* Characteristic Presentation Format */
        [IDX_CE_ACTIVE_FORMAT] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&char_format_uuid, ESP_GATT_PERM_READ,
             sizeof(ce_active_present_str), sizeof(ce_active_present_str), (uint8_t *)ce_active_present_str}},

        /* CO2-Active */
        /* Characteristic Declaration */
        [IDX_CO2_ACTIVE_CHAR] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
             CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

        /* Characteristic Value */
        [IDX_CO2_ACTIVE_VAL] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_CO2_ACTIVE, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
             sizeof(uint8_t), sizeof(co2_active_value), (uint8_t *)co2_active_value}},

        /* Characteristic User Description */
        [IDX_CO2_ACTIVE_DESC] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_user_description, ESP_GATT_PERM_READ,
             sizeof(co2_active_user_descr), sizeof(co2_active_user_descr), (uint8_t *)co2_active_user_descr}},

        /* Characteristic Presentation Format */
        [IDX_CO2_ACTIVE_FORMAT] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&char_format_uuid, ESP_GATT_PERM_READ,
             sizeof(co2_active_present_str), sizeof(co2_active_present_str), (uint8_t *)co2_active_present_str}},

        /* TMP-Active */
        /* Characteristic Declaration */
        [IDX_TMP_ACTIVE_CHAR] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
             CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

        /* Characteristic Value */
        [IDX_TMP_ACTIVE_VAL] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TMP_ACTIVE, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
             sizeof(uint8_t), sizeof(tmp_active_value), (uint8_t *)tmp_active_value}},

        /* Characteristic User Description */
        [IDX_TMP_ACTIVE_DESC] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_user_description, ESP_GATT_PERM_READ,
             sizeof(tmp_active_user_descr), sizeof(tmp_active_user_descr), (uint8_t *)tmp_active_user_descr}},

        /* Characteristic Presentation Format */
        [IDX_TMP_ACTIVE_FORMAT] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&char_format_uuid, ESP_GATT_PERM_READ,
             sizeof(tmp_active_present_str), sizeof(tmp_active_present_str), (uint8_t *)tmp_active_present_str}},

        /* Capacity Estimation */
        /* Characteristic Declaration */
        [IDX_CE_CHAR] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
             CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},

        /* Characteristic Value */
        [IDX_CE_VAL] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEST_CE, ESP_GATT_PERM_READ,
             sizeof(uint16_t), sizeof(ce_value), (uint8_t *)ce_value}},

        /* Characteristic User Description */
        [IDX_CE_DESC] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_user_description, ESP_GATT_PERM_READ,
             sizeof(ce_user_descr), sizeof(ce_user_descr), (uint8_t *)ce_user_descr}},

        /* Client Characteristic Configuration Descriptor */
        [IDX_CE_NTF_CFG] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
             sizeof(uint16_t), sizeof(ce_ccc), (uint8_t *)ce_ccc}},

        /* Characteristic Presentation Format */
        [IDX_CE_FORMAT] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&char_format_uuid, ESP_GATT_PERM_READ,
             sizeof(ce_present_str), sizeof(ce_present_str), (uint8_t *)ce_present_str}},

        /* CO2 */
        /* Characteristic Declaration */
        [IDX_CO2_CHAR] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
             CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},

        /* Characteristic Value */
        [IDX_CO2_VAL] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEST_CO2, ESP_GATT_PERM_READ,
             sizeof(uint16_t), sizeof(co2_value), (uint8_t *)co2_value}},

        /* Characteristic User Description */
        [IDX_CO2_DESC] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_user_description, ESP_GATT_PERM_READ,
             sizeof(co2_user_descr), sizeof(co2_user_descr), (uint8_t *)co2_user_descr}},

        /* Client Characteristic Configuration Descriptor */
        [IDX_CO2_NTF_CFG] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
             sizeof(uint16_t), sizeof(co2_ccc), (uint8_t *)co2_ccc}},

        /* Characteristic Presentation Format */
        [IDX_CO2_FORMAT] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&char_format_uuid, ESP_GATT_PERM_READ,
             sizeof(co2_present_str), sizeof(co2_present_str), (uint8_t *)co2_present_str}},

        /* Temperature */
        /* Characteristic Declaration */
        [IDX_TMP_CHAR] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
             CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},

        /* Characteristic Value */
        [IDX_TMP_VAL] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEST_TMP, ESP_GATT_PERM_READ,
             sizeof(uint16_t), sizeof(tmp_value), (uint8_t *)tmp_value}},

        /* Characteristic User Description */
        [IDX_TMP_DESC] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_user_description, ESP_GATT_PERM_READ,
             sizeof(tmp_user_descr), sizeof(tmp_user_descr), (uint8_t *)tmp_user_descr}},

        /* Client Characteristic Configuration Descriptor */
        [IDX_TMP_NTF_CFG] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
             sizeof(uint16_t), sizeof(tmp_ccc), (uint8_t *)tmp_ccc}},

        /* Characteristic Presentation Format */
        [IDX_TMP_FORMAT] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&char_format_uuid, ESP_GATT_PERM_READ,
             sizeof(tmp_present_str), sizeof(tmp_present_str), (uint8_t *)tmp_present_str}},

};

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    // uint32_t duration = 5;
    switch (event)
    {
    // SCAN EVENTS
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: 
        //the unit of the duration is second
        //duration = 5;     //************
        // esp_ble_gap_start_scanning(duration);
        break;
    
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        //scan start complete event to indicate scan start successfully or failed
        if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTC_TAG, "scan start failed, error status = %x", param->scan_start_cmpl.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "scan start success");
        break;

    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        switch (scan_result->scan_rst.search_evt) {
        case ESP_GAP_SEARCH_INQ_RES_EVT: {
            // uint8_t *adv_name = NULL;
            // uint8_t adv_name_len = 0;
            // esp_log_buffer_hex(GATTC_TAG, scan_result->scan_rst.bda, 6);
            // ESP_LOGI(GATTC_TAG, "searched Adv Data Len %d, Scan Response Len %d", scan_result->scan_rst.adv_data_len, scan_result->scan_rst.scan_rsp_len);
            // adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv,
            //                                     ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);
            // ESP_LOGI(GATTC_TAG, "searched Device Name Len %d", adv_name_len);
            // esp_log_buffer_char(GATTC_TAG, adv_name, adv_name_len);
            // ESP_LOGI(GATTC_TAG, "RSSI: %d", scan_result->scan_rst.rssi);

            // Add to hash table if meets condition
            add_device(scan_result->scan_rst.bda, scan_result->scan_rst.rssi);
            break;
        }
        case ESP_GAP_SEARCH_INQ_CMPL_EVT:
            // ESP_LOGI(GATTC_TAG, "INQ COMPLETE EVENT");
            ESP_LOGI(GATTC_TAG, "A total of %u devices were detected in the range\n", HASH_COUNT(devices));
            break;
        default:
            break;
        }
        break;
    }
    // ADVERTISE EVENTS
#ifdef CONFIG_SET_RAW_ADV_DATA
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
        adv_config_done &= (~ADV_CONFIG_FLAG);
        if (adv_config_done == 0)
        {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
        adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
        if (adv_config_done == 0)
        {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
#else
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~ADV_CONFIG_FLAG);
        if (adv_config_done == 0)
        {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
        if (adv_config_done == 0)
        {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
#endif
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        /* advertising start complete event to indicate advertising start successfully or failed */
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "advertising start failed");
        }
        else
        {
            ESP_LOGI(GATTS_TABLE_TAG, "advertising start successfully");
        }
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "Advertising stop failed");
        }
        else
        {
            ESP_LOGI(GATTS_TABLE_TAG, "Stop adv successfully\n");
        }
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
        ESP_LOGI(GATTS_TABLE_TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                 param->update_conn_params.status,
                 param->update_conn_params.min_int,
                 param->update_conn_params.max_int,
                 param->update_conn_params.conn_int,
                 param->update_conn_params.latency,
                 param->update_conn_params.timeout);
        break;
    default:
        break;
    }
}

void example_prepare_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGI(GATTS_TABLE_TAG, "prepare write, handle = %d, value len = %d", param->write.handle, param->write.len);
    esp_gatt_status_t status = ESP_GATT_OK;
    if (prepare_write_env->prepare_buf == NULL)
    {
        prepare_write_env->prepare_buf = (uint8_t *)malloc(PREPARE_BUF_MAX_SIZE * sizeof(uint8_t));
        prepare_write_env->prepare_len = 0;
        if (prepare_write_env->prepare_buf == NULL)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "%s, Gatt_server prep no mem", __func__);
            status = ESP_GATT_NO_RESOURCES;
        }
    }
    else
    {
        if (param->write.offset > PREPARE_BUF_MAX_SIZE)
        {
            status = ESP_GATT_INVALID_OFFSET;
        }
        else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE)
        {
            status = ESP_GATT_INVALID_ATTR_LEN;
        }
    }
    /*send response when param->write.need_rsp is true */
    if (param->write.need_rsp)
    {
        esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
        if (gatt_rsp != NULL)
        {
            gatt_rsp->attr_value.len = param->write.len;
            gatt_rsp->attr_value.handle = param->write.handle;
            gatt_rsp->attr_value.offset = param->write.offset;
            gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
            memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
            esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);
            if (response_err != ESP_OK)
            {
                ESP_LOGE(GATTS_TABLE_TAG, "Send response error");
            }
            free(gatt_rsp);
        }
        else
        {
            ESP_LOGE(GATTS_TABLE_TAG, "%s, malloc failed", __func__);
        }
    }
    if (status != ESP_GATT_OK)
    {
        return;
    }
    memcpy(prepare_write_env->prepare_buf + param->write.offset,
           param->write.value,
           param->write.len);
    prepare_write_env->prepare_len += param->write.len;
}

void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param)
{
    if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC && prepare_write_env->prepare_buf)
    {
        esp_log_buffer_hex(GATTS_TABLE_TAG, prepare_write_env->prepare_buf, prepare_write_env->prepare_len);
    }
    else
    {
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATT_PREP_WRITE_CANCEL");
    }
    if (prepare_write_env->prepare_buf)
    {
        free(prepare_write_env->prepare_buf);
        prepare_write_env->prepare_buf = NULL;
    }
    prepare_write_env->prepare_len = 0;
}

void taskNotify(enum characteristic idx, uint8_t *notify_data)
{
    esp_ble_gatts_send_indicate(gatt_db_profile_tab[0].gatts_if,
                                gatt_db_profile_tab[0].conn_id,
                                gatt_db_handle_table[idx],
                                sizeof(notify_data),
                                notify_data,
                                false);
}

void ce_measure(void)
{
    uint16_t count_dev = 0;

    struct device_rssi *current, *tmp;

    // Iterate and remove hash table
    HASH_ITER(hh, devices, current, tmp) {
        // ESP_LOGI(GATTC_TAG, "Device addr:");
        // esp_log_buffer_hex(GATTC_TAG, current->bd_addr, ESP_BD_ADDR_LEN);
        // ESP_LOGI(GATTC_TAG, "Estimated distance %.4f m\n", current->dist);
        float dev_dist = distance(current->rssi);
        if (dev_dist <= f_dist_value){
            count_dev++;
        }
        /* ... it is safe to delete and free current here */
        HASH_DEL(devices, current);  /* delete; devices advances to next */
        free(current);            /* optional- if you want to free  */
    }

    // ce_value[0] = count_dev;
    *(uint16_t*)&ce_value = count_dev;
    // Update GATT table
    esp_err_t set_attr_value_ret = esp_ble_gatts_set_attr_value(gatt_db_handle_table[IDX_CE_VAL],
                                                                sizeof(ce_value),
                                                                ce_value);
    if (set_attr_value_ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "set attribute value failed, error code = %x", set_attr_value_ret);
    }
}

void co2_measure(void)
{
    // Random value
    for (int i = 0; i < sizeof(co2_value); ++i)
    {
        co2_value[i] = rand() % 0xff;
    }
    // Update GATT table
    esp_err_t set_attr_value_ret = esp_ble_gatts_set_attr_value(gatt_db_handle_table[IDX_CO2_VAL],
                                                                sizeof(co2_value),
                                                                co2_value);
    if (set_attr_value_ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "set attribute value failed, error code = %x", set_attr_value_ret);
    }
}

void tmp_measure(void)
{
    // Random value
    for (int i = 0; i < sizeof(tmp_value); ++i)
    {
        tmp_value[i] = rand() % 0xff;
    }
    // Update GATT table
    esp_err_t set_attr_value_ret = esp_ble_gatts_set_attr_value(gatt_db_handle_table[IDX_TMP_VAL],
                                                                sizeof(tmp_value),
                                                                tmp_value);
    if (set_attr_value_ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "set attribute value failed, error code = %x", set_attr_value_ret);
    }
}

static void measure_notify(void *pvParameters)
{
    while (1)
    {
        // ESP_LOGI(GATTS_TABLE_TAG, "measure_notify task");
        // Measure
        if (ce_active_value[0])
            ce_measure();
        if (co2_active_value[0])
            co2_measure();
        if (tmp_active_value[0])
            tmp_measure();

        // Notify
        if (ce_ccc[0] == 0x01 && ce_ccc[1] == 0x00 && ce_active_value[0])
            taskNotify(IDX_CE_VAL, ce_value);
        if (co2_ccc[0] == 0x01 && co2_ccc[1] == 0x00 && co2_active_value[0])
            taskNotify(IDX_CO2_VAL, co2_value);
        if (tmp_ccc[0] == 0x01 && tmp_ccc[1] == 0x00 && tmp_active_value[0])
            taskNotify(IDX_TMP_VAL, tmp_value);

        // Sleep
        if (ce_active_value[0])
            esp_ble_gap_start_scanning((uint32_t)f_freq_value);
        vTaskDelay((f_freq_value + 0.1) * 1000 / portTICK_PERIOD_MS);
    }
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event)
    {
    case ESP_GATTS_REG_EVT:
    {
        esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(SAMPLE_DEVICE_NAME);
        if (set_dev_name_ret)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "set device name failed, error code = %x", set_dev_name_ret);
        }

        esp_err_t scan_ret = esp_ble_gap_set_scan_params(&ble_scan_params);
        if (scan_ret){
            ESP_LOGE(GATTC_TAG, "set scan params error, error code = %x", scan_ret);
        }
#ifdef CONFIG_SET_RAW_ADV_DATA
        esp_err_t raw_adv_ret = esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof(raw_adv_data));
        if (raw_adv_ret)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "config raw adv data failed, error code = %x ", raw_adv_ret);
        }
        adv_config_done |= ADV_CONFIG_FLAG;
        esp_err_t raw_scan_ret = esp_ble_gap_config_scan_rsp_data_raw(raw_scan_rsp_data, sizeof(raw_scan_rsp_data));
        if (raw_scan_ret)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "config raw scan rsp data failed, error code = %x", raw_scan_ret);
        }
        adv_config_done |= SCAN_RSP_CONFIG_FLAG;
#else
        //config adv data
        esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
        if (ret)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "config adv data failed, error code = %x", ret);
        }
        adv_config_done |= ADV_CONFIG_FLAG;
        //config scan response data
        ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
        if (ret)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "config scan response data failed, error code = %x", ret);
        }
        adv_config_done |= SCAN_RSP_CONFIG_FLAG;
#endif
        esp_err_t create_attr_ret = esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, CEAQ_IDX_NB, SVC_INST_ID);
        if (create_attr_ret)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "create attr table failed, error code = %x", create_attr_ret);
        }
    }
    break;
    case ESP_GATTS_READ_EVT:
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_READ_EVT");
        break;
    case ESP_GATTS_WRITE_EVT:
        if (!param->write.is_prep)
        {
            // the data length of gattc write  must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
            ESP_LOGI(GATTS_TABLE_TAG, "GATT_WRITE_EVT, handle = %d, value len = %d, value :", param->write.handle, param->write.len);
            esp_log_buffer_hex(GATTS_TABLE_TAG, param->write.value, param->write.len);
            // System configuration handlers
            if (gatt_db_handle_table[IDX_MEAS_FREQ_VAL] == param->write.handle)
            {
                uint16_t length = 0;
                const uint8_t *character;
                esp_gatt_status_t status = esp_ble_gatts_get_attr_value(param->write.handle,
                                                                        &length,
                                                                        &character);
                if (status != ESP_GATT_OK)
                {
                    return;
                }
                if (strtof((char *)character, NULL) > 0)
                {
                    // Input param is a valid value
                    memset(freq_value,0,sizeof(freq_value));
                    strncpy((char *)freq_value, (char *)character, length);
                    f_freq_value = strtof((char *)freq_value, NULL);
                    ESP_LOGI(GATTS_TABLE_TAG, "New Measurement Frequency value: %.2f secs", f_freq_value);
                }
                else
                {
                    // TODO: Notify user?
                    esp_err_t set_attr_value_ret = esp_ble_gatts_set_attr_value(gatt_db_handle_table[IDX_MEAS_FREQ_VAL],
                                                                                sizeof(freq_value),
                                                                                freq_value);
                    if (set_attr_value_ret)
                    {
                        ESP_LOGE(GATTS_TABLE_TAG, "set attribute value failed, error code = %x", set_attr_value_ret);
                    }
                }
            }
            else if (gatt_db_handle_table[IDX_DIST_VAL] == param->write.handle)
            {
                uint16_t length = 0;
                const uint8_t *character;
                esp_gatt_status_t status = esp_ble_gatts_get_attr_value(param->write.handle,
                                                                        &length,
                                                                        &character);
                if (status != ESP_GATT_OK)
                {
                    return;
                }
                if (strtof((char *)character, NULL) > 0)
                {
                    // Input param is a valid value
                    memset(dist_value,0,sizeof(dist_value));
                    strncpy((char *)dist_value, (char *)character, length);
                    f_dist_value = strtof((char *)dist_value, NULL);
                    ESP_LOGI(GATTS_TABLE_TAG, "New Distance value: %.2f m", f_dist_value);
                }
                else
                {
                    // TODO: Notify user?
                    esp_err_t set_attr_value_ret = esp_ble_gatts_set_attr_value(gatt_db_handle_table[IDX_DIST_VAL],
                                                                                sizeof(dist_value),
                                                                                dist_value);
                    if (set_attr_value_ret)
                    {
                        ESP_LOGE(GATTS_TABLE_TAG, "set attribute value failed, error code = %x", set_attr_value_ret);
                    }
                }
            }
            else if (gatt_db_handle_table[IDX_CE_ACTIVE_VAL] == param->write.handle)
            {
                ce_active_value[0] = param->write.value[0];
            }
            else if (gatt_db_handle_table[IDX_CO2_ACTIVE_VAL] == param->write.handle)
            {
                co2_active_value[0] = param->write.value[0];
            }
            else if (gatt_db_handle_table[IDX_TMP_ACTIVE_VAL] == param->write.handle)
            {
                tmp_active_value[0] = param->write.value[0];
            }
            // Notify handlers
            else if (gatt_db_handle_table[IDX_CE_NTF_CFG] == param->write.handle && param->write.len == 2)
            {
                uint16_t descr_value = param->write.value[1] << 8 | param->write.value[0];
                ce_ccc[0] = param->write.value[0];
                ce_ccc[1] = param->write.value[1];
                if (descr_value == 0x0001)
                {
                    ESP_LOGI(GATTS_TABLE_TAG, "CE notify enable");
                }
                else if (descr_value == 0x0002)
                {
                    ESP_LOGI(GATTS_TABLE_TAG, "CE indicate enable");
                    esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gatt_db_handle_table[IDX_CE_VAL],
                                                sizeof(ce_value), ce_value, true);
                }
                else if (descr_value == 0x0000)
                {
                    ESP_LOGI(GATTS_TABLE_TAG, "CE notify/indicate disable ");
                }
                else
                {
                    ESP_LOGE(GATTS_TABLE_TAG, "unknown descr value");
                    esp_log_buffer_hex(GATTS_TABLE_TAG, param->write.value, param->write.len);
                }
            }
            else if (gatt_db_handle_table[IDX_CO2_NTF_CFG] == param->write.handle && param->write.len == 2)
            {
                uint16_t descr_value = param->write.value[1] << 8 | param->write.value[0];
                co2_ccc[0] = param->write.value[0];
                co2_ccc[1] = param->write.value[1];
                if (descr_value == 0x0001)
                {
                    ESP_LOGI(GATTS_TABLE_TAG, "CO2 notify enable");
                }
                else if (descr_value == 0x0002)
                {
                    ESP_LOGI(GATTS_TABLE_TAG, "CO2 indicate enable");
                    esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gatt_db_handle_table[IDX_CE_VAL],
                                                sizeof(co2_value), co2_value, true);
                }
                else if (descr_value == 0x0000)
                {
                    ESP_LOGI(GATTS_TABLE_TAG, "CO2 notify/indicate disable ");
                    // if (co2_notify_task != NULL)
                    // {
                    //     vTaskSuspend(co2_notify_task);
                    // }
                }
                else
                {
                    ESP_LOGE(GATTS_TABLE_TAG, "unknown descr value");
                    esp_log_buffer_hex(GATTS_TABLE_TAG, param->write.value, param->write.len);
                }
            }
            else if (gatt_db_handle_table[IDX_TMP_NTF_CFG] == param->write.handle && param->write.len == 2)
            {
                uint16_t descr_value = param->write.value[1] << 8 | param->write.value[0];
                tmp_ccc[0] = param->write.value[0];
                tmp_ccc[1] = param->write.value[1];
                if (descr_value == 0x0001)
                {
                    ESP_LOGI(GATTS_TABLE_TAG, "TMP notify enable");
                }
                else if (descr_value == 0x0002)
                {
                    ESP_LOGI(GATTS_TABLE_TAG, "TMP indicate enable");
                    esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gatt_db_handle_table[IDX_CE_VAL],
                                                sizeof(tmp_value), tmp_value, true);
                }
                else if (descr_value == 0x0000)
                {
                    ESP_LOGI(GATTS_TABLE_TAG, "TMP notify/indicate disable ");
                }
                else
                {
                    ESP_LOGE(GATTS_TABLE_TAG, "unknown descr value");
                    esp_log_buffer_hex(GATTS_TABLE_TAG, param->write.value, param->write.len);
                }
            }
            /* send response when param->write.need_rsp is true*/
            if (param->write.need_rsp)
            {
                esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
            }
        }
        else
        {
            /* handle prepare write */
            example_prepare_write_event_env(gatts_if, &prepare_write_env, param);
        }
        break;
    case ESP_GATTS_EXEC_WRITE_EVT:
        // the length of gattc prepare write data must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_EXEC_WRITE_EVT");
        example_exec_write_event_env(&prepare_write_env, param);
        break;
    case ESP_GATTS_MTU_EVT:
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
        break;
    case ESP_GATTS_CONF_EVT:
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONF_EVT, status = %d, attr_handle %d", param->conf.status, param->conf.handle);
        break;
    case ESP_GATTS_START_EVT:
        ESP_LOGI(GATTS_TABLE_TAG, "SERVICE_START_EVT, status %d, service_handle %d", param->start.status, param->start.service_handle);
        break;
    case ESP_GATTS_CONNECT_EVT:
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONNECT_EVT, conn_id = %d", param->connect.conn_id);
        esp_log_buffer_hex(GATTS_TABLE_TAG, param->connect.remote_bda, 6);
        esp_ble_conn_update_params_t conn_params = {0};
        memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        /* For the iOS system, please refer to Apple official documents about the BLE connection parameters restrictions. */
        conn_params.latency = 0;
        conn_params.max_int = 0x20; // max_int = 0x20*1.25ms = 40ms
        conn_params.min_int = 0x10; // min_int = 0x10*1.25ms = 20ms
        conn_params.timeout = 400;  // timeout = 400*10ms = 4000ms
        //start sent the update connection parameters to the peer device.
        esp_ble_gap_update_conn_params(&conn_params);
        //start measuring and notifying
        if (measure_notify_task == NULL)
        {
            xTaskCreate(
                (TaskFunction_t)&measure_notify,
                "measure_notify",
                2048,
                NULL,
                5,
                &measure_notify_task);
        }
        else
        {
            vTaskResume(measure_notify_task);
        }
        break;
    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_DISCONNECT_EVT, reason = 0x%x", param->disconnect.reason);
        esp_ble_gap_start_advertising(&adv_params);
        //stop measuring and notifying
        if (measure_notify_task != NULL)
        {
            vTaskSuspend(measure_notify_task);
        }
        break;
    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
    {
        if (param->add_attr_tab.status != ESP_GATT_OK)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "create attribute table failed, error code=0x%x", param->add_attr_tab.status);
        }
        else if (param->add_attr_tab.num_handle != CEAQ_IDX_NB)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "create attribute table abnormally, num_handle (%d) \
                        doesn't equal to CEAQ_IDX_NB(%d)",
                        param->add_attr_tab.num_handle, CEAQ_IDX_NB);
        }
        else
        {
            ESP_LOGI(GATTS_TABLE_TAG, "create attribute table successfully, the number handle = %d\n", param->add_attr_tab.num_handle);
            memcpy(gatt_db_handle_table, param->add_attr_tab.handles, sizeof(gatt_db_handle_table));
            esp_ble_gatts_start_service(gatt_db_handle_table[CEAQ_IDX_SVC]);
        }
        break;
    }
    case ESP_GATTS_STOP_EVT:
    case ESP_GATTS_OPEN_EVT:
    case ESP_GATTS_CANCEL_OPEN_EVT:
    case ESP_GATTS_CLOSE_EVT:
    case ESP_GATTS_LISTEN_EVT:
    case ESP_GATTS_CONGEST_EVT:
    case ESP_GATTS_UNREG_EVT:
    case ESP_GATTS_DELETE_EVT:
    default:
        break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{

    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT)
    {
        if (param->reg.status == ESP_GATT_OK)
        {
            gatt_db_profile_tab[PROFILE_APP_IDX].gatts_if = gatts_if;
        }
        else
        {
            ESP_LOGE(GATTS_TABLE_TAG, "reg app failed, app_id %04x, status %d",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }
    do
    {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++)
        {
            /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
            if (gatts_if == ESP_GATT_IF_NONE || gatts_if == gatt_db_profile_tab[idx].gatts_if)
            {
                if (gatt_db_profile_tab[idx].gatts_cb)
                {
                    gatt_db_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

void app_main(void)

{
    /* Initialize static variables */
    f_freq_value = strtof((char *)freq_value, NULL);
    f_dist_value = strtof((char *)dist_value, NULL);

    /* Initialize networking stack */
    ESP_ERROR_CHECK(esp_netif_init());

    /* Create default event loop needed by the
     * main app and the provisioning service */
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_err_t ret;

    /* Initialize NVS needed by BT & Wi-Fi */
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Initialize Bluetooth */

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "gatts register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "gap register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gatts_app_register(ESP_APP_ID);
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "gatts app register error, error code = %x", ret);
        return;
    }

    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }

    /* Initialize Wi-Fi including netif with default config */
    esp_netif_create_default_wifi_sta();
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    /* Check if device is provisioned */
    bool provisioned;
    if (app_prov_is_provisioned(&provisioned) != ESP_OK)
    {
        ESP_LOGE(WIFI_PROV_TAG, "Error getting device provisioning state");
        return;
    }

    if (provisioned == false)
    {
        /* If not provisioned, start provisioning via soft AP */
        ESP_LOGI(WIFI_PROV_TAG, "Starting WiFi SoftAP provisioning");
        start_softap_provisioning();
    }
    else
    {
        /* Start WiFi station with credentials set during provisioning */
        ESP_LOGI(WIFI_PROV_TAG, "Starting WiFi station");
        wifi_init_sta();
    }
}
