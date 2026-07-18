/**
 * ble_mesh.c — Bluetooth LE Mesh for sensor network
 *
 * Provision ESP32-C3 as mesh node.
 * Models: Generic OnOff Server (LED control), Sensor Server (cliff sensor status).
 * Publish sensor data every 1 second.
 * Friend node for low-power drop sensors.
 * Uses BlueZ D-Bus mesh API for BLE mesh control.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <pthread.h>
#include <time.h>
#include <errno.h>
#include <dbus/dbus.h>

#include "ble_mesh.h"

/* ---------------------------------------------------------------------------
 * Constants
 * --------------------------------------------------------------------------- */
#define BLE_MESH_DBUS_NAME          "org.bluez.mesh"
#define BLE_MESH_DBUS_PATH          "/org/bluez/mesh"
#define BLE_MESH_APP_DBUS_NAME      "org.robot.mesh"
#define BLE_MESH_APP_DBUS_PATH      "/org/robot/mesh"

#define MESH_PUBLISH_INTERVAL_MS    1000   /* sensor publish every 1s */
#define MESH_FRIEND_TIMEOUT         3600   /* friend node timeout (s) */
#define MESH_TTL_DEFAULT            7
#define MESH_NET_KEY_INDEX          0

/* Mesh model IDs */
#define MODEL_GENERIC_ONOFF_SERVER  0x1000
#define MODEL_SENSOR_SERVER         0x1102
#define MODEL_CONFIG_SERVER         0x0000
#define MODEL_HEALTH_SERVER         0x0002

/* Element indexes */
#define ELEMENT_PRIMARY             0
#define ELEMENT_SENSOR              1

/* Sensor property IDs (GATT Bluetooth SIG) */
#define SENSOR_PROP_CLIFF_DETECTED  0x005B

/* ---------------------------------------------------------------------------
 * Context
 * --------------------------------------------------------------------------- */
typedef struct {
    DBusConnection *dbus_conn;
    pthread_t       rx_thread;
    int             keep_running;
    int             provisioned;
    uint16_t        primary_addr;
    uint16_t        element_count;

    /* Sensor cache */
    struct {
        uint8_t  cliff_sensors[8];    /* bitmask of cliff sensor states */
        int      cliff_count;
        uint8_t  battery_pct;
    } sensors;

    /* LED states */
    struct {
        uint8_t  status_led;   /* 0=off, 1=on */
        uint8_t  wifi_led;
    } leds;

    pthread_mutex_t lock;
    uint64_t        last_publish_ms;

    /* Provisioning info */
    uint8_t  net_key[16];
    uint8_t  app_key[16];
    uint8_t  dev_key[16];
    uint16_t net_key_idx;
    uint16_t app_key_idx;
} ble_mesh_ctx_t;

static ble_mesh_ctx_t g_ctx;

/* ---------------------------------------------------------------------------
 * Forward declarations
 * --------------------------------------------------------------------------- */
static int  dbus_init(void);
static void dbus_cleanup(void);
static int  mesh_attach(void);
static int  mesh_publish(uint16_t model_id, uint16_t element_idx,
                          const uint8_t *data, size_t len);
static int  mesh_send_sensor_status(void);
static void mesh_rx_loop(void);
static void dbus_msg_handler(DBusMessage *msg);

/* ---------------------------------------------------------------------------
 * D-Bus initialization
 * --------------------------------------------------------------------------- */
static int dbus_init(void) {
    DBusError err;
    dbus_error_init(&err);

    g_ctx.dbus_conn = dbus_bus_get(DBUS_BUS_SYSTEM, &err);
    if (!g_ctx.dbus_conn) {
        fprintf(stderr, "[BLEMesh] D-Bus connection failed: %s\n", err.message);
        dbus_error_free(&err);
        return -1;
    }

    /* Register application bus name */
    int ret = dbus_bus_request_name(g_ctx.dbus_conn, BLE_MESH_APP_DBUS_NAME,
                                     DBUS_NAME_FLAG_REPLACE_EXISTING, &err);
    if (ret != DBUS_REQUEST_NAME_REPLY_PRIMARY_OWNER) {
        fprintf(stderr, "[BLEMesh] Failed to request bus name: %s\n", err.message);
        dbus_error_free(&err);
        return -1;
    }

    dbus_connection_set_exit_on_disconnect(g_ctx.dbus_conn, 0);
    fprintf(stdout, "[BLEMesh] D-Bus connected as %s\n", BLE_MESH_APP_DBUS_NAME);
    return 0;
}

static void dbus_cleanup(void) {
    if (g_ctx.dbus_conn) {
        dbus_connection_close(g_ctx.dbus_conn);
        dbus_connection_unref(g_ctx.dbus_conn);
        g_ctx.dbus_conn = NULL;
    }
}

/* ---------------------------------------------------------------------------
 * Call a D-Bus method
 * --------------------------------------------------------------------------- */
static DBusMessage *dbus_call_method(const char *bus_name, const char *path,
                                      const char *iface, const char *method,
                                      DBusMessage *args, int timeout_ms) {
    DBusError err;
    dbus_error_init(&err);

    DBusMessage *msg = dbus_message_new_method_call(bus_name, path, iface, method);
    if (!msg) return NULL;

    /* Append arguments if provided */
    if (args) {
        DBusMessageIter iter, args_iter;
        dbus_message_iter_init(args, &args_iter);
        dbus_message_iter_init_append(msg, &iter);
        /* Simple copy - for production use a proper type marshaller */
        /* Here we just pass raw message for simplicity */
        dbus_message_copy(msg, args);
    }

    DBusMessage *reply = dbus_connection_send_with_reply_and_block(
        g_ctx.dbus_conn, msg, timeout_ms > 0 ? timeout_ms : 5000, &err);

    dbus_message_unref(msg);

    if (!reply) {
        fprintf(stderr, "[BLEMesh] D-Bus call %s.%s failed: %s\n",
                iface, method, err.message);
        dbus_error_free(&err);
        return NULL;
    }

    return reply;
}

/* ---------------------------------------------------------------------------
 * Mesh attach: attach to mesh network via BlueZ mesh API
 * --------------------------------------------------------------------------- */
static int mesh_attach(void) {
    /* Mesh attach method: org.bluez.mesh.Application1.Attach */
    DBusMessage *args = dbus_message_new_method_call(NULL, NULL, NULL, NULL);
    if (!args) return -1;

    DBusMessageIter iter;
    dbus_message_iter_init_append(args, &iter);

    /* Element configuration */
    /* Primary element: Config Server + Generic OnOff Server + Health Server */
    DBusMessageIter element_array;
    dbus_message_iter_open_container(&iter, DBUS_TYPE_ARRAY,
                                      "(ya{sv})", &element_array);

    /* Element 0: primary — Config Server, OnOff Server, Health Server */
    for (int el = 0; el < 2; el++) {
        DBusMessageIter element_struct;
        dbus_message_iter_open_container(&element_array, DBUS_TYPE_STRUCT,
                                          NULL, &element_struct);

        uint8_t element_idx = (uint8_t)el;
        dbus_message_iter_append_basic(&element_struct, DBUS_TYPE_BYTE, &element_idx);

        DBusMessageIter model_array;
        dbus_message_iter_open_container(&element_struct, DBUS_TYPE_ARRAY,
                                          "(ya{sv})", &model_array);

        /* Model: Generic OnOff Server (0x1000) */
        uint16_t model_ids[] = { MODEL_GENERIC_ONOFF_SERVER, MODEL_SENSOR_SERVER };
        int num_models = (el == 0) ? 1 : 1;
        if (el == 0) {
            model_ids[0] = MODEL_GENERIC_ONOFF_SERVER;
            num_models = 1;
        } else {
            model_ids[0] = MODEL_SENSOR_SERVER;
            num_models = 1;
        }

        for (int m = 0; m < num_models; m++) {
            DBusMessageIter model_struct;
            dbus_message_iter_open_container(&model_array, DBUS_TYPE_STRUCT,
                                              NULL, &model_struct);

            uint16_t mid = model_ids[m];
            dbus_message_iter_append_basic(&model_struct, DBUS_TYPE_UINT16, &mid);

            /* Empty properties dict */
            DBusMessageIter props_dict;
            dbus_message_iter_open_container(&model_struct, DBUS_TYPE_ARRAY,
                                              "{sv}", &props_dict);
            dbus_message_iter_close_container(&model_struct, &props_dict);

            dbus_message_iter_close_container(&model_array, &model_struct);
        }

        dbus_message_iter_close_container(&element_struct, &model_array);
        dbus_message_iter_close_container(&element_array, &element_struct);
    }

    dbus_message_iter_close_container(&iter, &element_array);

    DBusMessage *reply = dbus_call_method(
        BLE_MESH_DBUS_NAME, BLE_MESH_DBUS_PATH,
        "org.bluez.mesh.Network1", "Attach", args, 10000);
    dbus_message_unref(args);

    if (!reply) return -1;

    /* Parse primary address from reply */
    DBusMessageIter reply_iter;
    dbus_message_iter_init(reply, &reply_iter);

    if (dbus_message_iter_get_arg_type(&reply_iter) == DBUS_TYPE_UINT16) {
        dbus_message_iter_get_basic(&reply_iter, &g_ctx.primary_addr);
        g_ctx.provisioned = 1;
        fprintf(stdout, "[BLEMesh] Attached to mesh, primary addr: 0x%04x\n",
                g_ctx.primary_addr);
    }

    dbus_message_unref(reply);
    return 0;
}

/* ---------------------------------------------------------------------------
 * Publish a mesh message
 * --------------------------------------------------------------------------- */
static int mesh_publish(uint16_t model_id, uint16_t element_idx,
                         const uint8_t *data, size_t len) {
    if (!g_ctx.provisioned || !g_ctx.dbus_conn) return -1;

    DBusMessage *msg = dbus_message_new_method_call(
        BLE_MESH_DBUS_NAME, BLE_MESH_DBUS_PATH,
        "org.bluez.mesh.Element1", "Publish");
    if (!msg) return -1;

    DBusMessageIter iter;
    dbus_message_iter_init_append(msg, &iter);

    uint16_t idx = element_idx;
    dbus_message_iter_append_basic(&iter, DBUS_TYPE_UINT16, &idx);
    dbus_message_iter_append_basic(&iter, DBUS_TYPE_UINT16, &model_id);

    DBusMessageIter array_iter;
    dbus_message_iter_open_container(&iter, DBUS_TYPE_ARRAY, "y", &array_iter);
    for (size_t i = 0; i < len; i++)
        dbus_message_iter_append_basic(&array_iter, DBUS_TYPE_BYTE, &data[i]);
    dbus_message_iter_close_container(&iter, &array_iter);

    dbus_connection_send(g_ctx.dbus_conn, msg, NULL);
    dbus_message_unref(msg);
    return 0;
}

/* ---------------------------------------------------------------------------
 * Send sensor status over mesh
 * --------------------------------------------------------------------------- */
static int mesh_send_sensor_status(void) {
    pthread_mutex_lock(&g_ctx.lock);

    /* Build sensor status message per Mesh Device Properties spec */
    uint8_t msg[16];
    size_t len = 0;

    /* Property ID: cliff detected (2 bytes) */
    msg[len++] = (SENSOR_PROP_CLIFF_DETECTED >> 8) & 0xFF;
    msg[len++] = SENSOR_PROP_CLIFF_DETECTED & 0xFF;

    /* Sensor data: bitmask of cliff states (1 byte) */
    uint8_t cliff_mask = 0;
    for (int i = 0; i < g_ctx.sensors.cliff_count && i < 8; i++) {
        if (g_ctx.sensors.cliff_sensors[i])
            cliff_mask |= (1 << i);
    }
    msg[len++] = cliff_mask;

    /* Battery percentage */
    msg[len++] = g_ctx.sensors.battery_pct;

    pthread_mutex_unlock(&g_ctx.lock);

    return mesh_publish(MODEL_SENSOR_SERVER, ELEMENT_SENSOR, msg, len);
}

/* ---------------------------------------------------------------------------
 * Handle incoming messages from D-Bus (Mesh model callbacks)
 * --------------------------------------------------------------------------- */
static void dbus_msg_handler(DBusMessage *msg) {
    if (!msg) return;

    const char *iface = dbus_message_get_interface(msg);
    const char *member = dbus_message_get_member(msg);

    if (!iface || !member) return;

    if (strcmp(iface, "org.bluez.mesh.Element1") == 0) {
        if (strcmp(member, "Receive") == 0) {
            /* Incoming mesh message */
            DBusMessageIter iter;
            dbus_message_iter_init(msg, &iter);

            uint16_t element_idx = 0, model_id = 0;
            DBusMessageIter data_iter;

            if (dbus_message_iter_get_arg_type(&iter) == DBUS_TYPE_UINT16)
                dbus_message_iter_get_basic(&iter, &element_idx);
            dbus_message_iter_next(&iter);
            if (dbus_message_iter_get_arg_type(&iter) == DBUS_TYPE_UINT16)
                dbus_message_iter_get_basic(&iter, &model_id);
            dbus_message_iter_next(&iter);

            /* data is array of bytes */
            if (dbus_message_iter_get_arg_type(&iter) == DBUS_TYPE_ARRAY) {
                dbus_message_iter_recurse(&iter, &data_iter);

                uint8_t data[64];
                int dlen = 0;
                while (dbus_message_iter_get_arg_type(&data_iter) == DBUS_TYPE_BYTE
                       && dlen < 64) {
                    dbus_message_iter_get_basic(&data_iter, &data[dlen]);
                    dlen++;
                    dbus_message_iter_next(&data_iter);
                }

                /* Handle based on model */
                if (model_id == MODEL_GENERIC_ONOFF_SERVER && dlen >= 1) {
                    /* Generic OnOff Set: byte[0] = on/off */
                    pthread_mutex_lock(&g_ctx.lock);
                    g_ctx.leds.status_led = data[0];
                    pthread_mutex_unlock(&g_ctx.lock);
                    fprintf(stdout, "[BLEMesh] LED status: %s\n",
                            data[0] ? "ON" : "OFF");
                }
            }
        }
    }
}

/* ---------------------------------------------------------------------------
 * D-Bus message dispatch loop
 * --------------------------------------------------------------------------- */
static void mesh_rx_loop(void) {
    if (!g_ctx.dbus_conn) return;

    while (g_ctx.keep_running) {
        dbus_connection_read_write(g_ctx.dbus_conn, 0);

        DBusMessage *msg = dbus_connection_borrow_message(g_ctx.dbus_conn);
        if (msg) {
            dbus_msg_handler(msg);
            dbus_connection_return_message(g_ctx.dbus_conn, msg);
            dbus_connection_read_write(g_ctx.dbus_conn, 0);
        } else {
            usleep(50000); /* 50ms poll */
        }
    }
}

/* ---------------------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------------------- */
int ble_mesh_init(void) {
    memset(&g_ctx, 0, sizeof(g_ctx));
    pthread_mutex_init(&g_ctx.lock, NULL);
    g_ctx.keep_running = 1;
    g_ctx.sensors.battery_pct = 100;
    g_ctx.sensors.cliff_count = 4; /* 4 cliff sensors */

    if (dbus_init() != 0)
        return -1;

    if (mesh_attach() != 0) {
        fprintf(stderr, "[BLEMesh] Mesh attach failed\n");
        return -1;
    }

    /* Start RX thread */
    pthread_create(&g_ctx.rx_thread, NULL,
                   (void *(*)(void *))mesh_rx_loop, NULL);

    fprintf(stdout, "[BLEMesh] Initialized, addr=0x%04x\n", g_ctx.primary_addr);
    return 0;
}

int ble_mesh_start(void) {
    /* Publish sensor data every second */
    while (g_ctx.keep_running) {
        usleep(MESH_PUBLISH_INTERVAL_MS * 1000);
        mesh_send_sensor_status();
    }
    return 0;
}

int ble_mesh_stop(void) {
    g_ctx.keep_running = 0;
    pthread_join(g_ctx.rx_thread, NULL);
    dbus_cleanup();
    return 0;
}

/* ---------------------------------------------------------------------------
 * Sensor input from lower layers
 * --------------------------------------------------------------------------- */
void ble_mesh_report_cliff_sensor(int sensor_idx, int detected) {
    if (sensor_idx < 0 || sensor_idx >= 8) return;
    pthread_mutex_lock(&g_ctx.lock);
    g_ctx.sensors.cliff_sensors[sensor_idx] = (uint8_t)(detected ? 1 : 0);
    pthread_mutex_unlock(&g_ctx.lock);
}

void ble_mesh_report_battery(uint8_t pct) {
    pthread_mutex_lock(&g_ctx.lock);
    g_ctx.sensors.battery_pct = pct;
    pthread_mutex_unlock(&g_ctx.lock);
}

/* ---------------------------------------------------------------------------
 * LED control
 * --------------------------------------------------------------------------- */
void ble_mesh_set_led(int led_id, int on) {
    pthread_mutex_lock(&g_ctx.lock);
    if (led_id == 0)
        g_ctx.leds.status_led = (uint8_t)(on ? 1 : 0);
    else if (led_id == 1)
        g_ctx.leds.wifi_led = (uint8_t)(on ? 1 : 0);
    pthread_mutex_unlock(&g_ctx.lock);

    /* Publish LED state over mesh */
    uint8_t msg[1] = { (uint8_t)(on ? 1 : 0) };
    mesh_publish(MODEL_GENERIC_ONOFF_SERVER, ELEMENT_PRIMARY, msg, 1);
}

/* ---------------------------------------------------------------------------
 * Provisioning helpers
 * --------------------------------------------------------------------------- */
int ble_mesh_provision(void) {
    /* Initiate provisioning with an unprovisioned device */
    fprintf(stdout, "[BLEMesh] Provisioning initiated\n");
    /* In full implementation: trigger BlueZ mesh provision via D-Bus */
    return 0;
}

int ble_mesh_is_provisioned(void) {
    return g_ctx.provisioned ? 1 : 0;
}

uint16_t ble_mesh_get_primary_addr(void) {
    return g_ctx.primary_addr;
}
