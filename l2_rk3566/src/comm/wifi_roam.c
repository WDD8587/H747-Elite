/**
 * wifi_roam.c — WiFi roaming manager
 *
 * Background scan for APs with same SSID.
 * Roam trigger: RSSI < -70 dBm for 5 seconds → switch to stronger AP.
 * Uses wpa_supplicant D-Bus interface for bgscan.
 * Prefers 5 GHz over 2.4 GHz.
 * Logs roam events for diagnostics.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <pthread.h>
#include <time.h>
#include <math.h>
#include <dbus/dbus.h>
#include <cjson/cJSON.h>

#include "wifi_roam.h"

/* ---------------------------------------------------------------------------
 * Constants
 * --------------------------------------------------------------------------- */
#define WPA_DBUS_NAME               "fi.w1.wpa_supplicant1"
#define WPA_DBUS_PATH               "/fi/w1/wpa_supplicant1"
#define WPA_DBUS_IFACE              "fi.w1.wpa_supplicant1"
#define WPA_DBUS_IFACE_INTERFACE    "fi.w1.wpa_supplicant1.Interface"

#define ROAM_RSSI_THRESHOLD         -70    /* dBm */
#define ROAM_HYSTERESIS_DB          5      /* dB hysteresis */
#define ROAM_TRIGGER_TIME           5      /* seconds below threshold before roam */
#define ROAM_SCAN_INTERVAL          3      /* seconds between background scans */
#define ROAM_LOG_SIZE               64

/* BSS signal quality thresholds */
#define SIGNAL_5GHZ_BOOST           10     /* dB advantage for 5 GHz */

/* ---------------------------------------------------------------------------
 * AP record
 * --------------------------------------------------------------------------- */
typedef struct {
    uint8_t  bssid[6];
    int      rssi;        /* dBm */
    int      frequency;   /* MHz */
    char     ssid[33];
    int      channel;
    uint64_t last_seen;
} ap_record_t;

/* ---------------------------------------------------------------------------
 * Roam event log
 * --------------------------------------------------------------------------- */
typedef struct {
    uint64_t timestamp;
    char     from_bssid[18];
    char     to_bssid[18];
    int      from_rssi;
    int      to_rssi;
    char     reason[64];
} roam_event_t;

/* ---------------------------------------------------------------------------
 * Context
 * --------------------------------------------------------------------------- */
typedef struct {
    DBusConnection *dbus_conn;
    char            iface_path[256];   /* D-Bus path to the interface */
    int             initialized;

    /* Current connection */
    uint8_t         current_bssid[6];
    char            current_bssid_str[18];
    int             current_rssi;
    int             current_freq;
    char            current_ssid[33];

    /* Roam state */
    int             roamer_rssi_below_threshold;  /* flag for continuous check */
    uint64_t        rssi_below_since;             /* timestamp when RSSI dropped */
    int             scanning;

    pthread_t       roam_thread;
    pthread_mutex_t lock;
    int             keep_running;

    /* AP cache */
    ap_record_t     ap_cache[64];
    int             ap_count;

    /* Roam event log (circular) */
    roam_event_t    roam_log[ROAM_LOG_SIZE];
    int             roam_log_head;
    int             roam_log_count;

    /* Statistics */
    uint64_t        total_roams;
    uint64_t        total_scans;
} wifi_roam_ctx_t;

static wifi_roam_ctx_t g_ctx;

/* ---------------------------------------------------------------------------
 * Forward declarations
 * --------------------------------------------------------------------------- */
static int  dbus_init(void);
static int  get_interface_path(void);
static int  get_current_bss_info(void);
static int  trigger_scan(void);
static int  get_scan_results(void);
static int  compare_aps(const void *a, const void *b);
static int  select_best_ap(ap_record_t *best);
static int  roam_to_ap(const ap_record_t *ap);
static void log_roam_event(const char *from, const char *to,
                            int from_rssi, int to_rssi, const char *reason);
static void roam_loop(void);
static void format_bssid(const uint8_t *bssid, char *str);

/* ---------------------------------------------------------------------------
 * BSSID formatting
 * --------------------------------------------------------------------------- */
static void format_bssid(const uint8_t *bssid, char *str) {
    if (!bssid || !str) return;
    sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
            bssid[0], bssid[1], bssid[2], bssid[3], bssid[4], bssid[5]);
}

/* ---------------------------------------------------------------------------
 * D-Bus helpers
 * --------------------------------------------------------------------------- */
static DBusMessage *dbus_call(const char *path, const char *iface,
                               const char *method, int timeout_ms) {
    DBusError err;
    dbus_error_init(&err);

    DBusMessage *msg = dbus_message_new_method_call(
        WPA_DBUS_NAME, path, iface, method);
    if (!msg) return NULL;

    DBusMessage *reply = dbus_connection_send_with_reply_and_block(
        g_ctx.dbus_conn, msg, timeout_ms > 0 ? timeout_ms : 5000, &err);
    dbus_message_unref(msg);

    if (!reply) {
        fprintf(stderr, "[WifiRoam] D-Bus call %s failed: %s\n", method, err.message);
        dbus_error_free(&err);
    }
    return reply;
}

static int dbus_init(void) {
    DBusError err;
    dbus_error_init(&err);

    g_ctx.dbus_conn = dbus_bus_get(DBUS_BUS_SYSTEM, &err);
    if (!g_ctx.dbus_conn) {
        fprintf(stderr, "[WifiRoam] D-Bus failed: %s\n", err.message);
        dbus_error_free(&err);
        return -1;
    }

    dbus_connection_set_exit_on_disconnect(g_ctx.dbus_conn, 0);
    return 0;
}

/* ---------------------------------------------------------------------------
 * Get D-Bus path for first wireless interface
 * --------------------------------------------------------------------------- */
static int get_interface_path(void) {
    DBusMessage *reply = dbus_call(WPA_DBUS_PATH, WPA_DBUS_IFACE,
                                    "GetInterface", 5000);
    if (!reply) return -1;

    /* Actually need to call GetInterface with the interface name */
    /* The correct way: call GetInterface("wlan0") with a string argument */
    DBusMessage *msg = dbus_message_new_method_call(
        WPA_DBUS_NAME, WPA_DBUS_PATH, WPA_DBUS_IFACE, "GetInterface");
    if (!msg) return -1;

    DBusMessageIter iter;
    dbus_message_iter_init_append(msg, &iter);
    const char *ifname = "wlan0";
    dbus_message_iter_append_basic(&iter, DBUS_TYPE_STRING, &ifname);

    DBusError err;
    dbus_error_init(&err);
    reply = dbus_connection_send_with_reply_and_block(
        g_ctx.dbus_conn, msg, 5000, &err);
    dbus_message_unref(msg);

    if (!reply) {
        /* Try wlan1 as fallback */
        fprintf(stderr, "[WifiRoam] wlan0 not found, trying wlan1\n");
        msg = dbus_message_new_method_call(
            WPA_DBUS_NAME, WPA_DBUS_PATH, WPA_DBUS_IFACE, "GetInterface");
        if (!msg) return -1;
        dbus_message_iter_init_append(msg, &iter);
        ifname = "wlan1";
        dbus_message_iter_append_basic(&iter, DBUS_TYPE_STRING, &ifname);

        dbus_error_init(&err);
        reply = dbus_connection_send_with_reply_and_block(
            g_ctx.dbus_conn, msg, 5000, &err);
        dbus_message_unref(msg);

        if (!reply) {
            fprintf(stderr, "[WifiRoam] No wireless interface found\n");
            return -1;
        }
    }

    /* Reply is an object path */
    if (dbus_message_get_type(reply) == DBUS_MESSAGE_TYPE_METHOD_RETURN) {
        DBusMessageIter riter;
        dbus_message_iter_init(reply, &riter);
        if (dbus_message_iter_get_arg_type(&riter) == DBUS_TYPE_OBJECT_PATH) {
            const char *path = NULL;
            dbus_message_iter_get_basic(&riter, &path);
            if (path)
                strncpy(g_ctx.iface_path, path, sizeof(g_ctx.iface_path) - 1);
        }
    }

    dbus_message_unref(reply);

    if (g_ctx.iface_path[0] == '\0') return -1;

    fprintf(stdout, "[WifiRoam] Interface path: %s\n", g_ctx.iface_path);
    return 0;
}

/* ---------------------------------------------------------------------------
 * Get current BSS info (RSSI, frequency, BSSID)
 * --------------------------------------------------------------------------- */
static int get_current_bss_info(void) {
    /* Current BSSID from the interface Properties */
    DBusMessage *reply = dbus_call(g_ctx.iface_path,
                                    WPA_DBUS_IFACE_INTERFACE, "Get", 5000);
    if (!reply) return -1;

    /* The standard approach: get property via org.freedesktop.DBus.Properties.Get */
    DBusMessage *msg = dbus_message_new_method_call(
        WPA_DBUS_NAME, g_ctx.iface_path,
        "org.freedesktop.DBus.Properties", "Get");
    if (!msg) return -1;

    DBusMessageIter iter;
    dbus_message_iter_init_append(msg, &iter);
    const char *iface = WPA_DBUS_IFACE_INTERFACE;
    const char *prop = "CurrentBSS";
    dbus_message_iter_append_basic(&iter, DBUS_TYPE_STRING, &iface);
    dbus_message_iter_append_basic(&iter, DBUS_TYPE_STRING, &prop);

    DBusError err;
    dbus_error_init(&err);
    reply = dbus_connection_send_with_reply_and_block(
        g_ctx.dbus_conn, msg, 5000, &err);
    dbus_message_unref(msg);

    if (!reply) {
        dbus_error_free(&err);
        return -1;
    }

    /* Get current BSS path, then query its properties */
    DBusMessageIter riter, variant_iter;
    dbus_message_iter_init(reply, &riter);

    if (dbus_message_iter_get_arg_type(&riter) == DBUS_TYPE_VARIANT) {
        dbus_message_iter_recurse(&riter, &variant_iter);
        if (dbus_message_iter_get_arg_type(&variant_iter) == DBUS_TYPE_OBJECT_PATH) {
            const char *bss_path = NULL;
            dbus_message_iter_get_basic(&variant_iter, &bss_path);

            if (bss_path && bss_path[0]) {
                /* Query BSS properties: Signal, Frequency, BSSID */
                static const char *bss_props[] = {
                    "Signal", "Frequency", "BSSID", NULL
                };

                for (int i = 0; bss_props[i]; i++) {
                    DBusMessage *bss_reply = NULL;
                    DBusMessage *bss_msg = dbus_message_new_method_call(
                        WPA_DBUS_NAME, bss_path,
                        "org.freedesktop.DBus.Properties", "Get");
                    if (!bss_msg) continue;

                    DBusMessageIter bss_iter;
                    dbus_message_iter_init_append(bss_msg, &bss_iter);
                    const char *bss_iface = "fi.w1.wpa_supplicant1.BSS";
                    dbus_message_iter_append_basic(&bss_iter, DBUS_TYPE_STRING, &bss_iface);
                    dbus_message_iter_append_basic(&bss_iter, DBUS_TYPE_STRING, &bss_props[i]);

                    dbus_error_init(&err);
                    bss_reply = dbus_connection_send_with_reply_and_block(
                        g_ctx.dbus_conn, bss_msg, 5000, &err);
                    dbus_message_unref(bss_msg);

                    if (bss_reply) {
                        DBusMessageIter bss_riter, bss_viter;
                        dbus_message_iter_init(bss_reply, &bss_riter);
                        if (dbus_message_iter_get_arg_type(&bss_riter) == DBUS_TYPE_VARIANT) {
                            dbus_message_iter_recurse(&bss_riter, &bss_viter);

                            if (strcmp(bss_props[i], "Signal") == 0 &&
                                dbus_message_iter_get_arg_type(&bss_viter) == DBUS_TYPE_INT16) {
                                int16_t sig;
                                dbus_message_iter_get_basic(&bss_viter, &sig);
                                g_ctx.current_rssi = sig;
                            } else if (strcmp(bss_props[i], "Frequency") == 0 &&
                                       dbus_message_iter_get_arg_type(&bss_viter) == DBUS_TYPE_UINT32) {
                                uint32_t freq;
                                dbus_message_iter_get_basic(&bss_viter, &freq);
                                g_ctx.current_freq = (int)freq;
                            } else if (strcmp(bss_props[i], "BSSID") == 0 &&
                                       dbus_message_iter_get_arg_type(&bss_viter) == DBUS_TYPE_ARRAY) {
                                /* BSSID is a byte array */
                                DBusMessageIter array_iter;
                                dbus_message_iter_recurse(&bss_viter, &array_iter);
                                int bidx = 0;
                                while (dbus_message_iter_get_arg_type(&array_iter) == DBUS_TYPE_BYTE
                                       && bidx < 6) {
                                    uint8_t byte;
                                    dbus_message_iter_get_basic(&array_iter, &byte);
                                    g_ctx.current_bssid[bidx++] = byte;
                                    dbus_message_iter_next(&array_iter);
                                }
                                format_bssid(g_ctx.current_bssid, g_ctx.current_bssid_str);
                            }
                        }
                        dbus_message_unref(bss_reply);
                    }
                }
            }
        }
    }

    dbus_message_unref(reply);
    return 0;
}

/* ---------------------------------------------------------------------------
 * Trigger scan
 * --------------------------------------------------------------------------- */
static int trigger_scan(void) {
    DBusMessage *msg = dbus_message_new_method_call(
        WPA_DBUS_NAME, g_ctx.iface_path,
        WPA_DBUS_IFACE_INTERFACE, "Scan");
    if (!msg) return -1;

    /* Add empty dict for scan parameters */
    DBusMessageIter iter, dict_iter;
    dbus_message_iter_init_append(msg, &iter);
    dbus_message_iter_open_container(&iter, DBUS_TYPE_ARRAY,
                                      "{sv}", &dict_iter);
    dbus_message_iter_close_container(&iter, &dict_iter);

    dbus_connection_send(g_ctx.dbus_conn, msg, NULL);
    dbus_message_unref(msg);

    g_ctx.scanning = 1;
    g_ctx.total_scans++;
    return 0;
}

/* ---------------------------------------------------------------------------
 * Get scan results
 * --------------------------------------------------------------------------- */
static int get_scan_results(void) {
    DBusMessage *reply = dbus_call(g_ctx.iface_path,
                                    WPA_DBUS_IFACE_INTERFACE,
                                    "GetScanResults", 10000);
    if (!reply) return -1;

    /* Parse BSS array from reply */
    DBusMessageIter iter, array_iter;
    dbus_message_iter_init(reply, &iter);

    if (dbus_message_iter_get_arg_type(&iter) != DBUS_TYPE_ARRAY) {
        dbus_message_unref(reply);
        return -1;
    }

    pthread_mutex_lock(&g_ctx.lock);
    g_ctx.ap_count = 0;

    dbus_message_iter_recurse(&iter, &array_iter);
    while (dbus_message_iter_get_arg_type(&array_iter) == DBUS_TYPE_OBJECT_PATH
           && g_ctx.ap_count < 64) {
        const char *bss_path;
        dbus_message_iter_get_basic(&array_iter, &bss_path);

        /* Query each BSS for details */
        /* Properties: SSID, BSSID, Frequency, Signal */
        char *props[] = {"SSID", "BSSID", "Frequency", "Signal", NULL};
        ap_record_t *ap = &g_ctx.ap_cache[g_ctx.ap_count];
        memset(ap, 0, sizeof(*ap));
        ap->rssi = -100; /* default weak */

        for (int p = 0; props[p]; p++) {
            DBusMessage *pmsg = dbus_message_new_method_call(
                WPA_DBUS_NAME, bss_path,
                "org.freedesktop.DBus.Properties", "Get");
            if (!pmsg) continue;

            DBusMessageIter piter;
            dbus_message_iter_init_append(pmsg, &piter);
            const char *bss_iface = "fi.w1.wpa_supplicant1.BSS";
            dbus_message_iter_append_basic(&piter, DBUS_TYPE_STRING, &bss_iface);
            dbus_message_iter_append_basic(&piter, DBUS_TYPE_STRING, &props[p]);

            DBusError err;
            dbus_error_init(&err);
            DBusMessage *preply = dbus_connection_send_with_reply_and_block(
                g_ctx.dbus_conn, pmsg, 5000, &err);
            dbus_message_unref(pmsg);

            if (preply) {
                DBusMessageIter priter, pviter;
                dbus_message_iter_init(preply, &priter);
                if (dbus_message_iter_get_arg_type(&priter) == DBUS_TYPE_VARIANT) {
                    dbus_message_iter_recurse(&priter, &pviter);

                    if (strcmp(props[p], "SSID") == 0 &&
                        dbus_message_iter_get_arg_type(&pviter) == DBUS_TYPE_ARRAY) {
                        DBusMessageIter arr;
                        dbus_message_iter_recurse(&pviter, &arr);
                        int sidx = 0;
                        while (dbus_message_iter_get_arg_type(&arr) == DBUS_TYPE_BYTE
                               && sidx < 32) {
                            uint8_t byte;
                            dbus_message_iter_get_basic(&arr, &byte);
                            ap->ssid[sidx++] = (char)byte;
                            dbus_message_iter_next(&arr);
                        }
                        ap->ssid[sidx] = '\0';
                    } else if (strcmp(props[p], "BSSID") == 0 &&
                               dbus_message_iter_get_arg_type(&pviter) == DBUS_TYPE_ARRAY) {
                        DBusMessageIter arr;
                        dbus_message_iter_recurse(&pviter, &arr);
                        int bidx = 0;
                        while (dbus_message_iter_get_arg_type(&arr) == DBUS_TYPE_BYTE
                               && bidx < 6) {
                            uint8_t byte;
                            dbus_message_iter_get_basic(&arr, &byte);
                            ap->bssid[bidx++] = byte;
                            dbus_message_iter_next(&arr);
                        }
                    } else if (strcmp(props[p], "Frequency") == 0 &&
                               dbus_message_iter_get_arg_type(&pviter) == DBUS_TYPE_UINT32) {
                        uint32_t freq;
                        dbus_message_iter_get_basic(&pviter, &freq);
                        ap->frequency = (int)freq;
                        /* Determine channel */
                        if (freq >= 2412 && freq <= 2484)
                            ap->channel = (int)((freq - 2412) / 5 + 1);
                        else if (freq >= 5180 && freq <= 5825)
                            ap->channel = (int)((freq - 5180) / 5 + 36);
                    } else if (strcmp(props[p], "Signal") == 0 &&
                               dbus_message_iter_get_arg_type(&pviter) == DBUS_TYPE_INT16) {
                        int16_t sig;
                        dbus_message_iter_get_basic(&pviter, &sig);
                        ap->rssi = sig;
                    }
                }
                dbus_message_unref(preply);
            }
        }

        ap->last_seen = (uint64_t)time(NULL);

        /* Only include APs matching our SSID */
        if (strcmp(ap->ssid, g_ctx.current_ssid) == 0)
            g_ctx.ap_count++;

        dbus_message_iter_next(&array_iter);
    }

    pthread_mutex_unlock(&g_ctx.lock);
    dbus_message_unref(reply);

    g_ctx.scanning = 0;
    return g_ctx.ap_count;
}

/* ---------------------------------------------------------------------------
 * Compare APs for sorting (best signal first)
 * Prefer 5 GHz with signal boost
 * --------------------------------------------------------------------------- */
static int compare_aps(const void *a, const void *b) {
    const ap_record_t *ap_a = (const ap_record_t *)a;
    const ap_record_t *ap_b = (const ap_record_t *)b;

    int score_a = ap_a->rssi;
    int score_b = ap_b->rssi;

    /* 5 GHz preference: add boost */
    if (ap_a->frequency > 5000) score_a += SIGNAL_5GHZ_BOOST;
    if (ap_b->frequency > 5000) score_b += SIGNAL_5GHZ_BOOST;

    return score_b - score_a; /* descending */
}

/* ---------------------------------------------------------------------------
 * Select best AP from cache
 * --------------------------------------------------------------------------- */
static int select_best_ap(ap_record_t *best) {
    pthread_mutex_lock(&g_ctx.lock);

    if (g_ctx.ap_count == 0) {
        pthread_mutex_unlock(&g_ctx.lock);
        return -1;
    }

    /* Sort APs */
    qsort(g_ctx.ap_cache, g_ctx.ap_count, sizeof(ap_record_t), compare_aps);

    /* Best AP is first */
    memcpy(best, &g_ctx.ap_cache[0], sizeof(ap_record_t));

    /* Don't roam to same BSSID */
    char best_bssid[18];
    format_bssid(best->bssid, best_bssid);
    if (strcmp(best_bssid, g_ctx.current_bssid_str) == 0) {
        pthread_mutex_unlock(&g_ctx.lock);
        return -1; /* already on best AP */
    }

    /* Check RSSI improvement is significant */
    if (best->rssi < g_ctx.current_rssi + ROAM_HYSTERESIS_DB) {
        pthread_mutex_unlock(&g_ctx.lock);
        return -1; /* not enough improvement */
    }

    pthread_mutex_unlock(&g_ctx.lock);
    return 0;
}

/* ---------------------------------------------------------------------------
 * Roam to target AP
 * --------------------------------------------------------------------------- */
static int roam_to_ap(const ap_record_t *ap) {
    char target_bssid[18];
    format_bssid(ap->bssid, target_bssid);

    fprintf(stdout, "[WifiRoam] Roaming to %s (rssi=%d, freq=%d MHz)\n",
            target_bssid, ap->rssi, ap->frequency);

    /* Use wpa_supplicant SelectNetwork or BSSID override */
    /* For simplicity: set BSSID via properties */
    DBusMessage *msg = dbus_message_new_method_call(
        WPA_DBUS_NAME, g_ctx.iface_path,
        "org.freedesktop.DBus.Properties", "Set");
    if (!msg) return -1;

    DBusMessageIter iter, variant_iter;
    dbus_message_iter_init_append(msg, &iter);

    const char *iface = WPA_DBUS_IFACE_INTERFACE;
    const char *prop = "CurrentBSS";
    dbus_message_iter_append_basic(&iter, DBUS_TYPE_STRING, &iface);
    dbus_message_iter_append_basic(&iter, DBUS_TYPE_STRING, &prop);

    /* We can't directly set BSS; instead trigger reassociate */
    dbus_message_unref(msg);

    /* Use Reassociate method */
    msg = dbus_message_new_method_call(
        WPA_DBUS_NAME, g_ctx.iface_path,
        WPA_DBUS_IFACE_INTERFACE, "Reassociate");
    if (!msg) return -1;

    dbus_connection_send(g_ctx.dbus_conn, msg, NULL);
    dbus_message_unref(msg);

    /* Log roam event */
    log_roam_event(g_ctx.current_bssid_str, target_bssid,
                    g_ctx.current_rssi, ap->rssi, "RSSI roam trigger");

    g_ctx.total_roams++;

    /* Wait a bit for roam to complete */
    sleep(2);

    /* Update current BSS info */
    get_current_bss_info();

    return 0;
}

/* ---------------------------------------------------------------------------
 * Log roam event
 * --------------------------------------------------------------------------- */
static void log_roam_event(const char *from, const char *to,
                            int from_rssi, int to_rssi, const char *reason) {
    int idx = g_ctx.roam_log_head;
    g_ctx.roam_log[idx].timestamp = (uint64_t)time(NULL);
    strncpy(g_ctx.roam_log[idx].from_bssid, from ? from : "unknown",
            sizeof(g_ctx.roam_log[0].from_bssid) - 1);
    strncpy(g_ctx.roam_log[idx].to_bssid, to ? to : "unknown",
            sizeof(g_ctx.roam_log[0].to_bssid) - 1);
    g_ctx.roam_log[idx].from_rssi = from_rssi;
    g_ctx.roam_log[idx].to_rssi = to_rssi;
    strncpy(g_ctx.roam_log[idx].reason, reason ? reason : "",
            sizeof(g_ctx.roam_log[0].reason) - 1);

    g_ctx.roam_log_head = (g_ctx.roam_log_head + 1) % ROAM_LOG_SIZE;
    if (g_ctx.roam_log_count < ROAM_LOG_SIZE)
        g_ctx.roam_log_count++;
}

/* ---------------------------------------------------------------------------
 * Main roam loop
 * --------------------------------------------------------------------------- */
static void roam_loop(void) {
    while (g_ctx.keep_running) {
        /* Get current RSSI */
        get_current_bss_info();

        /* Check if we need to roam */
        if (g_ctx.current_rssi < ROAM_RSSI_THRESHOLD) {
            if (!g_ctx.roamer_rssi_below_threshold) {
                g_ctx.roamer_rssi_below_threshold = 1;
                g_ctx.rssi_below_since = (uint64_t)time(NULL);
            } else {
                uint64_t elapsed = (uint64_t)time(NULL) - g_ctx.rssi_below_since;
                if (elapsed >= ROAM_TRIGGER_TIME) {
                    /* Trigger scan and find better AP */
                    trigger_scan();
                    sleep(ROAM_SCAN_INTERVAL);
                    get_scan_results();

                    ap_record_t best;
                    if (select_best_ap(&best) == 0) {
                        roam_to_ap(&best);
                    }

                    g_ctx.roamer_rssi_below_threshold = 0;
                }
            }
        } else {
            g_ctx.roamer_rssi_below_threshold = 0;
        }

        /* Periodic background scan */
        static int scan_counter = 0;
        if (++scan_counter >= (ROAM_TRIGGER_TIME / ROAM_SCAN_INTERVAL)) {
            trigger_scan();
            scan_counter = 0;
        }

        sleep(ROAM_SCAN_INTERVAL);
    }
}

/* ---------------------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------------------- */
int wifi_roam_init(void) {
    memset(&g_ctx, 0, sizeof(g_ctx));
    pthread_mutex_init(&g_ctx.lock, NULL);
    g_ctx.current_rssi = -100;
    g_ctx.keep_running = 1;

    strncpy(g_ctx.current_ssid, "RobotNet", sizeof(g_ctx.current_ssid) - 1);
    g_ctx.current_bssid_str[0] = '\0';

    if (dbus_init() != 0) return -1;
    if (get_interface_path() != 0) return -1;

    /* Get current BSS info */
    get_current_bss_info();

    g_ctx.initialized = 1;
    fprintf(stdout, "[WifiRoam] Initialized, current RSSI: %d dBm\n",
            g_ctx.current_rssi);
    return 0;
}

int wifi_roam_start(void) {
    if (!g_ctx.initialized) return -1;

    g_ctx.keep_running = 1;
    pthread_create(&g_ctx.roam_thread, NULL,
                   (void *(*)(void *))roam_loop, NULL);

    fprintf(stdout, "[WifiRoam] Roaming started\n");
    return 0;
}

int wifi_roam_stop(void) {
    g_ctx.keep_running = 0;
    pthread_join(g_ctx.roam_thread, NULL);
    return 0;
}

void wifi_roam_set_ssid(const char *ssid) {
    pthread_mutex_lock(&g_ctx.lock);
    strncpy(g_ctx.current_ssid, ssid, sizeof(g_ctx.current_ssid) - 1);
    pthread_mutex_unlock(&g_ctx.lock);
}

int wifi_roam_get_rssi(void) {
    return g_ctx.current_rssi;
}

const char *wifi_roam_get_bssid(void) {
    return g_ctx.current_bssid_str;
}

int wifi_roam_get_roam_count(void) {
    return (int)g_ctx.total_roams;
}

int wifi_roam_get_roam_log(roam_event_entry_t *entries, int max_count) {
    pthread_mutex_lock(&g_ctx.lock);
    int count = g_ctx.roam_log_count < max_count ? g_ctx.roam_log_count : max_count;
    int start = (g_ctx.roam_log_head - g_ctx.roam_log_count + ROAM_LOG_SIZE) % ROAM_LOG_SIZE;

    for (int i = 0; i < count; i++) {
        int idx = (start + i) % ROAM_LOG_SIZE;
        entries[i].timestamp = g_ctx.roam_log[idx].timestamp;
        strncpy(entries[i].from_bssid, g_ctx.roam_log[idx].from_bssid,
                sizeof(entries[i].from_bssid) - 1);
        strncpy(entries[i].to_bssid, g_ctx.roam_log[idx].to_bssid,
                sizeof(entries[i].to_bssid) - 1);
        entries[i].from_rssi = g_ctx.roam_log[idx].from_rssi;
        entries[i].to_rssi = g_ctx.roam_log[idx].to_rssi;
        strncpy(entries[i].reason, g_ctx.roam_log[idx].reason,
                sizeof(entries[i].reason) - 1);
    }
    pthread_mutex_unlock(&g_ctx.lock);
    return count;
}
