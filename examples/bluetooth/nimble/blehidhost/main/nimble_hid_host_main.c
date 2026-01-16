#include "esp_log.h"
#include "esp_nimble_hci.h"
#include "nvs_flash.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_uuid.h"

static const char *TAG = "NIMBLE_HID_HOST";

static const ble_uuid16_t HID_SERVICE_UUID = BLE_UUID16_INIT(0x1812);
static const ble_uuid16_t HID_REPORT_UUID  = BLE_UUID16_INIT(0x2A4D);

static uint16_t conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint8_t own_addr_type;

static const struct ble_gap_disc_params disc_params = {
    .itvl = 0x0010,
    .window = 0x0010,
    .filter_policy = 0,
    .limited = 0,
    .passive = 0,
    .filter_duplicates = 1,
};

/*********************************************************************
 *  REPORT REFERENCE → FIND INPUT REPORT → ENABLE CCCD
 *********************************************************************/
static int
discover_cccd_for_input_cb(uint16_t conn_handle,
                           const struct ble_gatt_error *error,
                           uint16_t chr_val_handle,
                           const struct ble_gatt_dsc *dsc,
                           void *arg);

static int
report_ref_read_cb(uint16_t conn_handle,
                   const struct ble_gatt_error *error,
                   struct ble_gatt_attr *attr,
                   void *arg)
{
    if (error->status != 0 || !attr || !attr->om)
        return 0;

    uint8_t buf[2];
    if (OS_MBUF_PKTLEN(attr->om) < 2)
        return 0;

    os_mbuf_copydata(attr->om, 0, 2, buf);
    uint8_t report_id   = buf[0];
    uint8_t report_type = buf[1];

    ESP_LOGI(TAG, "ReportRef: id=%u type=%u", report_id, report_type);

    if (report_type == 0x01) {   // INPUT REPORT
        uint16_t chr_val_handle = (uint16_t)(uintptr_t)arg;

        ble_gattc_disc_all_dscs(conn_handle,
                                chr_val_handle + 1,
                                0xffff,
                                discover_cccd_for_input_cb,
                                NULL);
    }
    return 0;
}

/*********************************************************************
 *  DESCRIPTOR DISCOVERY (REPORT REF + CCCD)
 *********************************************************************/
static int
discover_dsc_cb(uint16_t conn_handle,
                const struct ble_gatt_error *error,
                uint16_t chr_val_handle,
                const struct ble_gatt_dsc *dsc,
                void *arg)
{
    if (error->status == BLE_HS_EDONE)
        return 0;

    if (error->status != 0 || !dsc)
        return 0;

    uint16_t uuid = dsc->uuid.u16.value;

    if (uuid == 0x2908) {   // Report Reference
        ble_gattc_read(conn_handle, dsc->handle,
                       report_ref_read_cb,
                       (void *)(uintptr_t)chr_val_handle);
    }

    return 0;
}

/*********************************************************************
 *  ENABLE NOTIFICATIONS ON INPUT REPORT
 *********************************************************************/
static int
discover_cccd_for_input_cb(uint16_t conn_handle,
                           const struct ble_gatt_error *error,
                           uint16_t chr_val_handle,
                           const struct ble_gatt_dsc *dsc,
                           void *arg)
{
    if (error->status == BLE_HS_EDONE)
        return 0;

    if (error->status != 0 || !dsc)
        return 0;

    if (dsc->uuid.u16.value == 0x2902) {
        uint8_t enable[2] = {0x01, 0x00};
        ESP_LOGI(TAG, "Enabling notifications on CCCD 0x%04x", dsc->handle);

        ble_gattc_write_flat(conn_handle,
                             dsc->handle,
                             enable,
                             sizeof(enable),
                             NULL,
                             NULL);
    }
    return 0;
}

/*********************************************************************
 *  CHARACTERISTIC DISCOVERY (HID REPORT)
 *********************************************************************/
static int
discover_chr_cb(uint16_t conn_handle,
                const struct ble_gatt_error *error,
                const struct ble_gatt_chr *chr,
                void *arg)
{
    uint16_t svc_end = (uint16_t)(uintptr_t)arg;

    if (error->status == BLE_HS_EDONE)
        return 0;

    if (error->status != 0)
        return 0;

    if (ble_uuid_cmp(&chr->uuid.u, &HID_REPORT_UUID.u) == 0) {
        ble_gattc_disc_all_dscs(conn_handle,
                                chr->val_handle + 1,
                                svc_end,
                                discover_dsc_cb,
                                NULL);
    }
    return 0;
}

/*********************************************************************
 *  SERVICE DISCOVERY (HID SERVICE)
 *********************************************************************/
static int
discover_svc_cb(uint16_t conn_handle,
                const struct ble_gatt_error *error,
                const struct ble_gatt_svc *svc,
                void *arg)
{
    if (error->status == BLE_HS_EDONE)
        return 0;

    if (error->status != 0)
        return 0;

    if (ble_uuid_cmp(&svc->uuid.u, &HID_SERVICE_UUID.u) == 0) {
        ESP_LOGI(TAG, "Found HID service");
        ble_gattc_disc_all_chrs(conn_handle,
                                svc->start_handle,
                                svc->end_handle,
                                discover_chr_cb,
                                (void *)(uintptr_t)svc->end_handle);
    }
    return 0;
}

/*********************************************************************
 *  NOTIFICATION HANDLER (REAL HID REPORTS ARRIVE HERE)
 *********************************************************************/
static int
gap_event_cb(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {

    case BLE_GAP_EVENT_DISC: {
        const struct ble_gap_disc_desc *d = &event->disc;

        struct ble_hs_adv_fields fields;
        if (ble_hs_adv_parse_fields(&fields, d->data, d->length_data) != 0)
            return 0;

        char name[32] = {0};
        if (fields.name && fields.name_len > 0) {
            int n = fields.name_len < 31 ? fields.name_len : 31;
            memcpy(name, fields.name, n);
        }

        ESP_LOGI(TAG, "DISC: %02X:%02X:%02X:%02X:%02X:%02X RSSI=%d name=%s",
                 d->addr.val[5], d->addr.val[4], d->addr.val[3],
                 d->addr.val[2], d->addr.val[1], d->addr.val[0],
                 d->rssi, name);

        if (strcmp(name, "Lab31 - Keyboard") == 0) {
            ESP_LOGI(TAG, "Found keyboard, connecting...");
            ble_gap_disc_cancel();

            struct ble_gap_conn_params cp = {
                .scan_itvl = 0x0010,
                .scan_window = 0x0010,
                .itvl_min = 0x0018,
                .itvl_max = 0x0028,
                .latency = 0,
                .supervision_timeout = 0x0100,
            };

            ble_gap_connect(BLE_OWN_ADDR_PUBLIC, &d->addr,
                            30000, &cp, gap_event_cb, NULL);
        }
        return 0;
    }

    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status != 0) {
            ESP_LOGW(TAG, "Connect failed: %d", event->connect.status);
            ble_gap_disc(own_addr_type, BLE_HS_FOREVER,
                         &disc_params, gap_event_cb, NULL);
            return 0;
        }

        conn_handle = event->connect.conn_handle;
        ESP_LOGI(TAG, "Connected, conn_handle=%d", conn_handle);

        ble_gap_security_initiate(conn_handle);
        return 0;

    case BLE_GAP_EVENT_ENC_CHANGE:
        ESP_LOGI(TAG, "Encryption change: %d", event->enc_change.status);
        ESP_LOGI(TAG, "Discovering HID services...");
        ble_gattc_disc_all_svcs(conn_handle, discover_svc_cb, NULL);
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "Disconnected, reason=%d", event->disconnect.reason);
        conn_handle = BLE_HS_CONN_HANDLE_NONE;
        ble_gap_disc(own_addr_type, BLE_HS_FOREVER,
                     &disc_params, gap_event_cb, NULL);
        return 0;

    case BLE_GAP_EVENT_NOTIFY_RX: {
        const struct os_mbuf *om = event->notify_rx.om;
        if (!om) return 0;

        uint16_t len = OS_MBUF_PKTLEN(om);
        ESP_LOGI(TAG, "HID REPORT len=%u", len);

        for (const struct os_mbuf *m = om; m; m = SLIST_NEXT(m, om_next))
            for (int i = 0; i < m->om_len; i++)
                printf("%02X ", m->om_data[i]);

        printf("\n");
        return 0;
    }

    default:
        return 0;
    }
}

/*********************************************************************
 *  NIMBLE INIT
 *********************************************************************/
static void on_reset(int reason) {
    ESP_LOGE(TAG, "Reset: %d", reason);
}

static void on_sync(void)
{
    ble_hs_id_infer_auto(0, &own_addr_type);
    ble_gap_disc(own_addr_type, BLE_HS_FOREVER,
                 &disc_params, gap_event_cb, NULL);
}

static void ble_hs_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void app_main(void)
{
    nvs_flash_init();
    esp_nimble_hci_and_controller_init();
    nimble_port_init();

    ble_hs_cfg.reset_cb = on_reset;
    ble_hs_cfg.sync_cb  = on_sync;

    nimble_port_freertos_init(ble_hs_task);
}
