#include "NimbleController.h"
#include <hal/nrf_rtc.h>
#define min // workaround: nimble's min/max macros conflict with libstdc++
#define max
#include <host/ble_gap.h>
#include <host/ble_hs.h>
#include <host/ble_hs_id.h>
#include <host/util/util.h>
#undef max
#undef min
#include <services/gap/ble_svc_gap.h>
#include <services/gatt/ble_svc_gatt.h>
#include "components/ble/BleController.h"
#include "components/ble/NotificationManager.h"
#include "components/datetime/DateTimeController.h"
#include "systemtask/SystemTask.h"


using namespace Watch::Controllers;

NimbleController::NimbleController(Watch::System::SystemTask& systemTask,
                                   Watch::Controllers::Ble& bleController,
        DateTime& dateTimeController,
        Watch::Controllers::NotificationManager& notificationManager,
        Controllers::Battery& batteryController,
        Watch::Drivers::SpiNorFlash& spiNorFlash) :
        systemTask{systemTask},
        bleController{bleController},
        dateTimeController{dateTimeController},
        notificationManager{notificationManager},
        spiNorFlash{spiNorFlash},
        dfuService{systemTask, bleController, spiNorFlash},
        currentTimeClient{dateTimeController},
        //anService{systemTask, notificationManager},
        alertNotificationClient{systemTask, notificationManager},
        //currentTimeService{dateTimeController},
        //musicService{systemTask},
        //batteryInformationService{batteryController},
        keyfob{batteryController},
        heartRateService{systemTask,batteryController},
        //dataSensor{batteryController},
        //appService{batteryController},
        //immediateAlertService{systemTask, notificationManager},
       serviceDiscovery({&currentTimeClient, &alertNotificationClient}) {
}

extern "C" {
int GAPEventCallback(struct ble_gap_event* event, void* arg) {
  auto nimbleController = static_cast<NimbleController*>(arg);
  return nimbleController->OnGAPEvent(event);
}

void NimbleController::Init() {
  while (!ble_hs_synced()) {
  }

  ble_svc_gap_init();
  ble_svc_gatt_init();
  //appService.Init();
  //deviceInformationService.Init();
  currentTimeClient.Init();
  //currentTimeService.Init();
  //musicService.Init();
  dfuService.Init();
  //batteryInformationService.Init();
  //immediateAlertService.Init();
  keyfob.Init();
  heartRateService.Init();

  int res;
  res = ble_hs_util_ensure_addr(0);
  ASSERT(res == 0);
  res = ble_hs_id_infer_auto(0, &addrType);
  ASSERT(res == 0);

  Watch::Controllers::Ble::BleAddress address;
  res = ble_hs_id_copy_addr(addrType, address.data(), nullptr);
  ASSERT(res == 0);
  bleController.AddressType((addrType == 0) ? Ble::AddressTypes::Public : Ble::AddressTypes::Random);
  bleController.Address(std::move(address));

  auto& bleAddr = bleController.Address();
  sprintf(deviceName, "SA%02x%02x%02x",bleAddr[2], bleAddr[1], bleAddr[0]);
  res = ble_svc_gap_device_name_set(deviceName);
  ASSERT(res == 0);

  res = ble_gatts_start();
  ASSERT(res == 0);
  }
}
void NimbleController::StopAdv() {
  ble_gatts_reset();
  ble_gap_adv_stop();  
}

void NimbleController::ble_checkevent()
{
  keyfob.ble_checkevent();
}

void NimbleController::ble_acc_checkevent()
{
  heartRateService.OnNewHeartRateValue();
}

extern "C" {
void NimbleController::StartAdvertising() {
  //if (bleController.IsConnected() || ble_gap_conn_active() || ble_gap_adv_active())
  //  return;

  struct ble_gap_adv_params adv_params;
  struct ble_hs_adv_fields fields;
    /**
     *  Set the advertisement data included in our advertisements:
     *     o Flags (indicates advertisement type and other general info).
     *     o Advertising tx power.
     *     o Device name.
     *     o 16-bit service UUIDs (alert notifications).
     */

  memset(&fields, 0, sizeof(fields));

   /* Advertise two flags:
     *     o Discoverability in forthcoming advertisement (general)
     *     o BLE-only (BR/EDR unsupported).
     */
  fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

  fields.uuids128 = &alertServiceUuid;
  fields.num_uuids128 = 1;
  fields.uuids128_is_complete = 1;

  //fields.tx_pwr_lvl_is_present = 1;
  fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

  fields.name = (uint8_t*) deviceName;
  fields.name_len = strlen(deviceName);  
  fields.name_is_complete = 1;

   /*
     * Set appearance.
     */
  //fields.appearance = ble_svc_gap_device_appearance();
  //fields.appearance_is_present = 1;

  ble_gap_adv_set_fields(&fields);

  /* Begin advertising. */
  memset(&adv_params, 0, sizeof(adv_params));
  adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
  adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

  ble_gap_adv_start(addrType, NULL, 180000, &adv_params, GAPEventCallback, this);

}

int NimbleController::OnGAPEvent(ble_gap_event* event) {
  switch (event->type) {
    case BLE_GAP_EVENT_ADV_COMPLETE:
      NRF_LOG_INFO("Advertising event : BLE_GAP_EVENT_ADV_COMPLETE");
      NRF_LOG_INFO("advertise complete; reason=%dn status=%d", event->adv_complete.reason, event->connect.status);
      break;
    case BLE_GAP_EVENT_CONNECT: {
      NRF_LOG_INFO("Advertising event : BLE_GAP_EVENT_CONNECT");

      /* A new connection was established or a connection attempt failed. */
      NRF_LOG_INFO("connection %s; status=%d ", event->connect.status == 0 ? "established" : "failed", event->connect.status);

      if (event->connect.status != 0) {
        /* Connection failed; resume advertising. */
        StartAdvertising();
        bleController.Disconnect();
      } else {
        bleController.Connect();
        systemTask.PushMessage(Watch::System::Messages::BleConnected);
        connectionHandle = event->connect.conn_handle;
        // Service discovery is deffered via systemtask
      }
    } break;
    case BLE_GAP_EVENT_DISCONNECT:
      NRF_LOG_INFO("Advertising event : BLE_GAP_EVENT_DISCONNECT");
      NRF_LOG_INFO("disconnect; reason=%d", event->disconnect.reason);
      /* Connection terminated; resume advertising. */
      currentTimeClient.Reset();
      //alertNotificationClient.Reset();
      connectionHandle = BLE_HS_CONN_HANDLE_NONE;
      systemTask.PushMessage(Watch::System::Messages::BleConnected);
      bleController.Disconnect();
      StartAdvertising();
      break;
    case BLE_GAP_EVENT_CONN_UPDATE:
      NRF_LOG_INFO("Advertising event : BLE_GAP_EVENT_CONN_UPDATE");
      /* The central has updated the connection parameters. */
      NRF_LOG_INFO("connection updated; status=%d ", event->conn_update.status);
      break;
    case BLE_GAP_EVENT_ENC_CHANGE:
      /* Encryption has been enabled or disabled for this connection. */
      NRF_LOG_INFO("encryption change event; status=%d ", event->enc_change.status);
      return 0;
    case BLE_GAP_EVENT_SUBSCRIBE:
      NRF_LOG_INFO("subscribe event; conn_handle=%d attr_handle=%d "
                   "reason=%d prevn=%d curn=%d previ=%d curi=???\n",
                   event->subscribe.conn_handle,
                   event->subscribe.attr_handle,
                   event->subscribe.reason,
                   event->subscribe.prev_notify,
                   event->subscribe.cur_notify,
                   event->subscribe.prev_indicate);
      return 0;
    case BLE_GAP_EVENT_MTU:
      NRF_LOG_INFO("mtu update event; conn_handle=%d cid=%d mtu=%d\n", event->mtu.conn_handle, event->mtu.channel_id, event->mtu.value);
      return 0;

    case BLE_GAP_EVENT_REPEAT_PAIRING: {
      /* We already have a bond with the peer, but it is attempting to
       * establish a new secure link.  This app sacrifices security for
       * convenience: just throw away the old bond and accept the new link.
       */

      /* Delete the old bond. */
      struct ble_gap_conn_desc desc;
      ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc);
      ble_store_util_delete_peer(&desc.peer_id_addr);

      /* Return BLE_GAP_REPEAT_PAIRING_RETRY to indicate that the host should
       * continue with the pairing operation.
       */
    }
      return BLE_GAP_REPEAT_PAIRING_RETRY;

    case BLE_GAP_EVENT_NOTIFY_RX: {
      /* Peer sent us a notification or indication. */
      size_t notifSize = OS_MBUF_PKTLEN(event->notify_rx.om);

      NRF_LOG_INFO("received %s; conn_handle=%d attr_handle=%d "
                   "attr_len=%d",
                   event->notify_rx.indication ? "indication" : "notification",
                   event->notify_rx.conn_handle,
                   event->notify_rx.attr_handle,
                   notifSize);

      //alertNotificationClient.OnNotification(event);
      return 0;
    }
      /* Attribute data is contained in event->notify_rx.attr_data. */

    default:
      //      NRF_LOG_INFO("Advertising event : %d", event->type);
      break;
  }
  return 0;
}
}
void NimbleController::StartDiscovery() {
  serviceDiscovery.StartDiscovery(connectionHandle);
}

uint16_t NimbleController::connHandle() {
  return connectionHandle;
}
