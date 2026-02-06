#ifdef HAS_BLE_WRITER
#include <Arduino.h>
#include <MD5Builder.h>

#include <NimBLEDevice.h>
#include "ble_filter.h"
#include "newproto.h"

#define INTERVAL_BLE_SCANNING_SECONDS 60
#define INTERVAL_HANDLE_PENDING_SECONDS 10
#define BUFFER_MAX_SIZE_COMPRESSING 135000

#define CONNECT_RETRIES 5

#define BLE_MAIN_STATE_IDLE 0
#define BLE_MAIN_STATE_PREPARE 1
#define BLE_MAIN_STATE_CONNECT 2
#define BLE_MAIN_STATE_UPLOAD 3
//#define BLE_MAIN_STATE_ATC_BLE_OEPL_UPLOAD 4

int ble_main_state = BLE_MAIN_STATE_IDLE;
uint32_t last_ble_scan = 0;

#define BLE_UPLOAD_STATE_INIT 0
#define BLE_UPLOAD_STATE_SIZE 1
#define BLE_UPLOAD_STATE_START 2
#define BLE_UPLOAD_STATE_UPLOAD 5
int BLE_upload_state = BLE_UPLOAD_STATE_INIT;

#define BLE_CMD_ACK_CMD 99
#define BLE_CMD_AVAILDATA 100
#define BLE_CMD_BLK_DATA 101
#define BLE_CMD_ERR_BLKPRT 196
#define BLE_CMD_ACK_BLKPRT 197
#define BLE_CMD_REQ 198
#define BLE_CMD_ACK 199
#define BLE_CMD_ACK_IS_SHOWN 200
#define BLE_CMD_ACK_FW_UPDATED 201

struct AvailDataInfo BLEavaildatainfo = {0};
struct blockRequest BLEblkRequst = {0};

bool BLE_connected = false;
volatile bool BLE_new_notify = false;

// Use explicit NimBLE types to avoid collisions
static NimBLEUUID ATC_BLE_OEPL_ServiceUUID((uint16_t)0x1337);
static NimBLEUUID ATC_BLE_OEPL_CtrlUUID((uint16_t)0x1337);
static NimBLEUUID gicServiceUUID((uint16_t)0xfef0);
static NimBLEUUID gicCtrlUUID((uint16_t)0xfef1);
static NimBLEUUID gicImgUUID((uint16_t)0xfef2);

// Use NimBLE specific pointers
NimBLERemoteCharacteristic* ctrlChar = nullptr;
NimBLERemoteCharacteristic* imgChar = nullptr;
NimBLEClient* pClient = nullptr;

uint8_t BLE_notify_buffer[256] = {0};

uint32_t BLE_err_counter = 0;
uint32_t BLE_curr_part = 0;
uint32_t BLE_max_block_parts = 0;
uint8_t BLE_mini_buff[256];

uint32_t BLE_last_notify = 0;
uint32_t BLE_last_pending_check = 0;
uint8_t BLE_curr_address[8] = {0};

uint32_t BLE_compressed_len = 0;
uint8_t* BLE_image_buffer = nullptr;

// Signature updated for NimBLE: pData is now a pointer to the internal buffer
static void notifyCallback(
    NimBLERemoteCharacteristic* pBLERemoteCharacteristic,
    uint8_t* pData,
    size_t length,
    bool isNotify) {
    if (length > 255) length = 255;
    Serial.print("Notify callback for characteristic ");
    Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
    Serial.print(" of data length ");
    Serial.println(length);
    Serial.print("data: ");
    for (int i = 0; i < length; i++) {
        Serial.printf("%02X", pData[i]);
        BLE_notify_buffer[1 + i] = pData[i];
    }
    BLE_notify_buffer[0] = (uint8_t)length;
    Serial.println();
    BLE_new_notify = true;
}

class MyClientCallback : public NimBLEClientCallbacks {
    void onConnect(NimBLEClient* pclient) override {
        Serial.println("BLE onConnect");
        BLE_connected = true;
    }
    void onDisconnect(NimBLEClient* pclient, int reason) override {
        Serial.printf("BLE onDisconnect, reason=%d\n", reason);
        imgChar = nullptr;
        ctrlChar = nullptr;
        BLE_connected = false;
        ble_main_state = BLE_MAIN_STATE_IDLE;
    }
};

enum BLE_CONNECTION_TYPE {
    BLE_TYPE_GICISKY = 0,
    BLE_TYPE_ATC_BLE_OEPL
};

// NimBLE Scan Callback signature uses pointers
class MyAdvertisedDeviceCallbacks : public NimBLEScanCallbacks {
    void onResult(const NimBLEAdvertisedDevice* advertisedDevice) override {
        BLE_filter_add_device(*const_cast<NimBLEAdvertisedDevice*>(advertisedDevice));
    }
};

bool BLE_connect(uint8_t* addr, BLE_CONNECTION_TYPE conn_type) {
    // NOTE: Order of characteristics is important! - notify must be last
    // NimBLEAddress takes 2 arguments: the array and the type
    // Manually reverse the address to match NimBLE's expectation
    uint8_t flippedAddr[6];
    for(int i = 0; i < 6; i++) {
        flippedAddr[i] = addr[5 - i];
    }
    NimBLEAddress targetAddr(flippedAddr, BLE_ADDR_PUBLIC);
    Serial.printf("BLE Connecting to: %02X:%02X:%02X:%02X:%02X:%02X\r\n", addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);

    if (!pClient) {
        Serial.printf("BLE Created client\r\n");
        pClient = NimBLEDevice::createClient();
        pClient->setConnectionParams(36, 60, 0, 200);    //  (min/max interval, latency, timeout - 2s)
        pClient->setClientCallbacks(new MyClientCallback(), false);
    }

    if (pClient->isConnected()) {
        pClient->disconnect();
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }

    if (!pClient->connect(targetAddr)) {
        Serial.printf("BLE connection failed\r\n");
        return false;
    }

    vTaskDelay(150 / portTICK_PERIOD_MS); // let radio + link stabilize
    vTaskDelay(150 / portTICK_PERIOD_MS); // let GATT settle

    // Discover all services and characteristics in one go
    // This prevents multiple slow round-trips for getService() and getCharacteristic()
    if (!pClient->discoverAttributes()) {
        Serial.printf("BLE Attribute discovery failed\r\n");
        pClient->disconnect();
        return false;
    }

    NimBLERemoteService* pService = pClient->getService(
        (conn_type == BLE_TYPE_GICISKY) ? gicServiceUUID : ATC_BLE_OEPL_ServiceUUID
    );

    if (!pService) {
        Serial.printf("BLE Service failed\r\n");
        pClient->disconnect();
        return false;
    }

    if (conn_type == BLE_TYPE_GICISKY) {
        imgChar = pService->getCharacteristic(gicImgUUID);
        if (imgChar == nullptr) {
            Serial.printf("BLE IMG Char failed\r\n");
            pClient->disconnect();
            return false;
        }
    }

    ctrlChar = pService->getCharacteristic(
        (conn_type == BLE_TYPE_GICISKY) ? gicCtrlUUID : ATC_BLE_OEPL_CtrlUUID
    );

    if (ctrlChar == nullptr) {
        Serial.printf("BLE ctrl Char failed\r\n");
        pClient->disconnect();
        return false;
    }

    if (ctrlChar && ctrlChar->canNotify()) {
        if (!ctrlChar->subscribe(true, notifyCallback, true)) {
            Serial.printf("BLE Notify failed\r\n");
            pClient->disconnect();
            return false;
        }
    }

    Serial.printf("BLE Connected fully to: %02X:%02X:%02X:%02X:%02X:%02X\r\n", addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);
    vTaskDelay(100 / portTICK_PERIOD_MS); // small delay to allow Notify settings to take effect
    return true;
}

#define BLOCK_DATA_SIZE_BLE 4096
#define BLOCK_PART_DATA_SIZE_BLE 230
uint8_t tempBlockBuffer[BLOCK_DATA_SIZE_BLE + 4];
uint8_t tempPacketBuffer[2 + 3 + BLOCK_PART_DATA_SIZE_BLE];
void ATC_BLE_OEPL_PrepareBlk(uint8_t indexBlockId) {
    if (BLE_image_buffer == nullptr) {
        return;
    }
    uint32_t bufferPosition = (BLOCK_DATA_SIZE_BLE * indexBlockId);
    uint32_t lenNow = BLOCK_DATA_SIZE_BLE;
    uint16_t crcCalc = 0;
    if ((BLE_compressed_len - bufferPosition) < BLOCK_DATA_SIZE_BLE)
        lenNow = (BLE_compressed_len - bufferPosition);
    tempBlockBuffer[0] = lenNow & 0xff;
    tempBlockBuffer[1] = (lenNow >> 8) & 0xff;
    for (uint16_t c = 0; c < lenNow; c++) {
        tempBlockBuffer[4 + c] = BLE_image_buffer[c + bufferPosition];
        crcCalc += tempBlockBuffer[4 + c];
    }
    tempBlockBuffer[2] = crcCalc & 0xff;
    tempBlockBuffer[3] = (crcCalc >> 8) & 0xff;
    BLE_max_block_parts = (4 + lenNow) / BLOCK_PART_DATA_SIZE_BLE;
    if ((4 + lenNow) % BLOCK_PART_DATA_SIZE_BLE)
        BLE_max_block_parts++;
    Serial.println("Preparing block: " + String(indexBlockId) + " BuffPos: " + String(bufferPosition) + " LenNow: " + String(lenNow) + " MaxBLEparts: " + String(BLE_max_block_parts));
    BLE_curr_part = 0;
}

void ATC_BLE_OEPL_SendPart(uint8_t indexBlockId, uint8_t indexPkt) {
    uint8_t crcCalc = indexBlockId + indexPkt;
    for (uint16_t c = 0; c < BLOCK_PART_DATA_SIZE_BLE; c++) {
        tempPacketBuffer[5 + c] = tempBlockBuffer[c + (BLOCK_PART_DATA_SIZE_BLE * indexPkt)];
        crcCalc += tempPacketBuffer[5 + c];
    }
    tempPacketBuffer[0] = 0x00;
    tempPacketBuffer[1] = 0x65;
    tempPacketBuffer[2] = crcCalc;
    tempPacketBuffer[3] = indexBlockId;
    tempPacketBuffer[4] = indexPkt;
    Serial.println("BLE Sending packet Len " + String(sizeof(tempPacketBuffer)));
    ctrlChar->writeValue(tempPacketBuffer, sizeof(tempPacketBuffer), true);
}

void FreeBuffer(bool done=false, bool reset=false) {
    // free buffer, and clean up, report success or failure
    free(BLE_image_buffer);
    BLE_image_buffer = nullptr;
    if (++BLE_err_counter >= CONNECT_RETRIES || done || reset) {  // 5 Retries for a BLE Connection
        struct espXferComplete reportStruct;
        memcpy((uint8_t*)&reportStruct.src, BLE_curr_address, 8);
        if ((!done && reset) || BLE_err_counter >= CONNECT_RETRIES) {
            Serial.printf("BLE timeout, try: %i\r\n", BLE_err_counter);
            processXferTimeout(&reportStruct, true);
        } else {
            processXferComplete(&reportStruct, true);
        }
        BLE_err_counter = 0;
        Serial.printf("BLE reset error count: %i\r\n", BLE_err_counter);
        BLE_curr_part = 0;
        BLE_max_block_parts = 0;
    }
    if (pClient != nullptr && pClient->isConnected()) {
        pClient->disconnect();
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
    if (reset) {
        ble_main_state = BLE_MAIN_STATE_IDLE;
        BLE_last_pending_check = millis();
        //last_ble_scan = 0;
    }
    last_ble_scan = 0;
}

bool CreateBuffer() {
    // Here we create the compressed buffer
    if (BLE_image_buffer == nullptr) {
        BLE_image_buffer = (uint8_t*)malloc(BUFFER_MAX_SIZE_COMPRESSING);
        if (BLE_image_buffer == nullptr) {
            Serial.println("BLE Could not create buffer!");
            BLE_compressed_len = 0;
            return false;
        }
    }
    return true;
}

bool PrepareAndConnect(BLE_CONNECTION_TYPE conn_type) {
    // Create buffer, fill compressed buffer and connect
    // if the BLE_curr_address changes, reset the error count
    static uint8_t last_addr[6];
    if (memcmp(last_addr, BLE_curr_address, 6) != 0) {
        BLE_err_counter = 0;
        Serial.printf("BLE new tag: reset error count: %i\r\n", BLE_err_counter);
        memcpy(last_addr, BLE_curr_address, 6);
    }
    uint8_t dataType = 0x00;
    uint8_t dataTypeArgument = 0x00;
    uint16_t nextCheckin = 0x00;
    if (CreateBuffer()) {
        if (conn_type == BLE_TYPE_ATC_BLE_OEPL) {
            BLE_compressed_len = get_ATC_BLE_OEPL_image(BLE_curr_address, BLE_image_buffer, BUFFER_MAX_SIZE_COMPRESSING, &dataType, &dataTypeArgument, &nextCheckin);
            if (BLE_compressed_len) {
                uint8_t md5bytes[16];
                MD5Builder md5;
                md5.begin();
                md5.add(BLE_image_buffer, BLE_compressed_len);
                md5.calculate();
                md5.getBytes(md5bytes);

                BLEavaildatainfo.dataType = dataType;
                BLEavaildatainfo.dataVer = *((uint64_t*)md5bytes);
                BLEavaildatainfo.dataSize = BLE_compressed_len;
                BLEavaildatainfo.dataTypeArgument = dataTypeArgument;
                BLEavaildatainfo.nextCheckIn = nextCheckin;
                BLEavaildatainfo.checksum = 0;
                for (uint16_t c = 1; c < sizeof(struct AvailDataInfo); c++) {
                    BLEavaildatainfo.checksum += (uint8_t)((uint8_t*)&BLEavaildatainfo)[c];
                }
            }
        } else {
            BLE_compressed_len = compress_image(BLE_curr_address, BLE_image_buffer, BUFFER_MAX_SIZE_COMPRESSING);
        }
        Serial.printf("BLE Compressed Length: %i\r\n", BLE_compressed_len);
        Serial.printf("BLE connection try: %i\r\n", BLE_err_counter);
        // then we connect to BLE to send the compressed data
        if (BLE_compressed_len && BLE_connect(BLE_curr_address, conn_type)) {
            memset(BLE_notify_buffer, 0x00, sizeof(BLE_notify_buffer));
            ble_main_state = BLE_MAIN_STATE_UPLOAD;
            BLE_upload_state = BLE_UPLOAD_STATE_INIT;
            BLE_new_notify = true;  // trigger the upload here
            return true;
        }
    } 
    FreeBuffer();
    return false;
}

void Upload(BLE_CONNECTION_TYPE conn_type) {
    // We call this each time we get a Notify from the tag, and progress through the upload stages
    if (BLE_curr_part == 0)
        Serial.println("BLE Starting Upload");
    BLE_new_notify = false;
    BLE_last_notify = millis();
    switch (conn_type) {
        default:
        case BLE_TYPE_GICISKY:
            BLE_upload_state = BLE_notify_buffer[1];
            switch (BLE_upload_state) {
                default:
                case BLE_UPLOAD_STATE_INIT:
                    BLE_mini_buff[0] = 0x01;
                    ctrlChar->writeValue((const uint8_t*)BLE_mini_buff, 1);
                    break;
                case BLE_UPLOAD_STATE_SIZE:
                    BLE_mini_buff[0] = 0x02;
                    BLE_mini_buff[1] = BLE_compressed_len & 0xff;
                    BLE_mini_buff[2] = (BLE_compressed_len >> 8) & 0xff;
                    BLE_mini_buff[3] = (BLE_compressed_len >> 16) & 0xff;
                    BLE_mini_buff[4] = (BLE_compressed_len >> 24) & 0xff;
                    BLE_mini_buff[5] = 0x00;
                    ctrlChar->writeValue((const uint8_t*)BLE_mini_buff, 6);
                    break;
                case BLE_UPLOAD_STATE_START:
                    BLE_mini_buff[0] = 0x03;
                    ctrlChar->writeValue((const uint8_t*)BLE_mini_buff, 1);
                    break;
                case BLE_UPLOAD_STATE_UPLOAD:
                    if (BLE_notify_buffer[2] == 0x08) {
                        // Done and the image is refreshing now
                        FreeBuffer(true, true);
                    } else {
                        uint32_t req_curr_part = (BLE_notify_buffer[6] << 24) | (BLE_notify_buffer[5] << 24) | (BLE_notify_buffer[4] << 24) | BLE_notify_buffer[3];
                        if (req_curr_part != BLE_curr_part) {
                            Serial.printf("Something went wrong, expected req part: %i but got: %i we better abort here.\r\n", req_curr_part, BLE_curr_part);
                            FreeBuffer(false, true);
                        }
                        uint32_t curr_len = 240;
                        if (BLE_compressed_len - (BLE_curr_part * 240) < 240)
                            curr_len = BLE_compressed_len - (BLE_curr_part * 240);
                        BLE_mini_buff[0] = BLE_curr_part & 0xff;
                        BLE_mini_buff[1] = (BLE_curr_part >> 8) & 0xff;
                        BLE_mini_buff[2] = (BLE_curr_part >> 16) & 0xff;
                        BLE_mini_buff[3] = (BLE_curr_part >> 24) & 0xff;
                        memcpy((uint8_t*)&BLE_mini_buff[4], (uint8_t*)&BLE_image_buffer[BLE_curr_part * 240], curr_len);
                        imgChar->writeValue((const uint8_t*)BLE_mini_buff, curr_len + 4);
                        Serial.printf("BLE sending part: %i\r\n", BLE_curr_part);
                        BLE_curr_part++;
                    }
                    break;
            }
            break;
        case BLE_TYPE_ATC_BLE_OEPL:
            switch (BLE_upload_state) {
                default:
                case BLE_UPLOAD_STATE_INIT:
                    BLE_mini_buff[0] = 0x00;
                    BLE_mini_buff[1] = 0x64;
                    memcpy((uint8_t*)&BLE_mini_buff[2], &BLEavaildatainfo, sizeof(struct AvailDataInfo));
                    ctrlChar->writeValue((const uint8_t*)BLE_mini_buff, sizeof(struct AvailDataInfo) + 2);
                    BLE_upload_state = BLE_UPLOAD_STATE_UPLOAD;
                    break;
                case BLE_UPLOAD_STATE_UPLOAD:
                    uint8_t notifyLen = BLE_notify_buffer[0];
                    uint16_t notifyCMD = (BLE_notify_buffer[1] << 8) | BLE_notify_buffer[2];
                    Serial.println("BLE CMD " + String(notifyCMD));
                    switch (notifyCMD) {
                        case BLE_CMD_REQ:
                            if (notifyLen == (sizeof(struct blockRequest) + 2)) {
                                Serial.println("We got a request for a BLK");
                                memcpy(&BLEblkRequst, &BLE_notify_buffer[3], sizeof(struct blockRequest));
                                BLE_curr_part = 0;
                                ATC_BLE_OEPL_PrepareBlk(BLEblkRequst.blockId);
                                ATC_BLE_OEPL_SendPart(BLEblkRequst.blockId, BLE_curr_part);
                            }
                            break;
                        case BLE_CMD_ACK_BLKPRT:
                            BLE_curr_part++;
                            BLE_err_counter = 0;
                        case BLE_CMD_ERR_BLKPRT:
                            if (BLE_curr_part <= BLE_max_block_parts && BLE_err_counter++ < 15) {
                                ATC_BLE_OEPL_SendPart(BLEblkRequst.blockId, BLE_curr_part);
                                break;
                            }  // FALLTROUGH!!! We cancel the upload if we land here since we dont have so many parts of a block!
                        case BLE_CMD_ACK:
                        case BLE_CMD_ACK_IS_SHOWN:
                        case BLE_CMD_ACK_FW_UPDATED:
                            FreeBuffer(true, true);
                            break;
                    }
                    break;
            }
            break;
    }
}

void BLETask(void* parameter) {
    vTaskDelay(pdMS_TO_TICKS(5000));
    Serial.println("BLE task started");
    NimBLEDevice::init("");
    NimBLEDevice::setMTU(255);
    //NimBLEDevice::setPower(ESP_PWR_LVL_P9); // Ensure maximum signal strength
    // Set callbacks ONCE here for 2.x
    NimBLEScan* pScan = NimBLEDevice::getScan();
    pScan->setScanCallbacks(new MyAdvertisedDeviceCallbacks(), true);
    pScan->setMaxResults(0);        // 0 = Do not store results, use callbacks only (unlimited)
    pScan->setInterval(1349);
    pScan->setWindow(449);
    pScan->setActiveScan(true);
    pScan->setDuplicateFilter(1);   // filter duplicates until scan is restarted
    BLE_CONNECTION_TYPE conn_type = BLE_TYPE_GICISKY;
    while (1) {
        switch (ble_main_state) {
            default:
            case BLE_MAIN_STATE_IDLE:
                if (millis() - last_ble_scan > (INTERVAL_BLE_SCANNING_SECONDS * 1000)) {
                    last_ble_scan = millis();
                    Serial.println("Restarting the BLE Scan");
                    pScan->start(0, false, true);   // non-blocking scan
                }
                if (millis() - BLE_last_pending_check >= (INTERVAL_HANDLE_PENDING_SECONDS * 1000)) {
                    if (BLE_is_image_pending(BLE_curr_address)) {
                        Serial.println("Stopping scan for upload...");
                        // Stop the background scan
                        pScan->stop();
                        Serial.println("BLE Image is pending but we wait a bit");
                        vTaskDelay(500 / portTICK_PERIOD_MS);                             // We better wait here, since the pending image needs to be created first
                        conn_type = BLE_curr_address[7] == 0x13 && BLE_curr_address[6] == 0x37 ? BLE_TYPE_ATC_BLE_OEPL : BLE_TYPE_GICISKY; // what type of OEPL display is this
                        PrepareAndConnect(conn_type);
                        BLE_last_pending_check = millis();
                    }
                }
                break;
            case BLE_MAIN_STATE_UPLOAD:
                if (BLE_connected && BLE_new_notify) {
                    Upload(conn_type);  // call each time we get a new Notify message from the tag
                } else if (millis() - BLE_last_notify > 30000) {  // Something odd, better reset connection!
                    Serial.println("BLE err going back to IDLE");
                    FreeBuffer(false, true);
                }
                break;
        }
        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
}

#endif
