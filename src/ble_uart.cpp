#include <Arduino.h>
#include <NimBLEDevice.h>
#include <esp_gap_ble_api.h>
#include <esp_gattc_api.h>
#include <esp_gatt_defs.h>
#include <esp_bt_main.h>
#include <esp_gatt_common_api.h>
#include "config.h"
#include "ble_uart.h"


#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID

#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

NimBLEServer* pBLEServer                = NULL;
NimBLEService* pService                 = NULL;
NimBLECharacteristic* pTxCharacteristic = NULL;
NimBLECharacteristic* pRxCharacteristic = NULL;

static uint8_t ble_uart_nmea_checksum(const char *szNMEA);

void ble_uart_init() {
	NimBLEDevice::init("BLE-Vario-esp32");
	NimBLEDevice::setMTU(46);
	// default power level is +3dB, max +9dB
	//NimBLEDevice::setPower(ESP_PWR_LVL_N3); // -3dB
	NimBLEDevice::setPower(ESP_PWR_LVL_N0); // 0dB
    //NimBLEDevice::setPower(ESP_PWR_LVL_P6);  // +6db 

	NimBLEDevice::setSecurityAuth(true, true, true);
	NimBLEDevice::setSecurityPasskey(123456);
	NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_ONLY);

	pBLEServer = NimBLEDevice::createServer();

	pService          = pBLEServer->createService(SERVICE_UUID);
	pTxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TX, NIMBLE_PROPERTY::NOTIFY);

	pRxCharacteristic = pService->createCharacteristic(
		CHARACTERISTIC_UUID_RX,
		NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_ENC | NIMBLE_PROPERTY::WRITE_AUTHEN);

	pService->start();
	pBLEServer->getAdvertising()->start();
	}


static uint8_t ble_uart_nmea_checksum(const char *szNMEA){
	const char* sz = &szNMEA[1]; // skip leading '$'
	uint8_t cksum = 0;
	while ((*sz) != 0 && (*sz != '*')) {
		cksum ^= (uint8_t) *sz;
		sz++;
		}
	return cksum;
	}

   
void ble_uart_transmit_LK8EX1(int32_t altm, int32_t cps, float batVoltage) {
	char szmsg[40];
#ifdef BATTERY_VOLTAGE_MONITOR
	sprintf(szmsg, "$LK8EX1,999999,%.d,%d,99,%.1f*", altm, cps, batVoltage);
#else
	sprintf(szmsg, "$LK8EX1,999999,%.d,%d,99,999*", altm, cps);
#endif
	uint8_t cksum = ble_uart_nmea_checksum(szmsg);
	char szcksum[5];
	sprintf(szcksum,"%02X\r\n", cksum);
	strcat(szmsg, szcksum);
#ifdef BLE_DEBUG	
    dbg_printf(("%s", szmsg)); 
#endif
	pTxCharacteristic->setValue((const uint8_t*)szmsg, strlen(szmsg));
	pTxCharacteristic->notify();   
	}

	void ble_uart_transmit_XCTRC(float altm, float mps, float pressurePa, float batVoltage) {
	/*	* Native XTRC sentences
		* $XCTRC,2015,1, 5,16,34,33,36,46.947508,7.453117, 540.32,12.35,270.4,2.78,,,,964.93,98*67
		*
		* $XCTRC,year,month,day,hour,minute,second,centisecond,latitude,longitude,altitude,speedoverground,
		*      course,climbrate,res,res,res,rawpressure,batteryindication*checksum
		*/
		char szmsg[100];
	#ifdef BATTERY_VOLTAGE_MONITOR
		sprintf(szmsg, "$XCTRC,0,0,0,0,0,0,0,0.0,0.0,%.2f,0,0,%.2f,,,,%.2f,%d*", altm, mps, raw_press, batVoltage);
	#else
		//sprintf(szmsg, "$XCTRC,0,0,0,0,0,0,0,0.000000,0.000000,%.2f,0.00,0.0,%.2f,,,,%.2f,99*", altm, mps, pressurePa/100.);
		//sprintf(szmsg, "$XCTRC,,,,,,,,,,,,,%.2f,,,,%.2f,99*", mps, pressurePa/100.);
		sprintf(szmsg, "$XCTRC,,,,,,,,,,,,,%.2f,,,,%.2f,99*", mps, pressurePa/100.);
	#endif
		uint8_t cksum = ble_uart_nmea_checksum(szmsg);
		char szcksum[5];
		sprintf(szcksum,"%02X\r\n", cksum);
		strcat(szmsg, szcksum);
		//sprintf(szmsg, "$XCTRC,2015,1,5,16,34,33,36,46.947508,7.453117,540.32,12.35,270.4,2.78,,,,964.93,98*67\r\n");
	#ifdef BLE_DEBUG	
		dbg_printf(("%s", szmsg)); 
	#endif
		pTxCharacteristic->setValue((const uint8_t*)szmsg, strlen(szmsg));
		pTxCharacteristic->notify();   
	}