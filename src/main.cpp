#include <Arduino.h>
#include <Wire.h>
#include <FS.h>
#include <LittleFS.h>
#include "config.h"
#include "spi_functions.h"
#include "util.h"
#include "imu.h"
#include "mpu9250.h"
#include "ms5611.h"
#include "kalmanfilter4d.h"
#include "audio.h"
#include "vaudio.h"
#include "adc.h"
#include "nvd.h"
#include "ringbuf.h"
#include "wifi_cfg.h"
#include "ui.h"
#include "ble_uart.h"

#ifdef EPAPER_DISPLAY
#include "epaper_display.h"
#endif

const char* FwRevision = "0.99";

MPU9250	Imu;
MS5611	Baro;

boolean	bWebConfigure = false;

volatile SemaphoreHandle_t DrdySemaphore;
volatile float AltitudeCm;
volatile int ClimbrateCps;
volatile float PressurePa;
int LEDState;

static void IRAM_ATTR drdy_interrupt_handler();
static void vario_task(void * pvParameter);
static void wifi_config_task(void * pvParameter);
static void ble_task(void* pvParameter);
#ifdef EPAPER_DISPLAY
static void epaper_task(void * pvParameter);
volatile int EpaperBackgroundUpdate = 0;
boolean boot_complete = false;
#endif

// PCCA button (GPIO9) has an external 10K pullup resistor to VCC
// This button has  different functions : program, configure, calibrate & audio toggle 
// 1. Program
//    Power on the unit with PCCA button pressed. Or with power on, keep 
//    PCCA pressed and momentarily press the reset button.
//    This will put the ESP32-C3 into programming mode, and you can flash 
//    the application code.
// 2. WiFi Configuration
//    After normal power on, immediately press PCCA and keep it pressed. 
//    Wait until you hear a low tone, then release. The unit will now be in WiFi configuration
//    configuration mode. 
// 3. Calibrate Accelerometer
//    After normal power on, wait until you hear the battery voltage feedback beeps and
//    then the countdown to gyroscope calibration. If you press the PCCA button
//    during the gyro calibration countdown, the unit will start accelerometer calibration first. 
//    Accelerometer re-calibration is required if the acceleration calibration values in 
//    flash were never written, or if the entire flash has been erased.
// 4. Audio Mute/Unmute  
//    When the vario is in normal operation, pressing PCCA will toggle the
//    vario audio feedback.


// handles data ready interrupt from MPU9250 (every 2ms)
static void IRAM_ATTR drdy_interrupt_handler() {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(DrdySemaphore, &xHigherPriorityTaskWoken);
    if( xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR(); // this wakes up vario_task immediately instead of on next FreeRTOS tick
		}	
	}	


void setup() {
	pinMode(pinPCCA, INPUT_PULLUP); //  Program/Configure/Calibrate/Audio Mute Button
#ifdef HAS_BUTTON_2
	pinMode(pinBut2, INPUT_PULLUP); // Button 2
#endif
	pinMode(pinLED, OUTPUT_OPEN_DRAIN); // power/bluetooth LED, active low
	LED_OFF();
#ifdef AUDIO_ENABLE_PIN
	pinMode(pinAudioEn, OUTPUT_OPEN_DRAIN); // output enable for 74HC240, active low
	digitalWrite(pinAudioEn, HIGH);
#endif

	wifi_off(); // turn off radio to save power

#ifdef TOP_DEBUG
	// Serial print redirected to USB CDC port (on Ubuntu, shows up as /dev/ttyACMx)
	Serial.begin(115200);
#endif

	LED_ON();
#ifdef AUDIO_ENABLE_PIN
	digitalWrite(pinAudioEn, LOW);
#endif
	spi_init();

#ifdef EPAPER_DISPLAY
	display_init();
#endif
#ifdef EPAPER_DISPLAY
	display_add_boot_message("Display initiated");
#endif
	dbg_printf(("\n\nESP32-WROOM BLUETOOTH VARIO compiled on %s at %s\n", __DATE__, __TIME__));
	dbg_printf(("Firmware Revision %s\n", FwRevision));
#ifdef EPAPER_DISPLAY
	display_add_boot_message("Firmware rev. " + String(FwRevision));
#endif
	dbg_println(("\nLoad non-volatile configuration and calibration data from flash"));
#ifdef EPAPER_DISPLAY
	display_add_boot_message("Load Flash");
#endif
	nvd_config_load(Config);
	nvd_calib_load(Calib);
	adc_init();
	int adcVal = adc_sample_average();
#ifdef BATTERY_VOLTAGE_MONITOR
	BatteryVoltage = adc_battery_voltage(adcVal);
#endif

	bWebConfigure = false;
	dbg_println(("To start web configuration mode, press and hold the PCC button"));
	dbg_println(("until you hear a low-frequency tone. Then release the button"));
	for (int cnt = 0; cnt < 6; cnt++) {
		dbg_println((8-cnt));
		delay(500);
		if (digitalRead(pinPCCA) == 0) {
			bWebConfigure = true;
			break;
			}
		}

  setCpuFrequencyMhz(80);
  uint32_t Freq = getCpuFrequencyMhz();
  Serial.print("CPU Freq = ");
  Serial.print(Freq);
  Serial.println(" MHz");
#ifdef EPAPER_DISPLAY
	display_add_boot_message("CPU Freq = " + String(Freq) + " Mhz");
#endif
  Freq = getXtalFrequencyMhz();
  Serial.print("XTAL Freq = ");
  Serial.print(Freq);
  Serial.println(" MHz");
#ifdef EPAPER_DISPLAY
	display_add_boot_message("XTAL Freq = " + String(Freq) + " Mhz");
#endif
  Freq = getApbFrequency();
  Serial.print("APB Freq = ");
  Serial.print(Freq);
  Serial.println(" Hz");
#ifdef EPAPER_DISPLAY
	display_add_boot_message("APB Freq = " + String(Freq) + " Mhz");
#endif	
   	
	if (bWebConfigure == true) {
		dbg_println(("Web configuration mode"));
#ifdef EPAPER_DISPLAY
		String messages[] = {"Web config", "mode"};
		display_show_modal_message("INFO", messages, 2);
#endif
		// 3 second long tone with low frequency to indicate unit is now in web server configuration mode.
		// After you are done with web configuration, switch off the vario as the wifi radio
		// consumes a lot of power.
		audio_generate_tone(200, 3000);
    	xTaskCreate( wifi_config_task, "wifi_config_task", 4096, NULL, WIFI_CFG_TASK_PRIORITY, NULL );
		}
  	else {
		dbg_println(("Vario mode"));
#ifdef BATTERY_VOLTAGE_MONITOR
		dbg_println(("\nAudio indication of battery voltage"));
		ui_indicate_battery_voltage(BatteryVoltage);
#endif
    	xTaskCreate( vario_task, "vario_task", 4096, NULL, VARIO_TASK_PRIORITY, NULL );
#ifdef EPAPER_DISPLAY
		xTaskCreate( epaper_task, "epaper_task", 4096, NULL, EPAPER_TASK_PRIORITY, NULL );
#endif
		}
	// delete the loopTask which called setup() from arduino app_main()
	vTaskDelete(NULL);
	}

static void ble_task(void* pvParameter){
	ble_uart_init();
	int counter = 0;
	dbg_println(("\nBluetooth LE LK8EX1 messages @ 10Hz\n"));
#ifdef EPAPER_DISPLAY
	display_add_boot_message("Bluetooth LK8EX1 @ 10Hz");
#endif
	while (1) {
		counter++;
		if (counter > 9) {
			counter = 0;
			LEDState = !LEDState;
			digitalWrite(pinLED, LEDState);
			}
#ifdef BATTERY_VOLTAGE_MONITOR
		BatteryVoltage = adc_battery_voltage();
		ble_uart_transmit_LK8EX1(((float)AltitudeCm)/100., ClimbrateCps, BatteryVoltage);
		//ble_uart_transmit_XCTRC(((float)AltitudeCm)/100., ((float)ClimbrateCps)/100., PressurePa, BatteryVoltage);
#else
		ble_uart_transmit_LK8EX1(((float)AltitudeCm)/100., ClimbrateCps, 100);
		//ble_uart_transmit_XCTRC(((float)AltitudeCm)/100., ((float)ClimbrateCps)/100., PressurePa, 100);
#endif
		vTaskDelay(100/portTICK_PERIOD_MS);
		}
	}

static void wifi_config_task(void * pvParameter) {
	if (!LittleFS.begin()){
		dbg_println(("Error mounting LittleFS, restarting..."));
#ifdef EPAPER_DISPLAY
		String messages[] = {"LittleFS", "mounting"};
		display_show_modal_message("ERROR", messages, 2);
#endif
		delay(1000);
		ESP.restart();
		}   
	wificfg_ap_server_init(); 
	while (1) {
		// nothing to do, async web server 
		vTaskDelay(1);
		}
	vTaskDelete(NULL);
	}


static void vario_task(void * pvParameter) {
	float accelmG[3];  // milli-Gs
	float gyroDps[3];  // degrees/second
	float mag[3];      // unitless vector  
	float kfAltitudeCm = 0.0f; // kalman filtered altitude in cm
	float kfClimbrateCps = 0.0f;  // kalman filtered climb/sink rate in cm/s

	uint32_t timePreviousUs, timeNowUs; // time markers
	float imuTimeDeltaUSecs; // time between imu samples, in microseconds
	float kfTimeDeltaUSecs; // time between kalman filter updates, in microseconds

	int pwrOffCounter, baroCounter, drdyCounter;
	int pwrOffTimeoutSecs;

	dbg_println(("\nCheck communication with MS5611"));
#ifdef EPAPER_DISPLAY
	display_add_boot_message("[CHECK] MS5611");
#endif
	if (!Baro.read_prom()) {
		dbg_println(("Bad CRC read from MS5611 calibration PROM"));
		dbg_flush();
#ifdef EPAPER_DISPLAY
	display_add_boot_message("[ERROR] MS5611 bad CRC");
#endif
		ui_indicate_fault_MS5611(); 
		}
	dbg_println(("MS5611 OK"));
#ifdef EPAPER_DISPLAY
	display_add_boot_message("[OK] MS5611");
#endif  
	dbg_println(("\nCheck communication with MPU9250"));
#ifdef EPAPER_DISPLAY
	display_add_boot_message("[ChHECK] MPU9250");
#endif
	if (!Imu.check_id()) {
		dbg_println(("Error reading Mpu9250 WHO_AM_I register"));
#ifdef EPAPER_DISPLAY
	display_add_boot_message("[ERROR] MPU9250");
#endif
		dbg_flush();
		ui_indicate_fault_MPU9250();
		}
	dbg_println(("MPU9250 OK"));
#ifdef EPAPER_DISPLAY
	display_add_boot_message("[OK] MPU9250");
#endif

#ifdef USE_9DOF_AHRS    
	// configure MPU9250 to start generating gyro, accel and mag data  
	Imu.config_accel_gyro_mag();
	// calibrate gyro (accel + mag if required)
	ui_calibrate_accel_gyro_mag();
#else
	// configure MPU9250 to start generating gyro, accel data  
	Imu.config_accel_gyro();
	// calibrate gyro (accel  if required)
	ui_calibrate_accel_gyro();
#endif
	delay(50);  
	  
	dbg_println(("\nMS5611 config"));
#ifdef EPAPER_DISPLAY
	display_add_boot_message("MS5611 config");
#endif
	Baro.reset();
	Baro.get_calib_coefficients(); // load MS5611 factory programmed calibration data
	Baro.averaged_sample(4); // get an estimate of starting altitude
	Baro.init_sample_state_machine(); // start the pressure & temperature sampling cycle

	dbg_println(("\nKalmanFilter config"));
#ifdef EPAPER_DISPLAY
	display_add_boot_message("KalmanFilter Config");
#endif
	// initialize kalman filter with MS5611 estimated altitude, estimated initial climbrate = 0.0
	kalmanFilter4d_configure(1000.0f*(float)Config.kf.accelVariance, ((float)Config.kf.adapt)/100.0f, Baro.altitudeCmAvg, 0.0f, 0.0f);

	if (Config.misc.bleEnable) {
		xTaskCreate(ble_task, "ble_task", 4096, NULL, BLE_TASK_PRIORITY, NULL );
		}

	vaudio_config();  
	timeNowUs = timePreviousUs = micros();
	ringbuf_init(); 
	ui_btn_init();	
	// interrupt output of MPU9250 is configured as push-pull, active high pulse. This is connected to
	// pinDRDYInt which has an external 10K pull-down resistor
	pinMode(pinDRDYInt, INPUT); 
    DrdySemaphore = xSemaphoreCreateBinary();
	baroCounter = pwrOffCounter = drdyCounter = 0;
	pwrOffTimeoutSecs = 0;
	kfTimeDeltaUSecs = imuTimeDeltaUSecs = 0.0f;
	attachInterrupt(pinDRDYInt, drdy_interrupt_handler, RISING);
#ifdef EPAPER_DISPLAY
	display_add_boot_message("-- BOOT COMPLETE --");
	if (!Config.misc.screenEnable) {
		display_off();
	}
#endif
	if (!Config.misc.soundEnable) {
		audio_off();
	}
	boot_complete = true;
	while (1) {
		// MPU9250 500Hz ODR => 2mS sample interval
		// wait for data ready interrupt from MPU9250 
		xSemaphoreTake(DrdySemaphore, portMAX_DELAY); 
		timeNowUs = micros();
		imuTimeDeltaUSecs = timeNowUs > timePreviousUs ? (float)(timeNowUs - timePreviousUs) : 2000.0f; // if rollover use expected time difference
		timePreviousUs = timeNowUs;
		drdyCounter++;
		uint32_t marker = micros(); // set marker for estimating the time taken to read and process the data
#ifdef USE_9DOF_AHRS
		// accelerometer samples (ax,ay,az) in milli-Gs, gyroscope samples (gx,gy,gz) in degrees/second, mag samples are unitless
		Imu.get_accel_gyro_mag_data(accelmG, gyroDps, mag); 
#else
		// accelerometer samples (ax,ay,az) in milli-Gs, gyroscope samples (gx,gy,gz) in degrees/second
		Imu.get_accel_gyro_data(accelmG, gyroDps); 
#endif
		// W.r.t. the CJMCU-117 board mounting on the VhARIO-ESPC3 PCB layout,
		// we arbitrarily decide that the CJMCU-117 board silkscreen -X points "forward" or "north", 
		// silkscreen -Y points "right" or "east", and silkscreen +Z points down. This is the North-East-Down (NED) 
		// right-handed coordinate frame used in the AHRS algorithm implementation.
		// The mapping from sensor frame to NED frame is : 
		// gn = gx, ge = -gy, gd = gz  : clockwise rotations about the + ned axis must result in +ve readings
		// an = ax, ae = ay, ad = -az   : when the + ned axis points down, axis reading must be +ve max
		// mn = -my, me = -mx, md = -mz   : when the + ned axis points magnetic north, axis reading must be +ve max
		// The AHRS algorithm expects rotation rates in radians/second
		// Acceleration data is only used for orientation correction when the acceleration magnitude is between 0.75G and 1.25G
		float accelMagnitudeSquared = accelmG[0]*accelmG[0] + accelmG[1]*accelmG[1] + accelmG[2]*accelmG[2];
		int bUseAccel = ((accelMagnitudeSquared > 562500.0f) && (accelMagnitudeSquared < 1562500.0f)) ? 1 : 0;
		float dtIMU = imuTimeDeltaUSecs/1000000.0f;

		float gn = DEG_TO_RAD*gyroDps[0];
		float ge = -DEG_TO_RAD*gyroDps[1];
		float gd = DEG_TO_RAD*gyroDps[2];
		float an = accelmG[0];
		float ae = accelmG[1];
		float ad = -accelmG[2];
#ifdef USE_9DOF_AHRS		
		float mn = -mag[1];
		float me = -mag[0];
		float md = -mag[2];
		imu_mahonyAHRS_update9DOF(bUseAccel, true, dtIMU, gn, ge, gd, an, ae, ad, mn, me, md);
#else
		imu_mahonyAHRS_update6DOF(bUseAccel, dtIMU, gn, ge, gd, an, ae, ad);
#endif		
		float gCompensatedAccel = imu_gravity_compensated_accel(an, ae, ad, Q0, Q1, Q2, Q3);
		ringbuf_add_sample(gCompensatedAccel);  
		baroCounter++;
		kfTimeDeltaUSecs += imuTimeDeltaUSecs;
		if (baroCounter >= 5) { // 5*2mS = 10mS elapsed, this is the sampling period for MS5611, 
			baroCounter = 0;    // alternating between pressure and temperature samples
			// one altitude sample is calculated for every new pair of pressure & temperature samples
			int zMeasurementAvailable = Baro.sample_state_machine(); 
			if ( zMeasurementAvailable ) { 
				// average earth-z acceleration over the 20mS interval between z samples
				// is used in the kf algorithm update phase
				float zAccelAverage = ringbuf_average_newest_samples(10); 
				float dtKF = kfTimeDeltaUSecs/1000000.0f;
				kalmanFilter4d_predict(dtKF);
				kalmanFilter4d_update(Baro.altitudeCm, zAccelAverage, (float*)&kfAltitudeCm, (float*)&kfClimbrateCps);
				// reset time elapsed between kalman filter algorithm updates
				kfTimeDeltaUSecs = 0.0f;
				AltitudeCm = kfAltitudeCm;
				ClimbrateCps = F_TO_I(kfClimbrateCps);
				PressurePa = Baro.pressurePa;
				vaudio_tick_handler(ClimbrateCps); // audio feedback handler
#ifdef PWR_OFF_AUTO
				if (ABS(ClimbrateCps) > PWR_OFF_THRESHOLD_CPS) { 
					// reset power-off timeout watchdog if there is significant vertical motion
					pwrOffTimeoutSecs = 0;
					}
				else
				if (pwrOffTimeoutSecs >= (Config.misc.pwrOffTimeoutMinutes*60)) {
					dbg_println(("Timed out with no significant climb/sink, power down"));
					ui_indicate_power_off();
					}   
#endif
				}
			}
			
		if (BtnPCCAPressed) {
			BtnPCCAPressed = false;
			#ifdef SOFTWARE_MUTE
			IsMuted = !IsMuted;
			if (IsMuted) audio_off();
			#endif
			}
		if (BtnPCCALongPress) {
			BtnPCCALongPress = false;
			#ifdef EPAPER_DISPLAY
			display_toggle();
			#endif
		}
#ifdef HAS_BUTTON_2
		if (Btn2Pressed) {
			Btn2Pressed = false;
			display_cycle_alt();
			}
		if (Btn2LongPress) {
			Btn2LongPress = false;
			display_reset_alt();
			}
#endif
		uint32_t elapsedUs =  micros() - marker; // calculate time  taken to read and process the data, must be less than 2mS
		if (drdyCounter >= 500) {
			drdyCounter = 0; // 1 second elapsed
			pwrOffTimeoutSecs++;
			#ifdef IMU_DEBUG
			//float yaw, pitch, roll;
			//imu_quaternion_to_yaw_pitch_roll(Q0,Q1,Q2,Q3, &yaw, &pitch, &roll);
			// Pitch is positive for clockwise rotation about the NED frame +Y axis
			// Roll is positive for clockwise rotation about the NED frame +X axis
			// Yaw is positive for clockwise rotation about the NED frame +Z axis
			// If magnetometer isn't used, yaw is initialized to 0 on power up.
			//dbg_printf(("\nY = %d P = %d R = %d\n", (int)yaw, (int)pitch, (int)roll));
			//dbg_printf(("kv = %d, timeout_counter = %d\n", (int)kfClimbrateCps, pwrOffTimeoutSecs));
			//dbg_printf(("ax = %.1f ay = %.1f az = %.1f\n",accelmG[0], accelmG[1], accelmG[2]));
			//dbg_printf(("gx = %.1f gy = %.1f gz = %.1f\n",gyroDps[0], gyroDps[1], gyroDps[2]));
			//dbg_printf(("mx = %.1f my = %.1f mz = %.1f\n",mag[0], mag[1], mag[2]));
			//dbg_printf(("Elapsed %dus\n", (int)elapsedUs)); 
			#endif     
			}
		}
	vTaskDelete(NULL);
	}

#ifdef EPAPER_DISPLAY
static void epaper_task(void * pvParameter) {
	while (1) {
		if (boot_complete) {
			if (EpaperBackgroundUpdate <= 0) {
				EpaperBackgroundUpdate = EPAPER_BACKGROUND_REFRESH_CYCLES;
				//display_draw_background();
			}
			display_refresh_data(((float)AltitudeCm)/100., ClimbrateCps);
			EpaperBackgroundUpdate--;
		}
		vTaskDelay(EPAPER_REFRESH_RATE_MS/portTICK_PERIOD_MS);
	}
	vTaskDelete(NULL);
}
#endif

void loop(){
	}
