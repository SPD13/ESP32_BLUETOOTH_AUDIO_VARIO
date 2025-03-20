#ifndef CONFIG_H_
#define CONFIG_H_

#define pinPCCA		21	// program/configure/calibrate/audio button
#define HAS_BUTTON_2
#define pinBut2	    22  // Button 2
#define pinAudio	14	// pwm beeper audio output
#define pinAudioEn	6	// 74HC240 output enables, active low

#define pinCSB		12	// CSB (ms5611)
#define pinMISO		27	// SDO ms5611 & AD0 mpu9250
#define pinNCS		17 	// NCS (mpu9250)
#define pinMOSI		26 	// SDA & ePaper SDA
#define pinSCK		25	// SCL & ePaper Clock
#define pinDRDYInt	4  // INT
#define pinLED		16	// power-on and bluetooth active indication

//#define Serial USBSerial

//#define BATTERY_VOLTAGE_MONITOR //comment to disable battery monitoring
//#define AUDIO_ENABLE_PIN //Comment to disable audio enable through a pin

#define EPAPER_DISPLAY 							//Comment to remove display functions
#define EPAPER_REFRESH_RATE_MS 				500
#define EPAPER_BACKGROUND_REFRESH_CYCLES 	10 	//How many quick updates before a full refresh

// mapping suggestion for ESP32, e.g. TTGO T8 ESP32-WROVER
// BUSY -> 4, RST -> 2, DC -> 0, CS -> SS(5), CLK -> SCK(25), DIN/SDA -> MOSI(26), GND -> GND, 3.3V -> 3.3V
// for use with Board: "ESP32 Dev Module":
#define EPAPER_PIN_CS 5
#define EPAPER_PIN_DC 0
#define EPAPER_PIN_RES 2
#define EPAPER_PIN_BUSY 15

#define BTN_PCCA()  (digitalRead(pinPCCA) == LOW ? 1 : 0) //Don't know why it's inverted...
#define BTN_2()  (digitalRead(pinBut2) == LOW ? 1 : 0)
#define SOFTWARE_MUTE //Comment to disable muting from PCCA button

#define LED_ON() 	{digitalWrite(pinLED, 0); LEDState = 1;}
#define LED_OFF() 	{digitalWrite(pinLED, 1); LEDState = 0;}

#define PWR_ON_DELAY_MS		1000UL
#define PWR_OFF_DELAY_MS	2000UL

////////////////////////////////////////////////////////////////////
// WEB CONFIGURATION PARAMETER DEFAULTS AND LIMITS

// vario thresholds in cm/sec for generating different
// audio tones. Between the sink threshold and the zero threshold,
// the vario is quiet

#define VARIO_CLIMB_THRESHOLD_CPS_DEFAULT  	50
#define VARIO_CLIMB_THRESHOLD_CPS_MIN   	20
#define VARIO_CLIMB_THRESHOLD_CPS_MAX   	100

#define VARIO_ZERO_THRESHOLD_CPS_DEFAULT  	5
#define VARIO_ZERO_THRESHOLD_CPS_MIN    	-20
#define VARIO_ZERO_THRESHOLD_CPS_MAX    	20

#define VARIO_SINK_THRESHOLD_CPS_DEFAULT  	-250
#define VARIO_SINK_THRESHOLD_CPS_MIN    	-400
#define VARIO_SINK_THRESHOLD_CPS_MAX    	-100

// When generating climbtones, the vario allocates most of the speaker 
// frequency bandwidth to climbrates below this crossover threshold 
// so you have more frequency discrimination. So set the crossover threshold 
// to the average thermal core climbrate you expect for the site and conditions.
#define VARIO_CROSSOVER_CPS_DEFAULT     400
#define VARIO_CROSSOVER_CPS_MIN         300
#define VARIO_CROSSOVER_CPS_MAX         600

// Kalman filter configuration
#define KF_ACCEL_VARIANCE_DEFAULT     100
#define KF_ACCEL_VARIANCE_MIN         50
#define KF_ACCEL_VARIANCE_MAX         150

#define KF_ADAPT_DEFAULT	100
#define KF_ADAPT_MIN		50
#define KF_ADAPT_MAX		150

// Power-off timeout. The vario will power down
// if it does not detect climb or sink rates more than
// PWR_OFF_THRESHOLD_CPS, for the specified minutes.
//#define PWR_OFF_AUTO						//Comment to disable auto power off
#define PWR_OFF_TIMEOUT_MINUTES_DEFAULT   10
#define PWR_OFF_TIMEOUT_MINUTES_MIN       5
#define PWR_OFF_TIMEOUT_MINUTES_MAX       20

// audio feedback tones
#define BATTERY_TONE_HZ			400
#define CALIBRATING_TONE_HZ		800
#define UNCALIBRATED_TONE_HZ	2000
#define MPU9250_ERROR_TONE_HZ	200 
#define MS5611_ERROR_TONE_HZ	2500

// BLE transmission is enabled as default
#define BLE_DEFAULT  true
// BLE protocol
#define BLE_PROTOCOL_DEFAULT  0
// Sound on startup is enabled as default
#define SOUND_DEFAULT  true
// Screen on startup is enabled as default
#define SCREEN_DEFAULT  true

////////////////////////////////////////////////////////////////////////////////
// COMPILED CONFIGURATION PARAMETERS ( cannot be changed with web configuration )

//#define USE_9DOF_AHRS

#define BLE_TASK_PRIORITY		3
#define WIFI_CFG_TASK_PRIORITY	3
#define VARIO_TASK_PRIORITY		(configMAX_PRIORITIES-2)
#define EPAPER_TASK_PRIORITY	2

// change these parameters based on the frequency bandwidth of the speaker
#define VARIO_SPKR_MIN_FREQHZ      	200
#define VARIO_SPKR_MAX_FREQHZ       3200

// three octaves (2:1) of frequency for climbrates below crossoverCps,
// and one octave of frequency for climbrates above crossoverCps.
// This gives you more perceived frequency discrimination for climbrates 
// below crossoverCps
#define VARIO_CROSSOVER_FREQHZ    	1600

#define KF_ADAPT         1.0f

// Acceleration bias uncertainty is set low as the residual acceleration bias 
// (post-calibration) is expected to have low variation/drift. It is further reduced
// depending on the acceleration magnitude, as we want the acceleration bias estimate 
// to evolve ideally in a zero acceleration environment.
#define KF_ACCELBIAS_VARIANCE   	0.005f

// KF4 Acceleration Measurement Noise variance
#define KF_A_MEAS_VARIANCE   		10.0f

// KF4 Altitude Measurement Noise Variance
#define KF_Z_MEAS_VARIANCE			200.0f

// If climbrate or sinkrate stays below this threshold for the configured
// time interval, vario goes to sleep to conserve power
#define PWR_OFF_THRESHOLD_CPS    	50

// if you find that gyro calibration fails even when you leave
// the unit undisturbed, increase this offset limit
// until you find that gyro calibration works consistently.
#define GYRO_OFFSET_LIMIT_1000DPS   200

// print debug information to the serial port for different code modules
// For revB hardware, after flashing the code and validating the calibration parameters look reasonable,
// ensure this is commented out
#define TOP_DEBUG
#ifdef TOP_DEBUG
	#define dbg_println(x) {Serial.println x;}
	#define dbg_printf(x)  {Serial.printf x;}
	#define dbg_flush()  Serial.flush()
#else
	#define dbg_println(x)
	#define dbg_printf(x)
	#define dbg_flush()
#endif
// these #defines can be left uncommented after debugging, as the enclosed
// debug prints do not appear in the critical run-time loop
#define KF_DEBUG
#define VARIO_DEBUG
#define NVD_DEBUG
#define MPU9250_DEBUG
#define MS5611_DEBUG
#define WEBCFG_DEBUG

// !! ensure these #defines are commented out after debugging, as the 
// enclosed debug prints are in the critical run-time loop.
//#define IMU_DEBUG
//#define PERF_DEBUG
//#define BLE_DEBUG

#endif
