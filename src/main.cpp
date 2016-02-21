#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <Adafruit_ADS1015.h>

Adafruit_ADS1015 adc;

// AD8307
// Start 0.25V -74dBm
// 25mV/dB
//
//
// Direction Coupler: 26dB
// Input Att: 16.1dB

void setup () {
	Serial.begin(115200);
	Serial.println("init");

	pinMode(13, OUTPUT);

	// fullscale = ±4.096V
	adc.begin();
	adc.setGain(GAIN_ONE);
}

constexpr float AD9807_DBM_ORIGIN = -84.0; // 0.25/-74dBm => 0V/-84dBm
constexpr float COUPLING_FACTOR = -26.02;
constexpr float ATTENATOR_FACTOR = -16.1;

constexpr float CALIB_A = 0.4036326942482341;

void loop() {
	digitalWrite(13, HIGH);

	uint16_t adc_fwd = adc.readADC_SingleEnded(0) * 2;
	uint16_t adc_ref = adc.readADC_SingleEnded(1) * 2;
	uint16_t adc_vcc = adc.readADC_SingleEnded(2) * 2;
	uint16_t adc_gnd = adc.readADC_SingleEnded(3) * 2;

//	uint16_t adc_fwd = adc.readADC_Differential_0_3_V() * 1000;
//	uint16_t adc_ref = adc.readADC_Differential_1_3_V() * 1000;
//	uint16_t adc_vcc = adc.readADC_Differential_2_3_V() * 1000;
//	uint16_t adc_gnd = adc.readADC_SingleEnded(3) * 2;

	Serial.print("AIN fwd: "); Serial.println(adc_fwd);
	Serial.print("AIN ref: "); Serial.println(adc_ref);
	Serial.print("AIN vcc: "); Serial.println(adc_vcc);
	Serial.print("AIN gnd: "); Serial.println(adc_gnd);

	if (adc_fwd < adc_ref) {
		uint16_t tmp = adc_fwd;
		adc_fwd = adc_ref;
		adc_ref = tmp;
	}

	float dbm_fwd = (float)adc_fwd / 25.0 + AD9807_DBM_ORIGIN - COUPLING_FACTOR - ATTENATOR_FACTOR;
	float dbm_ref = (float)adc_ref / 25.0 + AD9807_DBM_ORIGIN - COUPLING_FACTOR - ATTENATOR_FACTOR;
	Serial.print("dbm_fwd = "); Serial.println(dbm_fwd);
	Serial.print("dbm_ref = "); Serial.println(dbm_ref);

	float watts_fwd = pow(10, dbm_fwd / 10) / 1000.0;
	float watts_ref = pow(10, dbm_ref / 10) / 1000.0;
//	Serial.print("watts_fwd = "); Serial.println(watts_fwd);
//	Serial.print("watts_ref = "); Serial.println(watts_ref);

	watts_fwd = CALIB_A * watts_fwd;
	watts_ref = CALIB_A * watts_ref;
	Serial.print("watts_fwd (W) = "); Serial.println(watts_fwd);
	Serial.print("watts_fwd (mW) = "); Serial.println(watts_fwd * 1000);
	Serial.print("watts_fwd (uW) = "); Serial.println(watts_fwd * 1000 * 1000);
	Serial.print("watts_ref = "); Serial.println(watts_ref);

	Serial.println(" ");
	digitalWrite(13, LOW);
	delay(1000);
}
