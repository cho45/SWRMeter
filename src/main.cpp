#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <Adafruit_ADS1015.h>

Adafruit_ADS1015 adc;

// AD8307
// Start 0.25V -74dBm
// 25mV/dB
constexpr float AD9807_DBM_ORIGIN = -84.0; // 0.25/-74dBm => 0V/-84dBm
constexpr float COUPLING_FACTOR = -29.54;
constexpr float ATTENATOR_FACTOR = -16.1;

constexpr float CALIB_A = 1.010544815;
constexpr float CALIB_B = -102.0720562;

// threshold (W) for not in transmit
constexpr float TX_THRESHOLD_WATTS = 0.001;
constexpr float TX_THRESHOLD_dBm = 10.0 * log10(TX_THRESHOLD_WATTS/1e-3);
constexpr uint16_t TX_THRESHOLD_ADC = (TX_THRESHOLD_dBm + COUPLING_FACTOR + ATTENATOR_FACTOR - AD9807_DBM_ORIGIN) * 25;

String formatWatts(float watts) {
	if (watts < 1e-6) {
		return String(watts * 1e6) + "uW";
	} else
	if (watts < 1e-3) {
		return String(watts * 1e3) + "mW";
	} else {
		return String(watts) + "W";
	}
}

void setup () {
	Serial.begin(115200);
	Serial.println("init");
	Serial.print("TX_THRESHOLD_WATTS = "); Serial.println(TX_THRESHOLD_WATTS);
	Serial.print("TX_THRESHOLD_dBm = "); Serial.println(TX_THRESHOLD_dBm);
	Serial.print("TX_THRESHOLD_ADC = "); Serial.println(TX_THRESHOLD_ADC);

	pinMode(13, OUTPUT);

	// fullscale = ±4.096V
	adc.begin();
	adc.setGain(GAIN_ONE);
}



void loop() {
	digitalWrite(13, HIGH);

	uint16_t adc_fwd = adc.readADC_SingleEnded(0) * 2;
	uint16_t adc_ref = adc.readADC_SingleEnded(1) * 2;
	uint16_t adc_vcc = adc.readADC_SingleEnded(2) * 2;
	uint16_t adc_gnd = adc.readADC_SingleEnded(3) * 2;
	if (adc_fwd < adc_ref) {
		uint16_t tmp = adc_fwd;
		adc_fwd = adc_ref;
		adc_ref = tmp;
	}

	Serial.print("AIN fwd: "); Serial.println(adc_fwd);
	Serial.print("AIN ref: "); Serial.println(adc_ref);
	Serial.print("AIN vcc: "); Serial.println(adc_vcc);
	Serial.print("AIN gnd: "); Serial.println(adc_gnd);

	adc_fwd = adc_fwd * CALIB_A + CALIB_B;
	adc_ref = adc_ref * CALIB_A + CALIB_B;
	if (adc_fwd < TX_THRESHOLD_ADC) {
		Serial.println(" ");
		digitalWrite(13, LOW);
		delay(1000);
		return;
	}

	float dbm_fwd = (float)adc_fwd / 25.0 + AD9807_DBM_ORIGIN - COUPLING_FACTOR - ATTENATOR_FACTOR;
	float dbm_ref = (float)adc_ref / 25.0 + AD9807_DBM_ORIGIN - COUPLING_FACTOR - ATTENATOR_FACTOR;
	Serial.print("dbm_fwd = "); Serial.println(dbm_fwd);
	Serial.print("dbm_ref = "); Serial.println(dbm_ref);

	float watts_fwd = pow(10, dbm_fwd / 10) / 1000.0;
	float watts_ref = pow(10, dbm_ref / 10) / 1000.0;
	Serial.print("watts_fwd = "); Serial.println(formatWatts(watts_fwd));
	Serial.print("watts_ref = "); Serial.println(formatWatts(watts_ref));

//	Serial.print("watts_fwd (W) = "); Serial.println(watts_fwd);
//	Serial.print("watts_fwd (mW) = "); Serial.println(watts_fwd * 1000);
//	Serial.print("watts_fwd (uW) = "); Serial.println(watts_fwd * 1000 * 1000);
//	Serial.print("watts_ref = "); Serial.println(watts_ref);

	float reflection_coefficient = sqrt(watts_ref / watts_fwd);
	float swr = (1.0 + reflection_coefficient) / (1.0 - reflection_coefficient);
	if (swr < 0) swr = 0/1; // invalid

	Serial.println(" ");
	digitalWrite(13, LOW);
	delay(500);
}
