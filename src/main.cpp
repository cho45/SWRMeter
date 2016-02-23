#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <Adafruit_ADS1015.h>
#include <LiquidCrystal_I2C.h>

#include "interval.hpp"

#define DEBUG 1
#ifdef DEBUG
#define DEBUG_VALUE(prefix, value) Serial.print(prefix); Serial.println(value);
#else
#define DEBUG_VALUE(prefix, value)
#endif

String formatWatts(const float watts) {
	if (watts < 1e-6) {
		return String(watts * 1e6) + "uW";
	} else
	if (watts < 1e-3) {
		return String(watts * 1e3) + "mW";
	} else {
		return String(watts) + "W ";
	}
}

struct SensorResult {
	float dbm_fwd = 0;
	float dbm_ref = 0;
	float watts_fwd = 0;
	float watts_ref = 0;
	float swr = 1.0;
};

class Sensor {
	Adafruit_ADS1015 adc;
public:
	// AD8307
	// Start 0.25V -74dBm
	// 25mV/dB
	static constexpr float AD9807_DBM_ORIGIN = -84.0; // 0.25/-74dBm = > 0V/-84dBm
	static constexpr float COUPLING_FACTOR   = -29.54;
	static constexpr float ATTENATOR_FACTOR  = -16.1;

	// threshold (W) for not in transmit
	static constexpr float TX_THRESHOLD_WATTS  = 0.001;
	static constexpr float TX_THRESHOLD_dBm    = 10.0 * log10(TX_THRESHOLD_WATTS/1e-3);
	static constexpr uint16_t TX_THRESHOLD_ADC = (TX_THRESHOLD_dBm + COUPLING_FACTOR + ATTENATOR_FACTOR - AD9807_DBM_ORIGIN) * 25;

	static constexpr uint16_t SAMPLE_RATE  = 50; // SPS
	static constexpr uint16_t SAMPLE_RATE_MS = 1000 / SAMPLE_RATE;
	static constexpr uint16_t HISTORY_SIZE = SAMPLE_RATE * 5;

	uint32_t adc_next_sample_time = 0;
	uint16_t adc_fwds[HISTORY_SIZE];
	uint16_t adc_refs[HISTORY_SIZE];
	uint16_t  adc_pos = 0;

	// calibration factors
	float fwd_a, fwd_b;
	float ref_a, ref_b;

	static void calculate_calibration_factors(
			float expected_watts1, float got_adc1,
			float expected_watts2, float got_adc2,
			float& a, float& b
		) {

		float expected_adc1 = 25 * (10 * log10(expected_watts1/1e-3) + COUPLING_FACTOR + ATTENATOR_FACTOR - AD9807_DBM_ORIGIN);
		float expected_adc2 = 25 * (10 * log10(expected_watts2/1e-3) + COUPLING_FACTOR + ATTENATOR_FACTOR - AD9807_DBM_ORIGIN);
		float deno = (got_adc1 - got_adc2);
		a = (expected_adc1 - expected_adc2) / deno;
		b = (expected_adc2 * got_adc1 - expected_adc1 * got_adc2) / deno;
	}

	Sensor() {
		adc.begin();
		// fullscale = ±4.096V
		adc.setGain(GAIN_ONE);
	}

	void begin() {
		DEBUG_VALUE("TX_THRESHOLD_WATTS = ", TX_THRESHOLD_WATTS);
		DEBUG_VALUE("TX_THRESHOLD_dBm = ", TX_THRESHOLD_dBm);
		DEBUG_VALUE("TX_THRESHOLD_ADC = ", TX_THRESHOLD_ADC);
	}

	void set_calibration_factors(
		const float _fwd_a, const float _fwd_b,
		const float _ref_a, const float _ref_b
	) {
		fwd_a = _fwd_a;
		fwd_b = _fwd_b;
		ref_a = _ref_a;
		ref_b = _ref_b;
	}

	uint16_t read_fwd() {
		return adc.readADC_SingleEnded(0) * 2;
	}

	uint16_t read_ref() {
		return adc.readADC_SingleEnded(1) * 2;
	}

	/**
	 * Return VCC in mV
	 */
	uint16_t read_vcc() {
		return adc.readADC_SingleEnded(2) * 2;
	}

	/**
	 * Return GND in mV (typically 0)
	 */
	uint16_t read_gnd() {
		return adc.readADC_SingleEnded(3) * 2;
	}

	void process() {
		uint32_t now = millis();
		if (adc_next_sample_time < now) {
			adc_next_sample_time = now + SAMPLE_RATE_MS;
			sample();
		}
	}

	void sample() {
		float adc_fwd = read_fwd();
		float adc_ref = read_ref();
		adc_fwd = adc_fwd * fwd_a + fwd_b;
		adc_ref = adc_ref * ref_a + ref_b;
		if (adc_fwd < adc_ref) {
			uint16_t tmp = adc_fwd;
			adc_fwd = adc_ref;
			adc_ref = tmp;
		}

		adc_pos = (adc_pos + 1) % HISTORY_SIZE;
		adc_fwds[adc_pos] = adc_fwd;
		adc_refs[adc_pos] = adc_ref;
	}

	const SensorResult calculate_pep(uint8_t time_range) {
		uint16_t count = SAMPLE_RATE * time_range;

		uint16_t adc_fwd = 0, adc_ref = 0, val = 0;
		for (uint16_t i = 0; i < count; i++) {
			val = adc_fwds[ (adc_pos + HISTORY_SIZE - i) % HISTORY_SIZE ];
			if (adc_fwd < val) {
				adc_fwd = val;
			}

			val = adc_refs[ (adc_pos + HISTORY_SIZE - i) % HISTORY_SIZE ];
			if (adc_ref < val) {
				adc_ref = val;
			}
		}

		return calculate(adc_fwd, adc_ref);
	}

	const SensorResult calculate_avg(uint8_t time_range) {
		uint16_t count = SAMPLE_RATE * time_range;

		uint32_t adc_fwd = 0, adc_ref = 0;
		for (uint8_t i = 0; i < count; i++) {
			adc_fwd += adc_fwds[ (adc_pos + HISTORY_SIZE - i) % HISTORY_SIZE ];
			adc_ref += adc_refs[ (adc_pos + HISTORY_SIZE - i) % HISTORY_SIZE ];
		}

		return calculate(adc_fwd / count, adc_ref / count);
	}

	const SensorResult calculate_last() {
		return calculate(adc_fwds[adc_pos], adc_refs[adc_pos]);
	}

	const SensorResult calculate(const uint16_t adc_fwd, const uint16_t adc_ref) {
		SensorResult result;
		result.dbm_fwd = static_cast<float>(adc_fwd) / 25.0 + AD9807_DBM_ORIGIN - COUPLING_FACTOR - ATTENATOR_FACTOR;
		result.dbm_ref = static_cast<float>(adc_ref) / 25.0 + AD9807_DBM_ORIGIN - COUPLING_FACTOR - ATTENATOR_FACTOR;

		result.watts_fwd = pow(10, result.dbm_fwd / 10) / 1000.0;
		result.watts_ref = pow(10, result.dbm_ref / 10) / 1000.0;

		float reflection_coefficient = sqrt(result.watts_ref / result.watts_fwd);
		result.swr = (1.0 + reflection_coefficient) / (1.0 - reflection_coefficient);
		if (result.swr < 0) result.swr = 0/1; // invalid
		return result;
	}
};

class Display {
	LiquidCrystal_I2C lcd;
public:
	Display() :
		lcd(LiquidCrystal_I2C(0x27, 16, 2)) 
	{
	}

	enum {
		MODE_RX,
		MODE_TX
	} mode;

	float swr = 1.0;
	float fwd_pep = 0;
	float fwd_avg = 0;

	void begin() {
		mode = MODE_RX;
		lcd.begin();
		lcd.backlight();
		lcd.setCursor(0, 0);
		lcd.print("Initializing...");
	}

	void set_last_result(const SensorResult& result) {
	}

	void set_avg_result(const SensorResult& result) {
		fwd_avg = result.watts_fwd;
	}

	void set_pep_result(const SensorResult& result) {
		if (result.watts_fwd > Sensor::TX_THRESHOLD_WATTS) {
			mode = MODE_TX;
		} else
		if (result.watts_fwd < Sensor::TX_THRESHOLD_WATTS) {
			mode = MODE_RX;
		}
		fwd_pep = result.watts_fwd;
		swr = result.swr;
	}

	void update() {
		lcd.setCursor(0, 0);
		lcd.print(formatWatts(fwd_pep));
		lcd.print(" / ");
		lcd.print(formatWatts(fwd_avg));
		lcd.print("                ");

		if (mode == MODE_TX) {
			lcd.setCursor(0, 1);
			lcd.print("SWR:");
			lcd.print(String(swr));
			lcd.print("       ");
		} else {
			lcd.setCursor(0, 1);
			lcd.print("Standby         ");
		}
	}
};

class SerialCommand {
	char buffer[32];
	uint8_t buffer_pos;
	Sensor& sensor;

	uint16_t read_adc_tx(uint8_t direction) {
		Serial.println("WAITING TX...");
		uint16_t adc = 0;
		uint32_t end = millis() + (10UL * 50000);
		while (millis() < end) {
			uint16_t read = direction == 0 ? sensor.read_fwd() : sensor.read_ref();
			if (read > Sensor::TX_THRESHOLD_ADC) {
				Serial.print("TX ADC: ");
				Serial.println(read);
				if (adc < read) {
					adc = read;
				}
				delay(500);
			}
		}
		return adc;
	}

	void command() {
		if (strcmp(buffer, "CALFWD")) {
			Serial.println("CAL FWD MODE");
			{
				Serial.print("INPUT CAL POWER1: ");
				float power1 = Serial.parseFloat();
				uint16_t adc1 = read_adc_tx(0);
				if (!adc1) {
					Serial.println("FAILED");
					return;
				}
				Serial.print("OK: "); Serial.println(adc1);


				Serial.print("INPUT CAL POWER2: ");
				float power2 = Serial.parseFloat();
				uint16_t adc2 = read_adc_tx(0);
				if (!adc2) {
					Serial.println("FAILED");
					return;
				}
				Serial.print("OK: "); Serial.println(adc2);

				float a, b;
				Sensor::calculate_calibration_factors(power1, adc1, power2, adc2, a, b);
				Serial.println("Calculated factors:");
				Serial.print(" a = "); Serial.println(a);
				Serial.print(" b = "); Serial.println(b);
				sensor.fwd_a = a;
				sensor.fwd_b = b;
				Serial.println("Applied factors to sensor. (save to use SAVE command)");
			};
		} else
		if (strcmp(buffer, "CALREF")) {
			Serial.println("CAL REF MODE");
		} else
		if (strcmp(buffer, "SAVE")) {
			Serial.println("Saved.");
		} else {
			Serial.print("UNKNOWN COMMAND: ");
			Serial.println(buffer);
		}
	}
public:
	SerialCommand(Sensor& _sensor) :
		buffer_pos(0),
		sensor(_sensor)
	{
	}

	void process() {
		while (Serial.available()) {
			char c = Serial.read();
			switch (c) {
				case '\n':
					buffer[buffer_pos] = 0; // terminate
					buffer_pos = 0;
					// now buffer is null terminated string
					command();
					break;
				case '\b':
					buffer_pos--;
					break;
				default:
					if (buffer_pos < sizeof(buffer) - 1) {
						buffer[buffer_pos] = c;
						buffer_pos++;
					} else {
						// just ignore
					}
			}
		}
	}
};

constexpr float CALIB_A = 1.010544815;
constexpr float CALIB_B = -102.0720562;

Sensor sensor;
// SerialCommand serial_command(sensor);
//
Display display;

void setup () {
	pinMode(13, OUTPUT);
	Serial.begin(115200);

	display.begin();

	sensor.begin();
	sensor.set_calibration_factors(CALIB_A, CALIB_B, CALIB_A, CALIB_B);
}

void loop() {
	interval<Sensor::SAMPLE_RATE_MS>([]{
		sensor.sample();
	});

	interval<500>([]{
		digitalWrite(13, HIGH);

		DEBUG_VALUE("AIN vcc: ", sensor.read_vcc());
		DEBUG_VALUE("AIN gnd: ", sensor.read_gnd());

		{
			SensorResult result = sensor.calculate_last();
			DEBUG_VALUE("dbm_fwd = ", result.dbm_fwd);
			DEBUG_VALUE("dbm_ref = ", result.dbm_ref);
			DEBUG_VALUE("watts_fwd = ", formatWatts(result.watts_fwd));
			DEBUG_VALUE("watts_ref = ", formatWatts(result.watts_ref));
			DEBUG_VALUE("swr ", result.swr);
			display.set_last_result(result);
		};

		{
			SensorResult result = sensor.calculate_avg(1);
			DEBUG_VALUE("watts_fwd avg = ", formatWatts(result.watts_fwd));
			display.set_avg_result(result);
		};

		{
			SensorResult result = sensor.calculate_pep(3);
			DEBUG_VALUE("watts_fwd pep = ", formatWatts(result.watts_fwd));
			display.set_pep_result(result);
		};

		digitalWrite(13, LOW);

		display.update();
	});
}


void serialEvent() {
//	SerialCommand.process();
}
