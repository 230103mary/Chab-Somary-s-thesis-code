
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>

Adafruit_ADS1015 myADS;
LiquidCrystal_I2C lcd(0x27, 20, 4);

unsigned long t0;
int16_t current_reading, voltage_reading;
volatile bool new_data = false;
volatile int channel = 0; // 0 for current, 1 for voltage

const int intPin = 5;
const float conversion_factor = 2.048 / 2048.0; // mV to V conversion for ADS1015 with GAIN_TWO



#define GAIN_CURRENT (9.5) 
#define GAIN_VOLTAGE (360) 

// Variables to store readings
float current_samples[1000];
float voltage_samples[1000];
unsigned long timestamps[1000];
int sample_index = 0;

// Frequency calculation variables
int zero_crossings[1000];
int zero_cross_index = 0;
float frequency = 0.0;

void newDataReady() {
    new_data = true;
}

void setup() {
    Serial.begin(115200);
    attachInterrupt(digitalPinToInterrupt(intPin), newDataReady, FALLING);

    myADS.begin();
    myADS.setGain(GAIN_TWO);
    myADS.setDataRate(RATE_ADS1015_3300SPS); // 3300 samples per second
    myADS.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_0, true); // Start with current sensor

    t0 = micros();

    lcd.init(); // Initialize the LCD
    lcd.backlight();
    lcd.print("Initializing...");
}

void loop() {
    if (!new_data) return;

    if (channel == 0) {
        current_reading = myADS.getLastConversionResults();
        myADS.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_1, true); // Switch to voltage sensor
        channel = 1;
    } else {
        voltage_reading = myADS.getLastConversionResults();
        myADS.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_0, true); // Switch to current sensor
        channel = 0;
    }

    timestamps[sample_index] = micros() - t0;
    current_samples[sample_index] = (((current_reading * conversion_factor) -1) * GAIN_CURRENT);
    voltage_samples[sample_index] = (((voltage_reading * conversion_factor) - 0.002) * GAIN_VOLTAGE);

    // Detect zero-crossings in the voltage signal
    if (sample_index > 0 && voltage_samples[sample_index - 1] <= 0 && voltage_samples[sample_index] > 0) {
        zero_crossings[zero_cross_index++] = sample_index;
    }

    sample_index++;

    if (sample_index >= 1000) {
        calculateAndDisplay();
        sample_index = 0;
    }

    new_data = false;
}

void calculateAndDisplay() {
    // Calculate RMS values
    float sum_current = 0;
    float sum_voltage = 0;
    for (int i = 0; i < 1000; i++) {
        sum_current += current_samples[i] * current_samples[i];
        sum_voltage += voltage_samples[i] * voltage_samples[i];
    }
    float Irms = sqrt(sum_current / 1000);
    float Vrms = sqrt(sum_voltage / 1000);

    // Calculate phase difference using cross-correlation
    float phase_diff = calculatePhaseDifferenceUsingCrossCorrelation(current_samples, voltage_samples);

    // Calculate power factor
    float power_factor = cos(phase_diff * PI / 180.0);

    // Calculate real power
    float real_power = Vrms * Irms * power_factor;

    // Calculate frequency
    if (zero_cross_index > 1) {
        float period_sum = 0.0;
        for (int i = 1; i < zero_cross_index; i++) {
            int period_samples = zero_crossings[i] - zero_crossings[i - 1];
            float period_seconds = period_samples * (1.0 / 3300.0); // 3300 samples per second
            period_sum += period_seconds;
        }
        float average_period = period_sum / (zero_cross_index - 1);
        frequency = 1.0 / average_period;
    }

    // Display the results on the LCD and Serial
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Vrms: ");
    lcd.print(Vrms, 2);
    lcd.setCursor(0, 1);
    lcd.print("Irms: ");
    lcd.print(Irms, 2);
    delay(2000);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("P.Factor: ");
    lcd.print(power_factor, 2);
    lcd.setCursor(0, 1);
    lcd.print("Power: ");
    lcd.print(real_power, 2);
    delay(2000);

lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Freq: ");
    lcd.print(frequency, 2); // Display frequency with 2 decimal places
    lcd.print(" Hz");
    delay(2000);

    // Print to Serial for debugging
    Serial.print("Vrms: ");
    Serial.print(Vrms, 2);
    Serial.print(" V, Irms: ");
    Serial.print(Irms, 2);
    Serial.print(" A, Power Factor: ");
    Serial.print(power_factor, 2);
    Serial.print(", Real Power: ");
    Serial.print(real_power, 2);
    Serial.print(", Frequency: ");
    Serial.print(frequency, 2); // Print frequency with 2 decimal places
    Serial.println(" Hz");
}

float calculatePhaseDifferenceUsingCrossCorrelation(float *current_samples, float *voltage_samples) {
    float max_correlation = 0;
    int best_lag = 0;
    int max_lag = 100; // Adjust as needed

    for (int lag = -max_lag; lag <= max_lag; lag++) {
        float correlation = 0;
        for (int i = 0; i < 1000; i++) {
            int j = i + lag;
            if (j >= 0 && j < 1000) {
                correlation += current_samples[i] * voltage_samples[j];
            }
        }
        if (correlation > max_correlation) {
            max_correlation = correlation;
            best_lag = lag;
        }
    }

    float time_diff = best_lag * (timestamps[1] - timestamps[0]) / 1e6; // in seconds
    float period = 1.0 / frequency;
    return (time_diff / period) * 360.0; // Phase difference in degrees
}

