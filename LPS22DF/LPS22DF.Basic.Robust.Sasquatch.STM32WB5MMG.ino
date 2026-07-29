/*
 * LPS22DF.Basic.Robust.Sasquatch.STM32WB5MMG
 *
 * Stand-alone LPS22DF reference sketch for the STM32WB5MMG Sasquatch.
 *
 * The normal build is silent and spends idle time in STM32WB STOP mode.
 * Set SERIAL_DEBUG true only while diagnosing or characterizing the sensor.
 *
 * Continuous mode:
 *   - LPS22DF samples at the selected ODR.
 *   - Pressure samples accumulate in the FIFO.
 *   - Sasquatch wakes only at the FIFO watermark, not for every sample.
 *
 * One-shot mode:
 *   - TimerMillis starts one conversion per minute.
 *   - A second TimerMillis instance checks conversion completion without
 *     blocking; the MCU returns to STOP after the sample is read.
 */

#include "Arduino.h"
#include "STM32WB.h"
#include "I2Cdev.h"
#include "LPS22DF.h"
#include "TimerMillis.h"

#define I2C_BUS Wire

constexpr bool SERIAL_DEBUG = false;
// Enable the per-entry FIFO dump and PRESS_OUT comparison while investigating
// FIFO behavior. This has no effect unless SERIAL_DEBUG is also true.
constexpr bool ENABLE_DIAGNOSTICS = false;
constexpr uint8_t LED_PIN = 23;                 // Red LED, active LOW.
constexpr uint8_t LPS22DF_INT_PIN = 8;

// Sensor configuration.
constexpr uint8_t PODR = P_1Hz;
constexpr uint8_t AVG = avg_64;
constexpr uint8_t LPF = lpf_odr4;
constexpr bool ONE_SHOT_MODE = true;
// LPF1 is useful for continuous sampling. AVG remains active in both modes.
constexpr bool ENABLE_LPF1 = !ONE_SHOT_MODE;

// Dynamic-stream FIFO is used only in continuous sensor mode. It keeps
// collecting after reads and does not require a bypass/reset between batches.
constexpr uint8_t FIFO_MODE = CONTINUOUS;
// Each FIFO data sample set is read as one 3-byte transaction. The first
// returned set is discarded as a repeatable FIFO-port priming artifact,
// leaving ten valid pressure samples for each reported average.
constexpr uint8_t FIFO_AVERAGE_SAMPLES = 10;
constexpr uint8_t FIFO_WATERMARK = FIFO_AVERAGE_SAMPLES + 1;
constexpr bool STOP_ON_WATERMARK = false;

// Startup pressure reference.
constexpr uint8_t BASELINE_SAMPLE_COUNT = 128;
// AN5699 Table 9: discard one ODR/4 sample or six ODR/9 samples
// whenever LPF1 is enabled or reset.
constexpr uint8_t LPF_SETTLING_SAMPLES =
    !ENABLE_LPF1 ? 0 : ((LPF == lpf_odr9) ? 6 : 1);
constexpr uint32_t CONVERSION_TIMEOUT_MS = 250;
constexpr uint32_t CONVERSION_CHECK_MS = 10;       // AVG=64 typical conversion is 5.4 ms.
constexpr uint32_t CONVERSION_RECHECK_MS = 2;
constexpr uint32_t ONE_SHOT_INTERVAL_MS = 60000;
constexpr uint32_t ERROR_RETRY_MS = 5000;

// CTRL_REG4 routing used by this sketch.
constexpr uint8_t INTERRUPTS_DISABLED = 0x00;
constexpr uint8_t INTERRUPT_FIFO_WATERMARK = 0x02;

enum class AppState : uint8_t {
  BASELINE_WAIT,
  RUN_CONTINUOUS,
  RUN_ONESHOT_IDLE,
  RUN_ONESHOT_WAIT,
  ERROR_RETRY
};

enum class ContinuousPhase : uint8_t {
  DISCARD_TRANSITION_BATCH,
  COLLECT_BASELINE_BATCH,
  REPORT_DATA
};

enum class ErrorCode : uint8_t {
  NONE,
  WHO_AM_I,
  RESET,
  CONFIGURE,
  POWER_DOWN,
  BASELINE_START,
  BASELINE_TIMEOUT,
  BASELINE_READ,
  FIFO_CONFIGURE,
  INTERRUPT_CONFIGURE,
  POWER_UP,
  FIFO_STATUS,
  FIFO_READ,
  FIFO_RESTART,
  ONESHOT_START,
  ONESHOT_TIMEOUT,
  ONESHOT_READ
};

I2Cdev i2c_0(&I2C_BUS);
LPS22DF barometer(&i2c_0);

volatile bool sensorInterrupt = false;
volatile bool conversionCheckDue = false;
volatile bool scheduledSampleDue = false;
volatile bool retryDue = false;

TimerMillis conversionTimer;
TimerMillis sampleTimer;
TimerMillis retryTimer;

AppState appState = AppState::ERROR_RETRY;
ErrorCode errorCode = ErrorCode::NONE;

uint32_t conversionStartedAt = 0;
uint8_t baselineSamples = 0;
uint8_t baselineSamplesDiscarded = 0;
double baselinePressureSum = 0.0;

float pressure_hPa = 0.0f;
float temperature_C = 0.0f;
float pressureBaseline_hPa = 0.0f;
float baselineAltitude_ft = 0.0f;
float relativeAltitude_ft = 0.0f;
bool baselineValid = false;
bool sampleTimerStarted = false;
bool diagnosticSummaryPrinted = false;
ContinuousPhase continuousPhase =
    ContinuousPhase::DISCARD_TRANSITION_BATCH;

float pressureToAltitudeFeet(float pressure, float referencePressure);
void setError(ErrorCode code);
bool startSensor();
bool startContinuousMode();
bool startBaselineConversion();
void serviceBaseline();
void finishBaseline();
bool startOneShotConversion();
void serviceOneShot();
void serviceContinuousFIFO();
void reportSample(const char *label, float pressure, float temperature);
void reportError(ErrorCode code);
void conversionTimerCallback();
void sampleTimerCallback();
void retryTimerCallback();
void LPS22DFIntHandler();


void setup()
{
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);                    // LED off.

  // The LPS22DF interrupt output is push-pull active-high, so no MCU pull
  // resistor is needed. Avoid internal pulls in the low-power configuration.
  pinMode(LPS22DF_INT_PIN, INPUT);
  attachInterrupt(LPS22DF_INT_PIN, LPS22DFIntHandler, RISING);

  if(SERIAL_DEBUG) {
    Serial.begin(115200);
    while (!Serial) {};
  }

  I2C_BUS.begin();
  I2C_BUS.setClock(400000);

  if(SERIAL_DEBUG) {
    Serial.println();
    Serial.println("LPS22DF stand-alone startup");
    Serial.println("Scanning I2C bus...");
    i2c_0.I2Cscan();
  }

  startSensor();
}


void loop()
{
  // A FIFO watermark is level-signaled. Checking the physical pin recovers
  // safely if an edge occurred immediately before STOP was entered.
  if(appState == AppState::RUN_CONTINUOUS &&
     digitalRead(LPS22DF_INT_PIN) == HIGH) {
    sensorInterrupt = true;
  }

  // LPS22DF FIFO watermark interrupt: clear the flag first, then drain the
  // dynamic-stream FIFO. The physical pin check above protects against a
  // lost edge immediately before STOP.
  if(sensorInterrupt) {
    sensorInterrupt = false;
    if(appState == AppState::RUN_CONTINUOUS) serviceContinuousFIFO();
  }

  // TimerMillis conversion check: baseline and one-shot conversions share the
  // timer, but their processing remains explicit here.
  if(conversionCheckDue) {
    conversionCheckDue = false;
    if(appState == AppState::BASELINE_WAIT) serviceBaseline();
    else if(appState == AppState::RUN_ONESHOT_WAIT) serviceOneShot();
  }

  // Periodic one-shot request. The callback sets only this flag and wakes the
  // MCU; the I2C transaction happens here in normal loop context.
  if(scheduledSampleDue) {
    scheduledSampleDue = false;
    if(appState == AppState::RUN_ONESHOT_IDLE) startOneShotConversion();
  }

  // Retry a failed sensor initialization without trapping the MCU forever.
  if(retryDue) {
    retryDue = false;
    if(appState == AppState::ERROR_RETRY) startSensor();
  }

  // TimerMillis callbacks and the sensor ISR wake the MCU and set flags.
  // No polling or sensor work is performed inside a callback.
  const bool fifoPinQuiet =
      appState != AppState::RUN_CONTINUOUS ||
      digitalRead(LPS22DF_INT_PIN) == LOW;
  if(!sensorInterrupt && !conversionCheckDue &&
     !scheduledSampleDue && !retryDue && fifoPinQuiet) STM32WB.stop();
}


bool startSensor()
{
  conversionTimer.stop();
  sampleTimer.stop();
  retryTimer.stop();
  digitalWrite(LED_PIN, LOW);                     // LED on during initialization.
  errorCode = ErrorCode::NONE;
  baselineValid = false;
  baselineSamples = 0;
  baselineSamplesDiscarded = 0;
  baselinePressureSum = 0.0;
  sensorInterrupt = false;
  conversionCheckDue = false;
  scheduledSampleDue = false;
  retryDue = false;
  sampleTimerStarted = false;
  diagnosticSummaryPrinted = false;
  continuousPhase = ContinuousPhase::DISCARD_TRANSITION_BATCH;

  uint8_t chipID = 0;
  if(!barometer.getChipID(&chipID) || chipID != 0xB4) {
    if(SERIAL_DEBUG) {
      Serial.print("WHO_AM_I = 0x");
      Serial.print(chipID, HEX);
      Serial.println(", expected 0xB4");
    }
    setError(ErrorCode::WHO_AM_I);
    return false;
  }
  if(SERIAL_DEBUG) {
    Serial.print("WHO_AM_I = 0x");
    Serial.print(chipID, HEX);
    Serial.println(" (correct)");
  }

  if(!barometer.reset()) {
    setError(ErrorCode::RESET);
    return false;
  }

  if(!barometer.Init(PODR, AVG, LPF, ENABLE_LPF1)) {
    setError(ErrorCode::CONFIGURE);
    return false;
  }

  if(!barometer.powerDown()) {
    setError(ErrorCode::POWER_DOWN);
    return false;
  }

  if(!barometer.configureInterrupts(INTERRUPTS_DISABLED)) {
    setError(ErrorCode::INTERRUPT_CONFIGURE);
    return false;
  }

  if(!barometer.configurationMatches(0, AVG, LPF, ENABLE_LPF1)) {
    setError(ErrorCode::CONFIGURE);
    return false;
  }

  if(SERIAL_DEBUG) Serial.println("Reset and configuration readback successful.");

  if(ONE_SHOT_MODE) {
    if(SERIAL_DEBUG) {
      Serial.print("Collecting ");
      Serial.print(BASELINE_SAMPLE_COUNT);
      Serial.println(" one-shot pressure baseline samples...");
    }
    return startBaselineConversion();
  }

  return startContinuousMode();
}


bool startContinuousMode()
{
  if(!barometer.initFIFO(FIFO_MODE, FIFO_WATERMARK,
                         STOP_ON_WATERMARK)) {
    setError(ErrorCode::FIFO_CONFIGURE);
    return false;
  }

  // FIFO watermark is the only sensor wake source in continuous mode.
  if(!barometer.configureInterrupts(INTERRUPT_FIFO_WATERMARK)) {
    setError(ErrorCode::INTERRUPT_CONFIGURE);
    return false;
  }

  if(!barometer.powerUp(PODR)) {
    setError(ErrorCode::POWER_UP);
    return false;
  }

  if(!barometer.configurationMatches(PODR, AVG, LPF, ENABLE_LPF1)) {
    setError(ErrorCode::CONFIGURE);
    return false;
  }

  continuousPhase = ContinuousPhase::DISCARD_TRANSITION_BATCH;
  appState = AppState::RUN_CONTINUOUS;

  if(SERIAL_DEBUG) {
    Serial.print("Operating mode = continuous, ODR code ");
    Serial.print(PODR);
    Serial.print(", AVG code ");
    Serial.print(AVG);
    Serial.print(", LPF1 ");
    Serial.print(ENABLE_LPF1 ? "enabled" : "disabled");
    Serial.print(", FIFO watermark = ");
    Serial.println(FIFO_WATERMARK);
    Serial.println("FIFO behavior = continuous dynamic-stream");
    Serial.println("Discarding first FIFO batch after the mode/ODR transition...");
  }

  return true;
}


bool startBaselineConversion()
{
  if(!barometer.oneShot()) {
    setError(ErrorCode::BASELINE_START);
    return false;
  }

  conversionStartedAt = millis();
  conversionCheckDue = false;
  if(!conversionTimer.start(conversionTimerCallback,
                            CONVERSION_CHECK_MS)) {
    setError(ErrorCode::BASELINE_START);
    return false;
  }
  appState = AppState::BASELINE_WAIT;
  return true;
}


void serviceBaseline()
{
  uint8_t status = 0;
  if(!barometer.status(&status)) {
    setError(ErrorCode::BASELINE_READ);
    return;
  }

  if((status & 0x03) == 0x03) {
    int32_t rawPressure = 0;
    int16_t rawTemperature = 0;
    if(!barometer.readSample(&rawPressure, &rawTemperature)) {
      setError(ErrorCode::BASELINE_READ);
      return;
    }

    pressure_hPa = (float)rawPressure / 4096.0f;
    temperature_C = (float)rawTemperature / 100.0f;

    if(baselineSamplesDiscarded < LPF_SETTLING_SAMPLES) {
      baselineSamplesDiscarded++;
      startBaselineConversion();
      return;
    }

    baselinePressureSum += pressure_hPa;
    baselineSamples++;

    if(baselineSamples >= BASELINE_SAMPLE_COUNT) finishBaseline();
    else startBaselineConversion();
    return;
  }

  if(millis() - conversionStartedAt >= CONVERSION_TIMEOUT_MS) {
    setError(ErrorCode::BASELINE_TIMEOUT);
  } else {
    if(!conversionTimer.start(conversionTimerCallback,
                              CONVERSION_RECHECK_MS)) {
      setError(ErrorCode::BASELINE_START);
    }
  }
}


void finishBaseline()
{
  pressureBaseline_hPa =
      (float)(baselinePressureSum / (double)baselineSamples);
  baselineAltitude_ft =
      pressureToAltitudeFeet(pressureBaseline_hPa, 1013.25f);
  relativeAltitude_ft = 0.0f;
  baselineValid = true;

  if(SERIAL_DEBUG) {
    Serial.print("Discarded LPF settling samples = ");
    Serial.println(baselineSamplesDiscarded);
    Serial.print("Accepted baseline samples = ");
    Serial.println(baselineSamples);
    Serial.print("Pressure baseline = ");
    Serial.print(pressureBaseline_hPa, 3);
    Serial.println(" hPa");
    Serial.print("Baseline pressure altitude = ");
    Serial.print(baselineAltitude_ft, 2);
    Serial.println(" ft");
  }

  if(!barometer.configureInterrupts(INTERRUPTS_DISABLED)) {
    setError(ErrorCode::INTERRUPT_CONFIGURE);
    return;
  }

  digitalWrite(LED_PIN, HIGH);                     // Initialization complete.
  if(SERIAL_DEBUG) {
    Serial.println("Initialization successful.");
    Serial.println("Operating mode = one-shot, one sample per minute");
    Serial.print("LPF1 = ");
    Serial.println(ENABLE_LPF1 ? "enabled" : "disabled");
    Serial.println();
  }
  startOneShotConversion();                        // Capture an initial sample.
}


bool startOneShotConversion()
{
  if(!barometer.oneShot()) {
    setError(ErrorCode::ONESHOT_START);
    return false;
  }

  conversionStartedAt = millis();
  conversionCheckDue = false;
  if(!conversionTimer.start(conversionTimerCallback,
                            CONVERSION_CHECK_MS)) {
    setError(ErrorCode::ONESHOT_START);
    return false;
  }
  appState = AppState::RUN_ONESHOT_WAIT;
  return true;
}


void serviceOneShot()
{
  uint8_t status = 0;
  if(!barometer.status(&status)) {
    setError(ErrorCode::ONESHOT_READ);
    return;
  }

  if((status & 0x03) == 0x03) {
    int32_t rawPressure = 0;
    int16_t rawTemperature = 0;
    if(!barometer.readSample(&rawPressure, &rawTemperature)) {
      setError(ErrorCode::ONESHOT_READ);
      return;
    }

    pressure_hPa = (float)rawPressure / 4096.0f;
    temperature_C = (float)rawTemperature / 100.0f;
    relativeAltitude_ft =
        pressureToAltitudeFeet(pressure_hPa, pressureBaseline_hPa);
    reportSample("One-shot", pressure_hPa, temperature_C);
    appState = AppState::RUN_ONESHOT_IDLE;
    if(!sampleTimerStarted) {
      sampleTimerStarted = true;
      if(!sampleTimer.start(sampleTimerCallback, ONE_SHOT_INTERVAL_MS,
                            ONE_SHOT_INTERVAL_MS)) {
        setError(ErrorCode::ONESHOT_START);
      }
    }
    return;
  }

  if(millis() - conversionStartedAt >= CONVERSION_TIMEOUT_MS) {
    setError(ErrorCode::ONESHOT_TIMEOUT);
  } else {
    if(!conversionTimer.start(conversionTimerCallback,
                              CONVERSION_RECHECK_MS)) {
      setError(ErrorCode::ONESHOT_START);
    }
  }
}


void serviceContinuousFIFO()
{
  uint8_t fifoStatus[2] = {0, 0};
  if(!barometer.FIFOStatus(fifoStatus)) {
    setError(ErrorCode::FIFO_STATUS);
    return;
  }

  const uint8_t fifoLevel = fifoStatus[0];
  const bool watermarkAsserted = (fifoStatus[1] & 0x80) != 0;
  if(!watermarkAsserted || fifoLevel == 0) return;

  int32_t fifoRaw[FIFO_WATERMARK] = {0};

  if(fifoLevel > FIFO_WATERMARK) {
    setError(ErrorCode::FIFO_READ);
    return;
  }

  for(uint8_t sample = 0; sample < fifoLevel; sample++) {
    if(!barometer.FIFOPressure(&fifoRaw[sample])) {
      setError(ErrorCode::FIFO_READ);
      return;
    }
  }

  // The first FIFO word returned after each watermark wake is repeatably about
  // 32 hPa high on this device, while all following words agree with PRESS_OUT.
  // Treat the first access as an invalid FIFO-port priming read.
  if(fifoLevel < 2) {
    setError(ErrorCode::FIFO_READ);
    return;
  }

  int64_t pressureSum = 0;
  const uint8_t firstValidSample = 1;
  const uint8_t samplesRead = fifoLevel - firstValidSample;
  if(samplesRead != FIFO_AVERAGE_SAMPLES) {
    setError(ErrorCode::FIFO_READ);
    return;
  }

  for(uint8_t sample = firstValidSample; sample < fifoLevel; sample++) {
    pressureSum += fifoRaw[sample];
  }

  // FIFO contains pressure only. Read the latest temperature separately.
  int16_t rawTemperature = 0;
  if(!barometer.Temperature(&rawTemperature)) {
    setError(ErrorCode::FIFO_READ);
    return;
  }

  if(samplesRead == 0) return;

  pressure_hPa =
      ((float)pressureSum / (float)samplesRead) / 4096.0f;
  temperature_C = (float)rawTemperature / 100.0f;

  if(SERIAL_DEBUG && ENABLE_DIAGNOSTICS) {
    // Compare PRESS_OUT with the FIFO data from the same wake cycle. This
    // diagnostic transaction is omitted completely in a clean run.
    int32_t standardRawPressure = 0;
    if(!barometer.Pressure(&standardRawPressure)) {
      setError(ErrorCode::FIFO_READ);
      return;
    }

    Serial.println("FIFO diagnostic:");
    for(uint8_t sample = 0;
        sample < fifoLevel && sample < FIFO_WATERMARK;
        sample++) {
      Serial.print("  FIFO[");
      Serial.print(sample);
      if(sample == 0) Serial.print("] discarded, raw = ");
      else Serial.print("] valid, raw = ");
      Serial.print(fifoRaw[sample]);
      Serial.print(", pressure = ");
      Serial.print((float)fifoRaw[sample] / 4096.0f, 3);
      Serial.println(" hPa");
    }
    Serial.print("  FIFO average = ");
    Serial.print(pressure_hPa, 3);
    Serial.println(" hPa");
    Serial.print("  Standard output raw = ");
    Serial.print(standardRawPressure);
    Serial.print(", pressure = ");
    Serial.print((float)standardRawPressure / 4096.0f, 3);
    Serial.println(" hPa");
    Serial.print("  Standard minus FIFO average = ");
    Serial.print(((float)standardRawPressure / 4096.0f) -
                 pressure_hPa, 3);
    Serial.println(" hPa");
  }
  if(continuousPhase == ContinuousPhase::DISCARD_TRANSITION_BATCH) {
    continuousPhase = ContinuousPhase::COLLECT_BASELINE_BATCH;
    if(SERIAL_DEBUG) {
      Serial.print("Discarded transitional FIFO average = ");
      Serial.print(pressure_hPa, 3);
      Serial.println(" hPa");
      Serial.println("Collecting continuous-mode baseline batch...");
    }
    return;
  }

  if(continuousPhase == ContinuousPhase::COLLECT_BASELINE_BATCH) {
    pressureBaseline_hPa = pressure_hPa;
    baselineAltitude_ft =
        pressureToAltitudeFeet(pressureBaseline_hPa, 1013.25f);
    relativeAltitude_ft = 0.0f;
    baselineValid = true;
    continuousPhase = ContinuousPhase::REPORT_DATA;
    digitalWrite(LED_PIN, HIGH);                   // Initialization complete.

    if(SERIAL_DEBUG) {
      Serial.print("Accepted continuous baseline samples = ");
      Serial.println(samplesRead);
      Serial.print("Pressure baseline = ");
      Serial.print(pressureBaseline_hPa, 3);
      Serial.println(" hPa");
      Serial.print("Baseline pressure altitude = ");
      Serial.print(baselineAltitude_ft, 2);
      Serial.println(" ft");
      Serial.println("Initialization successful.");
      Serial.println();
    }
    return;
  }

  relativeAltitude_ft =
      pressureToAltitudeFeet(pressure_hPa, pressureBaseline_hPa);
  reportSample("FIFO average", pressure_hPa, temperature_C);
}


void setError(ErrorCode code)
{
  conversionTimer.stop();
  sampleTimer.stop();
  retryTimer.stop();
  errorCode = code;
  appState = AppState::ERROR_RETRY;
  sensorInterrupt = false;
  conversionCheckDue = false;
  scheduledSampleDue = false;
  retryDue = false;
  sampleTimerStarted = false;
  barometer.configureInterrupts(INTERRUPTS_DISABLED);
  barometer.powerDown();
  digitalWrite(LED_PIN, LOW);                     // Solid red until retry succeeds.
  reportError(code);
  retryTimer.start(retryTimerCallback, ERROR_RETRY_MS);
}


void reportSample(const char *label, float pressure, float temperature)
{
  if(!SERIAL_DEBUG) return;

  if(!diagnosticSummaryPrinted) {
    diagnosticSummaryPrinted = true;
    Serial.println("First operational report:");
    Serial.println("WHO_AM_I verified and initialization completed.");
    Serial.print("Stored pressure baseline = ");
    Serial.print(pressureBaseline_hPa, 3);
    Serial.println(" hPa");
    Serial.print("Current minus baseline = ");
    Serial.print(pressure - pressureBaseline_hPa, 3);
    Serial.println(" hPa");
  }

  Serial.print(label);
  Serial.print(": ");
  Serial.print(pressure, 3);
  Serial.print(" hPa, ");
  Serial.print(temperature, 2);
  Serial.print(" C");
  if(baselineValid) {
    Serial.print(", relative altitude ");
    Serial.print(relativeAltitude_ft, 2);
    Serial.print(" ft");
  }
  Serial.println();
}


void reportError(ErrorCode code)
{
  if(!SERIAL_DEBUG) return;
  Serial.print("LPS22DF error code ");
  Serial.println((uint8_t)code);
}


float pressureToAltitudeFeet(float pressure, float referencePressure)
{
  return 145366.45f *
      (1.0f - powf(pressure / referencePressure, 0.190284f));
}


void conversionTimerCallback()
{
  conversionCheckDue = true;
  STM32WB.wakeup();
}


void sampleTimerCallback()
{
  scheduledSampleDue = true;
  STM32WB.wakeup();
}


void retryTimerCallback()
{
  retryDue = true;
  STM32WB.wakeup();
}


void LPS22DFIntHandler()
{
  sensorInterrupt = true;
  STM32WB.wakeup();
}
