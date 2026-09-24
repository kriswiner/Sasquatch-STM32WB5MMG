#include <PDM.h>
 
#define ARM_MATH_CM4
#include <arm_math.h>

#define SAMPLE_RATE_HZ 16000
#define FFT_N          512
#define NUM_BANDS      32
#define DUMP_PERIOD_MS 2000

static int16_t sampleBuffer[FFT_N];
static uint16_t sampleCount = 0;

static float32_t fftInput[FFT_N];
static float32_t fftOutput[FFT_N];
static float32_t magnitudes[FFT_N / 2];

static arm_rfft_fast_instance_f32 fft;

void setup()
{
    Serial.begin(115200);
    while (!Serial) { }

    delay(1000);

    PDM.setGain(0.0);

    if (!PDM.begin(1, SAMPLE_RATE_HZ)) {
        Serial.println("PDM failed");
        while (1) { }
    }

    if (arm_rfft_fast_init_f32(&fft, FFT_N) != ARM_MATH_SUCCESS) {
        Serial.println("FFT init failed");
        while (1) { }
    }

    Serial.println("PDM FFT ready");
    Serial.print("Sample rate: ");
    Serial.print(SAMPLE_RATE_HZ);
    Serial.println(" Hz");
    Serial.print("FFT size: ");
    Serial.println(FFT_N);
    Serial.println();
}

void loop()
{
    static uint32_t lastDump = 0;
    static int16_t pdmBlock[PDM_BUFFER_SIZE / 2];

    if (PDM.available() == PDM_BUFFER_SIZE) {
        int nbytes = PDM.read(pdmBlock, sizeof(pdmBlock));
        int nsamples = nbytes / sizeof(int16_t);

        for (int i = 0; i < nsamples && sampleCount < FFT_N; i++) {
            sampleBuffer[sampleCount++] = pdmBlock[i];
        }
    }

    if (sampleCount >= FFT_N && (millis() - lastDump) >= DUMP_PERIOD_MS) {
        lastDump = millis();
        sampleCount = 0;

        runAndPrintFFT();
    }
}

void runAndPrintFFT()
{
    float32_t mean = 0.0f;
    float32_t rms = 0.0f;

    for (int i = 0; i < FFT_N; i++) {
        mean += sampleBuffer[i];
    }
    mean /= FFT_N;

    for (int i = 0; i < FFT_N; i++) {
        float32_t x = (float32_t)sampleBuffer[i] - mean;

        // Hann window
        float32_t w = 0.5f - 0.5f * cosf((2.0f * PI * i) / (FFT_N - 1));
        fftInput[i] = x * w;

        rms += x * x;
    }

    rms = sqrtf(rms / FFT_N);

    arm_rfft_fast_f32(&fft, fftInput, fftOutput, 0);

    magnitudes[0] = fabsf(fftOutput[0]);
    magnitudes[FFT_N / 2 - 1] = fabsf(fftOutput[1]);
    arm_cmplx_mag_f32(&fftOutput[2], &magnitudes[1], (FFT_N / 2) - 1);

    Serial.println();
    Serial.println("Frequency_Hz\tAmplitude");
    Serial.print("RMS\t");
    Serial.println(rms, 2);

    float32_t peakAmp = 0.0f;
    float32_t peakFreq = 0.0f;

    for (int b = 0; b < NUM_BANDS; b++) {
        int startBin = 1 + b * ((FFT_N / 2 - 1) / NUM_BANDS);
        int endBin   = 1 + (b + 1) * ((FFT_N / 2 - 1) / NUM_BANDS);

        float32_t sum = 0.0f;

        for (int k = startBin; k < endBin; k++) {
            sum += magnitudes[k];
        }

        float32_t avg = sum / (endBin - startBin);
        float32_t centerFreq = ((startBin + endBin - 1) * 0.5f * SAMPLE_RATE_HZ) / FFT_N;

        if (avg > peakAmp) {
            peakAmp = avg;
            peakFreq = centerFreq;
        }

        Serial.print(centerFreq, 1);
        Serial.print('\t');
        Serial.println(avg, 1);
    }

    Serial.print("Peak_Hz\t");
    Serial.println(peakFreq, 1);
    Serial.println("----");
}
