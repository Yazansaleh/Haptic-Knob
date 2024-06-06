#include <Wire.h>
#include <Adafruit_DRV2605.h>
#include <FastLED.h>

// Input/Output declarations
#define POT_A_INPUT A0           // Analog input 1
#define POT_B_INPUT A1           // Analog input 2
#define LED_PIN 12               // Pin to which the LED ring is connected
#define NUM_LEDS 24              // Number of LEDs in the ring

// Constant value definitions
#define ADC_HYSTERESIS  24       // Base hysteresis value
#define MAX_POT_VALUE  127
#define ADC_MAX_VALUE 4095       // Adjusted for 12-bit ADC

// Variables for potentiometer
int ValuePotA = 0;            // Pot1 tap A value
int ValuePotB = 0;            // Pot1 tap B value
int PreviousValuePotA = 0;    // Used to remember last value to determine turn direction
int PreviousValuePotB = 0;    // Used to remember last value to determine turn direction
int DirPotA = 1;              // Direction for Pot 1 tap A
int DirPotB = 1;              // Direction for Pot1 tap B

int Direction  = 1;           // Final CALCULATED direction
int Value = 0;                // Final CALCULATED value
int LastValue = 0;            // Previous value to detect changes
int LastNumLedsToLight = 0;   // Previous number of LEDs lit

Adafruit_DRV2605 drv;
CRGB leds[NUM_LEDS];

// Queue handle for communicating between tasks
QueueHandle_t hapticQueue;

// Constants for ERP algorithm
#define ERP_HIGH_PEAK    (66688 / 16)
#define ERP_LOW_PEAK     (-1280 / 16)
#define ERP_MIDPOINT     ((ERP_HIGH_PEAK + ERP_LOW_PEAK) / 2)
#define ERP_RANGE        (ERP_HIGH_PEAK - ERP_LOW_PEAK)
#define ADC_BITWIDTH     12
#define ADC_RANGE        (1 << ADC_BITWIDTH)

#define ERP_0_DEG        (0)
#define ERP_90_DEG       (ERP_360_DEG / 4)
#define ERP_180_DEG      (ERP_360_DEG / 2)
#define ERP_270_DEG      (ERP_180_DEG + ERP_90_DEG)
#define ERP_360_DEG      (ERP_RANGE * 2)

#define DEAD_ZONE_WIDTH  (ADC_RANGE / 32)
#define DEAD_ZONE_BOTTOM (DEAD_ZONE_WIDTH)
#define DEAD_ZONE_TOP    (ADC_RANGE - DEAD_ZONE_WIDTH)
#define DERATING_ZONE_WIDTH  (ADC_RANGE / 32)
#define DERATING_ZONE_BOTTOM (DEAD_ZONE_BOTTOM + DERATING_ZONE_WIDTH)
#define DERATING_ZONE_TOP    (DEAD_ZONE_TOP - DERATING_ZONE_WIDTH)

#define WEIGHT_MAX (1 << 8)
#define WEIGHT_DEFAULT (WEIGHT_MAX / 2)

#define ERP_OUTPUT_RANGE 1000
#define TWO_POWER_32 (4294967296)
#define ERP_OUTPUT_FACTOR (ERP_OUTPUT_RANGE * TWO_POWER_32 / ERP_360_DEG)
#define ERP_OUTPUT_BITSHIFT (32)

#define GET_ANGLE_LEADING_QUADRANT_1(uRaw) (((uRaw) - ERP_LOW_PEAK) - ERP_90_DEG)
#define GET_ANGLE_TRAILING_QUADRANT_1(uRaw) ((uRaw) - ERP_LOW_PEAK)
#define GET_ANGLE_LEADING_QUADRANT_2(uRaw) (ERP_90_DEG + ERP_RANGE - ((uRaw) - ERP_LOW_PEAK))
#define GET_ANGLE_TRAILING_QUADRANT_2(uRaw) GET_ANGLE_TRAILING_QUADRANT_1(uRaw)
#define GET_ANGLE_LEADING_QUADRANT_3(uRaw) GET_ANGLE_LEADING_QUADRANT_2(uRaw)
#define GET_ANGLE_TRAILING_QUADRANT_3(uRaw) (ERP_180_DEG + ERP_RANGE - ((uRaw) - ERP_LOW_PEAK))
#define GET_ANGLE_LEADING_QUADRANT_4(uRaw) (((uRaw) - ERP_LOW_PEAK) + ERP_270_DEG)
#define GET_ANGLE_TRAILING_QUADRANT_4(uRaw) GET_ANGLE_TRAILING_QUADRANT_3(uRaw)

static_assert(GET_ANGLE_LEADING_QUADRANT_1(ERP_MIDPOINT) == ERP_0_DEG);
static_assert(GET_ANGLE_TRAILING_QUADRANT_1(ERP_LOW_PEAK) == ERP_0_DEG);
static_assert(GET_ANGLE_LEADING_QUADRANT_1(ERP_HIGH_PEAK) == ERP_90_DEG);
static_assert(GET_ANGLE_LEADING_QUADRANT_2(ERP_HIGH_PEAK) == ERP_90_DEG);
static_assert(GET_ANGLE_TRAILING_QUADRANT_1(ERP_MIDPOINT) == ERP_90_DEG);
static_assert(GET_ANGLE_TRAILING_QUADRANT_2(ERP_MIDPOINT) == ERP_90_DEG);
static_assert(GET_ANGLE_LEADING_QUADRANT_2(ERP_MIDPOINT) == ERP_180_DEG);
static_assert(GET_ANGLE_LEADING_QUADRANT_3(ERP_MIDPOINT) == ERP_180_DEG);
static_assert(GET_ANGLE_TRAILING_QUADRANT_2(ERP_HIGH_PEAK) == ERP_180_DEG);
static_assert(GET_ANGLE_TRAILING_QUADRANT_3(ERP_HIGH_PEAK) == ERP_180_DEG);
static_assert(GET_ANGLE_LEADING_QUADRANT_3(ERP_LOW_PEAK) == ERP_270_DEG);
static_assert(GET_ANGLE_LEADING_QUADRANT_4(ERP_LOW_PEAK) == ERP_270_DEG);
static_assert(GET_ANGLE_TRAILING_QUADRANT_3(ERP_MIDPOINT) == ERP_270_DEG);
static_assert(GET_ANGLE_TRAILING_QUADRANT_4(ERP_MIDPOINT) == ERP_270_DEG);
static_assert(GET_ANGLE_LEADING_QUADRANT_4(ERP_MIDPOINT) == ERP_360_DEG);
static_assert(GET_ANGLE_TRAILING_QUADRANT_4(ERP_LOW_PEAK) == ERP_360_DEG);

void setup() {
  analogReadResolution(12);  // Set ADC resolution to 12 bits
  Serial.begin(115200);
  FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, NUM_LEDS);
  FastLED.clear();
  FastLED.show();
  
  if (!drv.begin()) {
    Serial.println("DRV2605L not found");
    while (1);
  }
  Serial.println("DRV2605L found");

  drv.selectLibrary(1);
  drv.setMode(DRV2605_MODE_INTTRIG); // internal trigger when sending GO command

  PreviousValuePotA  = analogRead(POT_A_INPUT);   // Initialize Pot1 tap A value
  PreviousValuePotB  = analogRead(POT_B_INPUT);   // Initialize Pot1 tap B value
  Serial.println("Setup complete");

  // Create a queue capable of holding 10 values and directions
  hapticQueue = xQueueCreate(10, sizeof(int) * 2);

  // Create a task for reading potentiometer values on core NULL with higher priority
  xTaskCreatePinnedToCore(
    readPotentiometerTask,   // Task function
    "ReadPotTask",           // Task name
    4096,                    // Stack size
    NULL,                    // Task input parameter
    3,                       // Priority
    NULL,                    // Task handle
    NULL                        // Core
  );

  // Create a task for haptic feedback on core NULL
  xTaskCreatePinnedToCore(
    hapticFeedbackTask,      // Task function
    "HapticTask",            // Task name
    4096,                    // Stack size
    NULL,                    // Task input parameter
    2,                       // Priority
    NULL,                    // Task handle
    NULL                     // Core
  );

  // Create a task for LED ring update on core NULL
  xTaskCreatePinnedToCore(
    ledUpdateTask,           // Task function
    "LedUpdateTask",         // Task name
    4096,                    // Stack size
    NULL,                    // Task input parameter
    1,                       // Priority
    NULL,                    // Task handle
    NULL                        // Core
  );
}

void loop() {
  // Nothing to do here, everything is handled in tasks
}

void readPotentiometerTask(void *pvParameters) {
  const int numSamples = 5;  // Number of samples for averaging
  int samplesA[numSamples];
  int samplesB[numSamples];
  int sampleIndex = 0;

  while (1) {
    // Take multiple samples
    for (int i = 0; i < numSamples; i++) {
      samplesA[i] = analogRead(POT_A_INPUT);
      samplesB[i] = analogRead(POT_B_INPUT);
      vTaskDelay(pdMS_TO_TICKS(1));  // 1kHz sampling frequency
    }

    // Calculate the average value
    ValuePotA = 0;
    ValuePotB = 0;
    for (int i = 0; i < numSamples; i++) {
      ValuePotA += samplesA[i];
      ValuePotB += samplesB[i];
    }
    ValuePotA /= numSamples;
    ValuePotB /= numSamples;

    uint32_t decodedPosition = ERP_vDecodePosition(ValuePotA, ValuePotB);
    Value = constrain(map(decodedPosition, 0, ERP_OUTPUT_RANGE, 0, MAX_POT_VALUE + 1), 0, MAX_POT_VALUE);

    Serial.print("ValuePotA: "); Serial.print(ValuePotA);
    Serial.print(" ValuePotB: "); Serial.print(ValuePotB);
    Serial.print(" Decoded Position: "); Serial.print(decodedPosition);
    Serial.print(" Value: "); Serial.println(Value);

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void hapticFeedbackTask(void *pvParameters) {
  int message[2];
  while (1) {
    if (xQueueReceive(hapticQueue, &message, portMAX_DELAY)) {
      int receivedValue = message[0];
      int receivedDirection = message[1];
      drv.setWaveform(0, receivedDirection == 1 ? 5 : 6);  // Play effect on hapticc motor for increasing or decreasing value
      drv.go();
      vTaskDelay(pdMS_TO_TICKS(5));  // Small delay to allow the effect to be felt properly
    }
  }
}

void ledUpdateTask(void *pvParameters) {
  while (1) {
    int numLedsToLight = constrain(map(Value, 0, MAX_POT_VALUE, 0, NUM_LEDS), 0, NUM_LEDS - 1);

    if (numLedsToLight != LastNumLedsToLight) {
      for (int i = 0; i < NUM_LEDS; i++) {
        leds[i] = CRGB::Black;
      }

      // Turn on the appropriate number of LEDs
      for (int i = 0; i <= numLedsToLight; i++) {
        leds[i] = CRGB::Green;
      }

      FastLED.show();

      // Send haptic feedback
      int direction = (numLedsToLight > LastNumLedsToLight) ? 1 : -1;
      int message[2] = {numLedsToLight, direction};
      xQueueSend(hapticQueue, &message, portMAX_DELAY);

      LastNumLedsToLight = numLedsToLight;
    }

    vTaskDelay(pdMS_TO_TICKS(20)); // Update LEDs every 20ms
  }
}

// ERP algorithms 
uint32_t ERP_GET_OUTPUT(int32_t iAngle) {
  return (uint32_t)(((uint64_t)iAngle * ERP_OUTPUT_FACTOR) >> ERP_OUTPUT_BITSHIFT);
}

void vGetWeights(uint32_t uRawLeading, uint32_t uRawTrailing, uint32_t* puWeightLeading, uint32_t* puWeightTrailing) {
  if ((uRawLeading > DEAD_ZONE_TOP) || (uRawLeading < DEAD_ZONE_BOTTOM)) {
    *puWeightLeading = 0;
    *puWeightTrailing = WEIGHT_MAX;
  } else if ((uRawTrailing > DEAD_ZONE_TOP) || (uRawTrailing < DEAD_ZONE_BOTTOM)) {
    *puWeightLeading = WEIGHT_MAX;
    *puWeightTrailing = 0;
  } else {
    if (uRawLeading > DERATING_ZONE_TOP) {
      *puWeightLeading = (WEIGHT_DEFAULT * (DEAD_ZONE_TOP - uRawLeading)) / DERATING_ZONE_WIDTH;
      *puWeightTrailing = WEIGHT_MAX - *puWeightLeading;
    } else if (uRawLeading < DERATING_ZONE_BOTTOM) {
      *puWeightLeading = (WEIGHT_DEFAULT * (uRawLeading - DEAD_ZONE_BOTTOM)) / DERATING_ZONE_WIDTH;
      *puWeightTrailing = WEIGHT_MAX - *puWeightLeading;
    } else if (uRawTrailing > DERATING_ZONE_TOP) {
      *puWeightTrailing = (WEIGHT_DEFAULT * (DEAD_ZONE_TOP - uRawTrailing)) / DERATING_ZONE_WIDTH;
      *puWeightLeading = WEIGHT_MAX - *puWeightTrailing;
    } else if (uRawTrailing < DERATING_ZONE_BOTTOM) {
      *puWeightTrailing = (WEIGHT_DEFAULT * (uRawTrailing - DEAD_ZONE_BOTTOM)) / DERATING_ZONE_WIDTH;
      *puWeightLeading = WEIGHT_MAX - *puWeightTrailing;
    } else {
      *puWeightLeading = WEIGHT_DEFAULT;
      *puWeightTrailing = WEIGHT_DEFAULT;
    }
  }
}

uint32_t ERP_vDecodePosition(uint32_t uRawLeading, uint32_t uRawTrailing) {
  int32_t iAngleLeading;
  int32_t iAngleTrailing;

  if (uRawLeading >= ERP_MIDPOINT) {
    if (uRawTrailing < ERP_MIDPOINT) {
      iAngleLeading = GET_ANGLE_LEADING_QUADRANT_1(uRawLeading);
      iAngleTrailing = GET_ANGLE_TRAILING_QUADRANT_1(uRawTrailing);
    } else {
      iAngleLeading = GET_ANGLE_LEADING_QUADRANT_2(uRawLeading);
      iAngleTrailing = GET_ANGLE_TRAILING_QUADRANT_2(uRawTrailing);
    }
  } else {
    if (uRawTrailing > ERP_MIDPOINT) {
      iAngleLeading = GET_ANGLE_LEADING_QUADRANT_3(uRawLeading);
      iAngleTrailing = GET_ANGLE_TRAILING_QUADRANT_3(uRawTrailing);
    } else {
      iAngleLeading = GET_ANGLE_LEADING_QUADRANT_4(uRawLeading);
      iAngleTrailing = GET_ANGLE_TRAILING_QUADRANT_4(uRawTrailing);
    }
  }

  uint32_t uWeightLeading, uWeightTrailing;
  vGetWeights(uRawLeading, uRawTrailing, &uWeightLeading, &uWeightTrailing);

  iAngleLeading *= uWeightLeading;
  iAngleTrailing *= uWeightTrailing;

  int32_t iAngleWeighted = iAngleLeading + iAngleTrailing;
  iAngleWeighted /= WEIGHT_MAX;

  return ERP_GET_OUTPUT(iAngleWeighted);
}
