/* Edge Impulse ingestion SDK
   Copyright (c) 2022 EdgeImpulse Inc.

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at
   http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.

*/

// If your target is limited in memory remove this macro to save 10K RAM
#define EIDSP_QUANTIZE_FILTERBANK   0

/*
 ** NOTE: If you run into TFLite arena allocation issue.
 **
 ** This may be due to may dynamic memory fragmentation.
 ** Try defining "-DEI_CLASSIFIER_ALLOCATION_STATIC" in boards.local.txt (create
 ** if it doesn't exist) and copy this file to
 ** `<ARDUINO_CORE_INSTALL_PATH>/arduino/hardware/<mbed_core>/<core_version>/`.
 **
 ** See
 ** (https://support.arduino.cc/hc/en-us/articles/360012076960-Where-are-the-installed-cores-located-)
 ** to find where Arduino installs cores on your machine.
 **
 ** If the problem persists then there's not enough memory for this model and application.
*/

/* Includes ---------------------------------------------------------------- */
#include <Audio_Pico_Badge_inferencing.h>
#include <Stepper.h>
#include <PDM.h>
#include <bsec2.h>

/* Macros used */
#define PANIC_LED   LED_BUILTIN
#define ERROR_DUR   1000

/** Audio buffers, pointers and selectors */
typedef struct {
  int16_t *buffer;
  uint8_t buf_ready;
  uint32_t buf_count;
  uint32_t n_samples;
} inference_t;

static inference_t inference;
static signed short sampleBuffer[2048];
static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal
static volatile bool record_ready = false;
long current_time = 0;
Stepper stepper(630, 19, 18, 20, 21);

/* Helper functions declarations */
/**
   @brief : This function toggles the led when a fault was detected
*/
void errLeds(void);

/**
   @brief : This function checks the BSEC status, prints the respective error code. Halts in case of error
   @param[in] bsec  : Bsec2 class object
*/
void checkBsecStatus(Bsec2 bsec);

/**
   @brief : This function is called by the BSEC library when a new output is available
   @param[in] input     : BME68X sensor data before processing
   @param[in] outputs   : Processed BSEC BSEC output data
   @param[in] bsec      : Instance of BSEC2 calling the callback
*/
void newDataCallback(const bme68xData data, const bsecOutputs outputs, Bsec2 bsec);

/* Create an object of the class Bsec2 */
Bsec2 envSensor;

/**
   @brief      Arduino setup function
*/
void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  /* Desired subscription list of BSEC2 outputs */
  bsecSensor sensorList[] = {
    BSEC_OUTPUT_IAQ,
    BSEC_OUTPUT_RAW_TEMPERATURE,
    BSEC_OUTPUT_RAW_PRESSURE,
    BSEC_OUTPUT_RAW_HUMIDITY,
    BSEC_OUTPUT_RAW_GAS,
    BSEC_OUTPUT_STABILIZATION_STATUS,
    BSEC_OUTPUT_RUN_IN_STATUS
  };
  current_time = millis();
  // comment out the below line to cancel the wait for USB connection (needed for native USB)
  while (!Serial) {
    if ((millis() - current_time) > 5000) {
      break;
    }
  }
  Serial.println("Edge Impulse Inferencing Demo");
  Wire.begin();
  pinMode(PANIC_LED, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(22, OUTPUT);
  pinMode(23, OUTPUT);
  pinMode(24, OUTPUT);

  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(22, HIGH);
  digitalWrite(23, HIGH);
  digitalWrite(24, HIGH);

  stepper.setSpeed(60);
  stepper.step(630);
  stepper.step(-315);
  delay(5000);

  /* Initialize the library and interfaces */
  if (!envSensor.begin(BME68X_I2C_ADDR_LOW, Wire))
  {
    checkBsecStatus(envSensor);
  }

  /* Subsribe to the desired BSEC2 outputs */
  if (!envSensor.updateSubscription(sensorList, ARRAY_LEN(sensorList), BSEC_SAMPLE_RATE_LP))
  {
    checkBsecStatus(envSensor);
  }

  /* Whenever new data is available call the newDataCallback function */
  envSensor.attachCallback(newDataCallback);

  Serial.println("BSEC library version " + \
                 String(envSensor.version.major) + "." \
                 + String(envSensor.version.minor) + "." \
                 + String(envSensor.version.major_bugfix) + "." \
                 + String(envSensor.version.minor_bugfix));

  // summary of inferencing settings (from model_metadata.h)
  ei_printf("Inferencing settings:\n");
  ei_printf("\tInterval: ");
  ei_printf_float((float)EI_CLASSIFIER_INTERVAL_MS);
  ei_printf(" ms.\n");
  ei_printf("\tFrame size: %d\n", EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
  ei_printf("\tSample length: %d ms.\n", EI_CLASSIFIER_RAW_SAMPLE_COUNT / 16);
  ei_printf("\tNo. of classes: %d\n", sizeof(ei_classifier_inferencing_categories) / sizeof(ei_classifier_inferencing_categories[0]));

  if (microphone_inference_start(EI_CLASSIFIER_RAW_SAMPLE_COUNT) == false) {
    ei_printf("ERR: Could not allocate audio buffer (size %d), this could be due to the window length of your model\r\n", EI_CLASSIFIER_RAW_SAMPLE_COUNT);
    return;
  }


}

/**
   @brief      Arduino main function. Runs the inferencing loop.
*/
int consecutive_count = 0;
void loop()
{

  if (!envSensor.run())
  {
    checkBsecStatus(envSensor);
  }
  ei_printf("Starting inferencing in 2 seconds...\n");

  delay(2000);

  ei_printf("Recording...\n");

  bool m = microphone_inference_record();
  if (!m) {
    ei_printf("ERR: Failed to record audio...\n");
    return;
  }

  ei_printf("Recording done\n");

  signal_t signal;
  signal.total_length = EI_CLASSIFIER_RAW_SAMPLE_COUNT;
  signal.get_data = &microphone_audio_signal_get_data;
  ei_impulse_result_t result = { 0 };

  EI_IMPULSE_ERROR r = run_classifier(&signal, &result, debug_nn);
  if (r != EI_IMPULSE_OK) {
    ei_printf("ERR: Failed to run classifier (%d)\n", r);
    return;
  }

  // print inference return code
  ei_printf("run_classifier returned: %d\r\n", r);
  for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {

    if (result.classification[i].value > 0.9) {
      ei_printf("Sound Detected: %s", ei_classifier_inferencing_categories[i]);
      consecutive_count++;
      if (!envSensor.run())
      {
        checkBsecStatus(envSensor);
      }
      if (consecutive_count >= 3) {
        consecutive_count = 0;
        int steps = 0;
        switch (i) {
          case 0:
            steps = 65;
            break;
          case 1:
            steps = 252;
            break;
          case 2:
            steps = 165;
            break;
        }

        stepper.step(-steps);
        delay(5000);
        stepper.step(steps);

      }
    }
  }

}

/**
   @brief      PDM buffer full callback
               Copy audio data to app buffers
*/
static void pdm_data_ready_inference_callback(void)
{
  int bytesAvailable = PDM.available();

  // read into the sample buffer
  int bytesRead = PDM.read((char *)&sampleBuffer[0], bytesAvailable);

  if ((inference.buf_ready == 0) && (record_ready == true)) {

    for (int i = 0; i < bytesRead >> 1; i++) {
      inference.buffer[inference.buf_count++] = sampleBuffer[i];

      if (inference.buf_count >= inference.n_samples) {
        inference.buf_count = 0;
        inference.buf_ready = 1;
        break;
      }
    }
  }
}

/**
   @brief      Init inferencing struct and setup/start PDM

   @param[in]  n_samples  The n samples

   @return     { description_of_the_return_value }
*/
static bool microphone_inference_start(uint32_t n_samples)
{
  inference.buffer = (int16_t *)malloc(n_samples * sizeof(int16_t));

  if (inference.buffer == NULL) {
    return false;
  }

  inference.buf_count  = 0;
  inference.n_samples  = n_samples;
  inference.buf_ready  = 0;

  // configure the data receive callback
  PDM.onReceive(pdm_data_ready_inference_callback);

  PDM.setBufferSize(2048);
  delay(250);

  // initialize PDM with:
  // - one channel (mono mode)
  if (!PDM.begin(1, EI_CLASSIFIER_FREQUENCY)) {
    ei_printf("ERR: Failed to start PDM!");
    microphone_inference_end();
    return false;
  }

  // optionally set the gain, defaults to 24
  // Note: values >=52 not supported
  //PDM.setGain(40);

  return true;
}

/**
   @brief      Wait on new data

   @return     True when finished
*/
static bool microphone_inference_record(void)
{
  bool ret = true;

  record_ready = true;
  while (inference.buf_ready == 0) {
    delay(10);
  }

  inference.buf_ready = 0;
  record_ready = false;

  return ret;
}

/**
   Get raw audio signal data
*/
static int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr)
{
  numpy::int16_to_float(&inference.buffer[offset], out_ptr, length);

  return 0;
}

/**
   @brief      Stop PDM and release buffers
*/
static void microphone_inference_end(void)
{
  PDM.end();
  ei_free(inference.buffer);
}

void print_inference_result(ei_impulse_result_t result) {

  // Print how long it took to perform inference

  for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {

    if (result.classification[i].value > 0.9) {
      ei_printf("Sound Detected: %s", ei_classifier_inferencing_categories[i]);
    }
  }

  // Print anomaly result (if it exists)
#if EI_CLASSIFIER_HAS_ANOMALY == 1
  ei_printf("Anomaly prediction: %.3f\r\n", result.anomaly);
#endif

}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_MICROPHONE
#error "Invalid model for current sensor."
#endif

void errLeds(void)
{
  while (1)
  {
    digitalWrite(PANIC_LED, HIGH);
    delay(ERROR_DUR);
    digitalWrite(PANIC_LED, LOW);
    delay(ERROR_DUR);
  }
}

void newDataCallback(const bme68xData data, const bsecOutputs outputs, Bsec2 bsec)
{
  if (!outputs.nOutputs)
  {
    return;
  }

  Serial.println("BSEC outputs:\n\ttimestamp = " + String((int) (outputs.output[0].time_stamp / INT64_C(1000000))));
  for (uint8_t i = 0; i < outputs.nOutputs; i++)
  {
    const bsecData output  = outputs.output[i];
    switch (output.sensor_id)
    {
      case BSEC_OUTPUT_IAQ:
        Serial.println("\tiaq = " + String(output.signal));
        Serial.println("\tiaq accuracy = " + String((int) output.accuracy));
        break;
      case BSEC_OUTPUT_RAW_TEMPERATURE:
        Serial.println("\ttemperature = " + String(output.signal));
        break;
      case BSEC_OUTPUT_RAW_PRESSURE:
        Serial.println("\tpressure = " + String(output.signal));
        break;
      case BSEC_OUTPUT_RAW_HUMIDITY:
        Serial.println("\thumidity = " + String(output.signal));
        break;
      case BSEC_OUTPUT_RAW_GAS:
        Serial.println("\tgas resistance = " + String(output.signal));
        break;
      case BSEC_OUTPUT_STABILIZATION_STATUS:
        Serial.println("\tstabilization status = " + String(output.signal));
        break;
      case BSEC_OUTPUT_RUN_IN_STATUS:
        Serial.println("\trun in status = " + String(output.signal));
        break;
      default:
        break;
    }
  }
}

void checkBsecStatus(Bsec2 bsec)
{
  if (bsec.status < BSEC_OK)
  {
    Serial.println("BSEC error code : " + String(bsec.status));
    errLeds(); /* Halt in case of failure */
  }
  else if (bsec.status > BSEC_OK)
  {
    Serial.println("BSEC warning code : " + String(bsec.status));
  }

  if (bsec.sensor.status < BME68X_OK)
  {
    Serial.println("BME68X error code : " + String(bsec.sensor.status));
    errLeds(); /* Halt in case of failure */
  }
  else if (bsec.sensor.status > BME68X_OK)
  {
    Serial.println("BME68X warning code : " + String(bsec.sensor.status));
  }
}
