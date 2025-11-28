/* Radar Person Detection with TensorFlow Lite Micro
 * DEBUG VERSION - Uses AllOpsResolver to find required operations
 * Single file implementation
 */

#include <TensorFlowLite.h>
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/all_ops_resolver.h"  // ← Use ALL ops for debugging
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"

// YOUR MODEL FILES (must be in same folder)
#include "model.h"

// ============================================================================
// RADAR CONFIGURATION
// ============================================================================
#define NUM_ANTENNAS 3
#define RAW_FRAME_SIZE 240
#define CIR_SIZE 20
#define HEADER_SIZE 0
#define INTERESTING_CIR_INDEX 0
#define INTERESTING_CIR_SIZE 20
#define ALPHA 0.95

// ML CONFIGURATION 
#define WINDOW_SIZE 50
#define FFT_SIZE 64
#define FFT_OUTPUT_SIZE 32
#define NUM_RANGE_BINS 20

// ============================================================================
// DATA STRUCTURES
// ============================================================================
enum State {
  START_RADAR,
  READ_DATA,
  PREPROCESSING,
  RUN_MODEL
};

enum ReadState {
  BEGIN,
  FRAME,
  END
};

struct Complex {
  float real;
  float imag;
};

// ============================================================================
// GLOBALS
// ============================================================================
State currentState = START_RADAR;
uint8_t raw_frame[RAW_FRAME_SIZE];
bool firstSample = true;
Complex decBase[NUM_ANTENNAS][INTERESTING_CIR_SIZE];
int iteration = 0;

float frame_buffer[WINDOW_SIZE][INTERESTING_CIR_SIZE];
int frame_count = 0;
float hamming_window[WINDOW_SIZE];

namespace {
  tflite::ErrorReporter* error_reporter = nullptr;
  const tflite::Model* model = nullptr;
  tflite::MicroInterpreter* interpreter = nullptr;
  TfLiteTensor* input = nullptr;
  TfLiteTensor* output = nullptr;
  
  // Increased arena for AllOpsResolver (uses more memory)
  constexpr int kTensorArenaSize = 10 * 1024;
  alignas(16) uint8_t tensor_arena[kTensorArenaSize];
}

const char* CLASS_NAMES[] = {"person", "no_person"};

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================
void sendDEBUG(String print){
  Serial.println(print);
}

bool consume_string(String string_to_check){
  for (int i = 0; i < string_to_check.length(); i++){
    while (!Serial1.available()){
      delay(1);
    }
    char temp = (char)Serial1.read();
    // Serial.print(temp);
    if ((char)string_to_check[i] != temp){
      return false;
    }
  }
  while (!Serial1.available()){
    delay(1);
  }
  if (Serial1.read() != 0x0A){
    return false;
  }
  return true;
}

void init_hamming_window() {
  for (int i = 0; i < WINDOW_SIZE; i++) {
    hamming_window[i] = 0.54 - 0.46 * cos(2.0 * PI * i / (WINDOW_SIZE - 1));
  }
}

// ============================================================================
// RADAR FUNCTIONS
// ============================================================================
bool read_data(){
  int frame_index = 0;
  ReadState readState = BEGIN;
  char hexStr[RAW_FRAME_SIZE * 3 + 1]; 

  while(true){
    switch (readState) {
      case BEGIN:
        if (consume_string("BEGIN")){
          readState = FRAME;
        }
        else{
          sendDEBUG("Error BEGIN!");
        }
        break;
        
      case FRAME:
        frame_index = 0;
        while (frame_index < RAW_FRAME_SIZE) {
          if (Serial1.available()){
            raw_frame[frame_index++] = Serial1.read();
          }
        }
        Serial1.read();

        for (int i = 0; i < RAW_FRAME_SIZE; i++) {
          sprintf(hexStr + i*3, "%02X ", raw_frame[i]);
        }
        // sendDEBUG(hexStr);

        readState = END;
        break;
        
      case END:
        return consume_string("END");
        break;
    }
  }
  return false;
}

void processRx(int16_t *rx, Complex *output) {
  Complex cir[CIR_SIZE - HEADER_SIZE];
  int idx = 0;

  for (int i = HEADER_SIZE * 2; i < CIR_SIZE * 2; i += 2) {
    cir[idx].real = (float)rx[i];
    cir[idx].imag = (float)rx[i + 1];
    idx++;
  }

  for (int i = 0; i < INTERESTING_CIR_SIZE; i++) {
    output[i] = cir[INTERESTING_CIR_INDEX + i];
  }
}

void decluttering(int antenna, Complex *cir, Complex *out) {
  if (firstSample) {
    for (int i = 0; i < INTERESTING_CIR_SIZE; i++) {
      decBase[antenna][i] = cir[i];
      out[i] = cir[i];
    }
    return;
  }

  for (int i = 0; i < INTERESTING_CIR_SIZE; i++) {
    float base_r = ALPHA * decBase[antenna][i].real + (1 - ALPHA) * cir[i].real;
    float base_i = ALPHA * decBase[antenna][i].imag + (1 - ALPHA) * cir[i].imag;
    decBase[antenna][i].real = base_r;
    decBase[antenna][i].imag = base_i;
    out[i].real = cir[i].real - base_r;
    out[i].imag = cir[i].imag - base_i;
  }
}

void preprocessing(){
  int16_t frame[RAW_FRAME_SIZE / 2];
  Complex cir[NUM_ANTENNAS][INTERESTING_CIR_SIZE];
  Complex dec[NUM_ANTENNAS][INTERESTING_CIR_SIZE];

  for (int i = 0; i < RAW_FRAME_SIZE; i += 2) {
    frame[i/2] = (int16_t)((raw_frame[i+1] << 8) | raw_frame[i]);
  }

  for (int i = 0; i < NUM_ANTENNAS; i++) {
    processRx(&frame[i * CIR_SIZE * 2], cir[i]);
    decluttering(i, cir[i], dec[i]);
  }

  if (!firstSample && frame_count < WINDOW_SIZE) {
    for (int i = 0; i < INTERESTING_CIR_SIZE; i++) {
      frame_buffer[frame_count][i] = sqrt(dec[0][i].real * dec[0][i].real + 
                                          dec[0][i].imag * dec[0][i].imag);
    }
    frame_count++;
  }
}

// ============================================================================
// FFT FUNCTIONS
// ============================================================================
void fft(Complex* x, int N) {
  int j = 0;
  for (int i = 1; i < N; i++) {
    int bit = N >> 1;
    while (j >= bit) {
      j -= bit;
      bit >>= 1;
    }
    j += bit;
    if (i < j) {
      Complex temp = x[i];
      x[i] = x[j];
      x[j] = temp;
    }
  }
  
  for (int len = 2; len <= N; len <<= 1) {
    float angle = -2.0 * PI / len;
    Complex wlen = {cos(angle), sin(angle)};
    
    for (int i = 0; i < N; i += len) {
      Complex w = {1.0, 0.0};
      
      for (int j = 0; j < len / 2; j++) {
        Complex u = x[i + j];
        Complex v = {
          w.real * x[i + j + len/2].real - w.imag * x[i + j + len/2].imag,
          w.real * x[i + j + len/2].imag + w.imag * x[i + j + len/2].real
        };
        
        x[i + j].real = u.real + v.real;
        x[i + j].imag = u.imag + v.imag;
        x[i + j + len/2].real = u.real - v.real;
        x[i + j + len/2].imag = u.imag - v.imag;
        
        float w_temp = w.real * wlen.real - w.imag * wlen.imag;
        w.imag = w.real * wlen.imag + w.imag * wlen.real;
        w.real = w_temp;
      }
    }
  }
}

void fftshift(float* x, int N) {
  int half = N / 2;
  for (int i = 0; i < half; i++) {
    float temp = x[i];
    x[i] = x[i + half];
    x[i + half] = temp;
  }
}

void compute_fft_features(float fft_features[FFT_OUTPUT_SIZE][NUM_RANGE_BINS]) {
  Complex fft_input[FFT_SIZE];
  
  for (int range_bin = 0; range_bin < NUM_RANGE_BINS; range_bin++) {
    for (int i = 0; i < WINDOW_SIZE; i++) {
      fft_input[i].real = frame_buffer[i][range_bin] * hamming_window[i];
      fft_input[i].imag = 0.0f;
    }
    
    for (int i = WINDOW_SIZE; i < FFT_SIZE; i++) {
      fft_input[i].real = 0.0f;
      fft_input[i].imag = 0.0f;
    }
    
    fft(fft_input, FFT_SIZE);
    
    float fft_mag[FFT_SIZE];
    for (int i = 0; i < FFT_SIZE; i++) {
      fft_mag[i] = sqrt(fft_input[i].real * fft_input[i].real + 
                       fft_input[i].imag * fft_input[i].imag);
    }
    
    fftshift(fft_mag, FFT_SIZE);
    
    int start_idx = (FFT_SIZE - FFT_OUTPUT_SIZE) / 2;
    for (int i = 0; i < FFT_OUTPUT_SIZE; i++) {
      fft_features[i][range_bin] = fft_mag[start_idx + i];
    }
  }
}

// ============================================================================
// ML INFERENCE
// ============================================================================
void run_model(){
  if (iteration % 10 == 0){
    String temp = "Time: " + String(iteration/10) + "s";
    sendDEBUG(temp);
  }
  iteration++;

  if (frame_count >= WINDOW_SIZE) {
    sendDEBUG("Running inference...");
    unsigned long t0 = millis();
    
    float fft_features[FFT_OUTPUT_SIZE][NUM_RANGE_BINS];
    compute_fft_features(fft_features);
    
    for (int i = 0; i < FFT_OUTPUT_SIZE; i++) {
      for (int j = 0; j < NUM_RANGE_BINS; j++) {
        int idx = i * NUM_RANGE_BINS + j;
        input->data.f[idx] = fft_features[i][j];
      }
    }
    
    TfLiteStatus invoke_status = interpreter->Invoke();
    if (invoke_status != kTfLiteOk) {
      TF_LITE_REPORT_ERROR(error_reporter, "Invoke failed");
      frame_count = 0;
      return;
    }
    
    float person_score = output->data.f[0];
    float no_person_score = output->data.f[1];
    int predicted_class = (person_score > no_person_score) ? 0 : 1;
    
    unsigned long inference_time = millis() - t0;
    
    sendDEBUG("=================================");
    sendDEBUG("Prediction: " + String(CLASS_NAMES[predicted_class]));
    sendDEBUG("Inference time: " + String(inference_time) + " ms");
    sendDEBUG("=================================");
    
    frame_count = 0;
  }
}

// ============================================================================
// ARDUINO SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  delay(1000);
  
  sendDEBUG("=== RADAR PERSON DETECTION (DEBUG) ===");
  
  init_hamming_window();
  
  sendDEBUG("Setting up TensorFlow Lite...");
  
  static tflite::MicroErrorReporter micro_error_reporter;
  error_reporter = &micro_error_reporter;
  
  model = tflite::GetModel(g_model);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    TF_LITE_REPORT_ERROR(error_reporter,
                         "Model schema version %d not equal to supported version %d.",
                         model->version(), TFLITE_SCHEMA_VERSION);
    sendDEBUG("Model version mismatch!");
    while(1) delay(1000);
  }
  sendDEBUG("Model loaded OK");
  
  // USE ALL OPS (for debugging - uses more memory but works with any model)
  sendDEBUG("Using AllOpsResolver (debug mode)...");
  static tflite::AllOpsResolver resolver;
  
  sendDEBUG("Creating interpreter...");
  sendDEBUG("Arena size: " + String(kTensorArenaSize) + " bytes");
  
  static tflite::MicroInterpreter static_interpreter(
      model, resolver, tensor_arena, kTensorArenaSize, error_reporter);
  interpreter = &static_interpreter;
  
  sendDEBUG("Allocating tensors...");
  TfLiteStatus allocate_status = interpreter->AllocateTensors();
  if (allocate_status != kTfLiteOk) {
    TF_LITE_REPORT_ERROR(error_reporter, "AllocateTensors() failed");
    sendDEBUG("Tensor allocation FAILED!");
    sendDEBUG("Arena may be too small or model incompatible");
    while(1) delay(1000);
  }
  
  input = interpreter->input(0);
  output = interpreter->output(0);
  
  sendDEBUG("TensorFlow Lite ready!");
  sendDEBUG("Arena used: " + String(interpreter->arena_used_bytes()) + " / " + String(kTensorArenaSize));
  sendDEBUG("Input shape: [" + String(input->dims->data[0]) + ", " + 
            String(input->dims->data[1]) + ", " + 
            String(input->dims->data[2]) + ", " + 
            String(input->dims->data[3]) + "]");
  
  sendDEBUG("Finding radar...");
  bool radar_found = false;
  while (!radar_found){
    unsigned long t0 = millis();
    String lineBuffer = "";

    Serial1.print("INFO\r\n");

    while (millis() - t0 < 4000) {
      if (Serial1.available()) {
        char c = Serial1.read();
        lineBuffer += c;
      }
    }

    if (lineBuffer.indexOf("SR250") != -1) {
      radar_found = true;
      sendDEBUG("SR250 radar found!");
    }
  }
  
  sendDEBUG("=== READY ===");
}

// ============================================================================
// ARDUINO LOOP
// ============================================================================
void loop() {
  switch (currentState) {
    case START_RADAR:
      Serial1.print("START\n");
      currentState = READ_DATA;
      break;

    case READ_DATA:
      if (read_data()){
        currentState = PREPROCESSING;
      }
      else{
        sendDEBUG("Error parser frames - ErorrEND!!!");
      }
      break;

    case PREPROCESSING:
      preprocessing();
      if (firstSample) {
        currentState = READ_DATA;
        firstSample = false;
      }
      else{
        currentState = RUN_MODEL;
      }
      break;

    case RUN_MODEL:
      run_model();
      currentState = READ_DATA;
      break;
  }
}
