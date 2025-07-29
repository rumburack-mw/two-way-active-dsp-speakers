#include <Wire.h>
#include <driver/i2s.h>
#include <math.h>

// I2C Pin Definitions
#define I2C_SDA 18
#define I2C_SCL 23
#define TAS5828_ADDR 0x60  // TAS5828 I2C address (default)
#define AK5522_ADDR 0x11   // AK5522 I2C address

// I2S Pin Definitions
#define I2S_BCLK 5
#define I2S_LRCK 25
#define I2S_DOUT 26
#define I2S_DIN 35

//#define DMA_BUFFER_LENGHT 256
#define DMA_BUFFER_LENGHT 1024

// PDN Pin Definition
#define PDN_PIN 27  // TAS5828 PDN pin on GPIO 27

// I2S Configuration
#define I2S_NUM I2S_NUM_0
#define SAMPLE_RATE 48000
#define BITS_PER_SAMPLE 32

// Crossover Frequency
#define CROSSOVER_FREQ 1500.0  // Crossover frequency in Hz
#define WOOFER_GAIN 1.0
#define TWEETER_GAIN 1.0


// Low Shelf Filter Parameters
#define LOW_SHELF_FREQ 200.0   // Low shelf cutoff frequency in Hz
#define LOW_SHELF_GAIN_DB 3.5  // Low shelf gain in dB
#define LOW_SHELF_Q 0.707      // Q factor for low shelf filter

// Infrasound high-pass cutoff
#define INFRASOUND_CUTOFF 20.0  // Infrasound high-pass cutoff frequency in Hz

// Biquad filter structure
typedef struct {
  double b0, b1, b2;  // Numerator coefficients
  double a0, a1, a2;  // Denominator coefficients (a0 normalized to 1)
  double x1, x2;      // Input delay line (previous samples)
  double y1, y2;      // Output delay line (previous outputs)
} Biquad;

// Filter state for low-pass, high-pass, and low shelf
Biquad lp_filter1, lp_filter2;              // Low-pass filter stages
Biquad hp_filter1, hp_filter2;              // High-pass filter stages
Biquad ls_filter;                           // Low shelf filter
Biquad infra_hp_filter1, infra_hp_filter2;  // Infrasound high-pass filter stages

// TAS5828 Register
#define FS_MON_REG 0x37  // FS_MON register address

// AK5522 Registers
#define POWER_MANAGEMENT 0x00
#define PDLDOAN 4
#define PDPLLN 3
#define PDADLN 2
#define PDADRN 1
#define RSTN 0

#define PLL_CONTROL 0x01
#define FS3 7
#define FS2 6
#define FS1 5
#define FS0 4
#define PLL3 3
#define PLL2 2
#define PLL1 1
#define PLL0 0

#define CONTROL_1 0x02
#define CM3 7
#define CM2 6
#define CM1 5
#define CM0 4
#define TDM1 3
#define TDM0 2
#define DIF 1
#define HPFE 0

// Periodic read interval
#define READ_INTERVAL 2000  // Read FS_MON every 2000 ms (2 seconds)

// Function prototypes
void calculate_lr4_coefficients(double freq, Biquad* lp1, Biquad* lp2, Biquad* hp1, Biquad* hp2);
void calculate_low_shelf_coefficients(double freq, double gain_db, double Q, Biquad* filter);
void print_filter_parameters(Biquad* lp1, Biquad* lp2, Biquad* hp1, Biquad* hp2, Biquad* ls, Biquad* infra_hp1, Biquad* infra_hp2);
bool tas5828_init();
bool AK5522_init();
bool write_reg(uint8_t reg, uint8_t value);
bool write_reg_AK5522(uint8_t reg, uint8_t value);
uint8_t read_reg(uint8_t reg);
uint8_t read_reg_AK5522(uint8_t reg);
void print_binary(uint8_t value);
bool i2s_init();
double apply_biquad(double sample, Biquad* filter);

void setup() {
  // Initialize Serial for debugging
  Serial.begin(115200);
  delay(1000);
  Serial.println("TAS5828 ESP32 2-Way Crossover Program with Low Shelf Filter");

  // Initialize PDN pin
  pinMode(PDN_PIN, OUTPUT);
  digitalWrite(PDN_PIN, LOW);  // Keep TAS5828 in power-down
  delay(10);

  // Initialize I2C
  Wire.begin(I2C_SDA, I2C_SCL);

  // Power up TAS5828
  digitalWrite(PDN_PIN, HIGH);
  delay(10);  // Wait for power-up

  // Initialize TAS5828
  if (!tas5828_init()) {
    Serial.println("TAS5828 initialization failed!");
    while (1)
      ;
  }

  // Initialize AK5522
  if (!AK5522_init()) {
    Serial.println("AK5522 initialization failed!");
    while (1)
      ;
  }

  // Initialize I2S
  if (!i2s_init()) {
    Serial.println("I2S initialization failed!");
    while (1)
      ;
  }

  // Calculate crossover filter coefficients
  calculate_lr4_coefficients(CROSSOVER_FREQ, &lp_filter1, &lp_filter2, &hp_filter1, &hp_filter2);

  // Calculate low shelf filter coefficients
  calculate_low_shelf_coefficients(LOW_SHELF_FREQ, LOW_SHELF_GAIN_DB, LOW_SHELF_Q, &ls_filter);

  // Calculate infrasound high-pass filter coefficients
  calculate_lr4_coefficients(INFRASOUND_CUTOFF, &infra_hp_filter1, &infra_hp_filter2, &infra_hp_filter1, &infra_hp_filter2);

  // Print filter parameters
  print_filter_parameters(&lp_filter1, &lp_filter2, &hp_filter1, &hp_filter2, &ls_filter, &infra_hp_filter1, &infra_hp_filter2);

  // Read and display FS_MON register once at startup
  uint8_t fs_mon_value = read_reg(FS_MON_REG);
  Serial.print("FS_MON Register (0x37) Value: 0x");
  Serial.print(fs_mon_value, HEX);
  Serial.print(" (Binary: ");
  print_binary(fs_mon_value);
  Serial.println(")");

  Serial.println("AK5522, TAS5828, I2S, and Crossover Filters initialized successfully");
}

void loop() {
  int32_t samples[64];  // Buffer for stereo samples (32 stereo pairs = 64 int32_t samples)
  size_t bytes_read, bytes_written;

  // Read from I2S input
  esp_err_t read_result = i2s_read(I2S_NUM, samples, sizeof(samples), &bytes_read, portMAX_DELAY);
  if (read_result != ESP_OK) {
    Serial.println("I2S read error");
    return;
  }

  // Verify the number of bytes read
  if (bytes_read != sizeof(samples)) {
    Serial.print("I2S read incomplete: ");
    Serial.print(bytes_read);
    Serial.println(" bytes read");
  }

  // Process samples through crossover filters
  for (int i = 0; i < bytes_read / sizeof(int32_t); i += 2) {  // Process stereo pairs
                                                               // Convert int32_t to double for filtering (normalize to [-1.0, 1.0])

    double left = (double)samples[i];  // Left channel

    // Apply low shelf filter first
    double ls_left = apply_biquad(left, &ls_filter);
    //double ls_right = apply_biquad(right, &ls_filter);

    // Apply infrasound high-pass filter (2th order = two cascaded biquads)
    //double infra_hp_left = apply_biquad(ls_left, &infra_hp_filter1);

    // Apply low-pass filter (4th order = two cascaded biquads) to low shelf output
    double lp_left = apply_biquad(apply_biquad(ls_left, &lp_filter1), &lp_filter2);
    //double lp_right = apply_biquad(apply_biquad(ls_right, &lp_filter1), &lp_filter2);

    // Apply high-pass filter (4th order = two cascaded biquads) to low shelf output
    double hp_left = apply_biquad(apply_biquad(ls_left, &hp_filter1), &hp_filter2);
    //double hp_right = apply_biquad(apply_biquad(ls_right, &hp_filter1), &hp_filter2);

    samples[i] = (int32_t)(WOOFER_GAIN * lp_left);       // Left channel (low-pass)
    samples[i + 1] = (int32_t)(TWEETER_GAIN * hp_left);  // Right channel (high-pass)
  }

  // Write processed samples to I2S output
  esp_err_t write_result = i2s_write(I2S_NUM, samples, sizeof(samples), &bytes_written, portMAX_DELAY);
  if (write_result != ESP_OK) {
    Serial.println("I2S write error");
    return;
  }

  // Verify the number of bytes written
  if (bytes_written != sizeof(samples)) {
    Serial.print("I2S write incomplete: ");
    Serial.print(bytes_written);
    Serial.println(" bytes written");
  }

  // Periodically read FS_MON and AK5522 registers
  static unsigned long last_read_time = 0;
  unsigned long current_time = millis();
  if (current_time - last_read_time >= READ_INTERVAL) {
    uint8_t fs_mon_value = read_reg(FS_MON_REG);
    Serial.print("FS_MON Register (0x37) Value: 0x");
    Serial.print(fs_mon_value, HEX);
    Serial.print(" (Binary: ");
    print_binary(fs_mon_value);
    Serial.println(")");

    fs_mon_value = read_reg_AK5522(0x00);
    Serial.print("AK5522 Register (0x00) Value: 0x");
    Serial.print(fs_mon_value, HEX);
    Serial.print(" (Binary: ");
    print_binary(fs_mon_value);
    Serial.println(")");

    fs_mon_value = read_reg_AK5522(0x01);
    Serial.print("AK5522 Register (0x01) Value: 0x");
    Serial.print(fs_mon_value, HEX);
    Serial.print(" (Binary: ");
    print_binary(fs_mon_value);
    Serial.println(")");

    fs_mon_value = read_reg_AK5522(0x02);
    Serial.print("AK5522 Register (0x02) Value: 0x");
    Serial.print(fs_mon_value, HEX);
    Serial.print(" (Binary: ");
    print_binary(fs_mon_value);
    Serial.println(")");

    last_read_time = current_time;
  }
}

// Calculate Linkwitz-Riley 4th-order filter coefficients
void calculate_lr4_coefficients(double freq, Biquad* lp1, Biquad* lp2, Biquad* hp1, Biquad* hp2) {
  // Linkwitz-Riley 4th-order is two cascaded Butterworth 2nd-order filters
  double fs = SAMPLE_RATE;
  double omega = 2.0 * M_PI * freq / fs;
  double sin_omega = sin(omega);
  double cos_omega = cos(omega);
  double Q = 0.707;  // Butterworth Q for each 2nd-order section
  double alpha = sin_omega / (2.0 * Q);

  // Common denominator term
  double a0 = 1.0 + alpha;

  // Low-pass filter coefficients (identical for both biquad stages)
  double lp_b0 = (1.0 - cos_omega) / 2.0;
  double lp_b1 = 1.0 - cos_omega;
  double lp_b2 = (1.0 - cos_omega) / 2.0;
  double lp_a1 = -2.0 * cos_omega;
  double lp_a2 = 1.0 - alpha;

  // Normalize coefficients
  lp_b0 /= a0;
  lp_b1 /= a0;
  lp_b2 /= a0;
  lp_a1 /= a0;
  lp_a2 /= a0;

  // Set low-pass filter coefficients
  lp1->b0 = lp2->b0 = lp_b0;
  lp1->b1 = lp2->b1 = lp_b1;
  lp1->b2 = lp2->b2 = lp_b2;
  lp1->a0 = lp2->a0 = 1.0;
  lp1->a1 = lp2->a1 = lp_a1;
  lp1->a2 = lp2->a2 = lp_a2;
  lp1->x1 = lp1->x2 = lp1->y1 = lp1->y2 = 0.0;
  lp2->x1 = lp2->x2 = lp2->y1 = lp2->y2 = 0.0;

  // High-pass filter coefficients (identical for both biquad stages)
  double hp_b0 = (1.0 + cos_omega) / 2.0;
  double hp_b1 = -(1.0 + cos_omega);
  double hp_b2 = (1.0 + cos_omega) / 2.0;
  double hp_a1 = -2.0 * cos_omega;
  double hp_a2 = 1.0 - alpha;

  // Normalize coefficients
  hp_b0 /= a0;
  hp_b1 /= a0;
  hp_b2 /= a0;
  hp_a1 /= a0;
  hp_a2 /= a0;

  // Set high-pass filter coefficients
  hp1->b0 = hp2->b0 = hp_b0;
  hp1->b1 = hp2->b1 = hp_b1;
  hp1->b2 = hp2->b2 = hp_b2;
  hp1->a0 = hp2->a0 = 1.0;
  hp1->a1 = hp2->a1 = hp_a1;
  hp1->a2 = hp2->a2 = hp_a2;
  hp1->x1 = hp1->x2 = hp1->y1 = hp1->y2 = 0.0;
  hp2->x1 = hp2->x2 = hp2->y1 = hp2->y2 = 0.0;
}

// Calculate low shelf filter coefficients
void calculate_low_shelf_coefficients(double freq, double gain_db, double Q, Biquad* filter) {
  double fs = SAMPLE_RATE;
  double omega = 2.0 * M_PI * freq / fs;
  double sin_omega = sin(omega);
  double cos_omega = cos(omega);
  double A = pow(10.0, gain_db / 40.0);  // Amplitude gain
  double beta = sqrt(A) / Q;

  // Intermediate terms
  double alpha = sin_omega / 2.0 * beta;
  double a0 = (A + 1.0) + (A - 1.0) * cos_omega + 2.0 * sqrt(A) * alpha;

  // Low shelf filter coefficients
  double b0 = A * ((A + 1.0) - (A - 1.0) * cos_omega + 2.0 * sqrt(A) * alpha);
  double b1 = 2.0 * A * ((A - 1.0) - (A + 1.0) * cos_omega);
  double b2 = A * ((A + 1.0) - (A - 1.0) * cos_omega - 2.0 * sqrt(A) * alpha);
  double a1 = -2.0 * ((A - 1.0) + (A + 1.0) * cos_omega);
  double a2 = (A + 1.0) + (A - 1.0) * cos_omega - 2.0 * sqrt(A) * alpha;

  // Normalize coefficients
  b0 /= a0;
  b1 /= a0;
  b2 /= a0;
  a1 /= a0;
  a2 /= a0;

  // Set filter coefficients
  filter->b0 = b0;
  filter->b1 = b1;
  filter->b2 = b2;
  filter->a0 = 1.0;
  filter->a1 = a1;
  filter->a2 = a2;
  filter->x1 = filter->x2 = filter->y1 = filter->y2 = 0.0;
}

// Print filter parameters
void print_filter_parameters(Biquad* lp1, Biquad* lp2, Biquad* hp1, Biquad* hp2, Biquad* ls, Biquad* infra_hp1, Biquad* infra_hp2) {
  Serial.println("Linkwitz-Riley 4th-Order Crossover Filter Parameters");
  Serial.print("Crossover Frequency: ");
  Serial.print(CROSSOVER_FREQ);
  Serial.println(" Hz");
  Serial.print("Sample Rate: ");
  Serial.print(SAMPLE_RATE);
  Serial.println(" Hz");
  Serial.println();

  Serial.println("Low-Pass Filter 1 Coefficients:");
  Serial.print("  b0: ");
  Serial.println(lp1->b0, 6);
  Serial.print("  b1: ");
  Serial.println(lp1->b1, 6);
  Serial.print("  b2: ");
  Serial.println(lp1->b2, 6);
  Serial.print("  a0: ");
  Serial.println(lp1->a0, 6);
  Serial.print("  a1: ");
  Serial.println(lp1->a1, 6);
  Serial.print("  a2: ");
  Serial.println(lp1->a2, 6);
  Serial.println();

  Serial.println("Low-Pass Filter 2 Coefficients:");
  Serial.print("  b0: ");
  Serial.println(lp2->b0, 6);
  Serial.print("  b1: ");
  Serial.println(lp2->b1, 6);
  Serial.print("  b2: ");
  Serial.println(lp2->b2, 6);
  Serial.print("  a0: ");
  Serial.println(lp2->a0, 6);
  Serial.print("  a1: ");
  Serial.println(lp2->a1, 6);
  Serial.print("  a2: ");
  Serial.println(lp2->a2, 6);
  Serial.println();

  Serial.println("High-Pass Filter 1 Coefficients:");
  Serial.print("  b0: ");
  Serial.println(hp1->b0, 6);
  Serial.print("  b1: ");
  Serial.println(hp1->b1, 6);
  Serial.print("  b2: ");
  Serial.println(hp1->b2, 6);
  Serial.print("  a0: ");
  Serial.println(hp1->a0, 6);
  Serial.print("  a1: ");
  Serial.println(hp1->a1, 6);
  Serial.print("  a2: ");
  Serial.println(hp1->a2, 6);
  Serial.println();

  Serial.println("High-Pass Filter 2 Coefficients:");
  Serial.print("  b0: ");
  Serial.println(hp2->b0, 6);
  Serial.print("  b1: ");
  Serial.println(hp2->b1, 6);
  Serial.print("  b2: ");
  Serial.println(hp2->b2, 6);
  Serial.print("  a0: ");
  Serial.println(hp2->a0, 6);
  Serial.print("  a1: ");
  Serial.println(hp2->a1, 6);
  Serial.print("  a2: ");
  Serial.println(hp2->a2, 6);
  Serial.println();

  Serial.println("Low Shelf Filter Coefficients:");
  Serial.print("  b0: ");
  Serial.println(ls->b0, 6);
  Serial.print("  b1: ");
  Serial.println(ls->b1, 6);
  Serial.print("  b2: ");
  Serial.println(ls->b2, 6);
  Serial.print("  a0: ");
  Serial.println(ls->a0, 6);
  Serial.print("  a1: ");
  Serial.println(ls->a1, 6);
  Serial.print("  a2: ");
  Serial.println(ls->a2, 6);
  Serial.println();

  Serial.println("Infrasound High-Pass Filter 1 Coefficients:");
  Serial.print("  b0: ");
  Serial.println(infra_hp1->b0, 6);
  Serial.print("  b1: ");
  Serial.println(infra_hp1->b1, 6);
  Serial.print("  b2: ");
  Serial.println(infra_hp1->b2, 6);
  Serial.print("  a0: ");
  Serial.println(infra_hp1->a0, 6);
  Serial.print("  a1: ");
  Serial.println(infra_hp1->a1, 6);
  Serial.print("  a2: ");
  Serial.println(infra_hp1->a2, 6);
  Serial.println();

  Serial.println("Infrasound High-Pass Filter 2 Coefficients:");
  Serial.print("  b0: ");
  Serial.println(infra_hp2->b0, 6);
  Serial.print("  b1: ");
  Serial.println(infra_hp2->b1, 6);
  Serial.print("  b2: ");
  Serial.println(infra_hp2->b2, 6);
  Serial.print("  a0: ");
  Serial.println(infra_hp2->a0, 6);
  Serial.print("  a1: ");
  Serial.println(infra_hp2->a1, 6);
  Serial.print("  a2: ");
  Serial.println(infra_hp2->a2, 6);
  Serial.println();
}


// Apply biquad filter to a sample
double apply_biquad(double sample, Biquad* filter) {
  // Direct Form I biquad implementation
  double result = filter->b0 * sample + filter->b1 * filter->x1 + filter->b2 * filter->x2
                  - filter->a1 * filter->y1 - filter->a2 * filter->y2;

  // Update delay lines
  filter->x2 = filter->x1;
  filter->x1 = sample;
  filter->y2 = filter->y1;
  filter->y1 = result;

  return result;
}

// Initialize TAS5828 via I2C
bool tas5828_init() {
  // Reset device
  if (!write_reg(0x01, 0x11)) return false;  // Software reset
  delay(10);
  // Configure format (I2S, 32-bit)
  if (!write_reg(0x33, 0b00000011)) return false;  // I2S format, 32-bit
  delay(10);
  // Set to play mode
  if (!write_reg(0x03, 0x03)) return false;  // Exit power-down
  delay(10);
  return true;
}

// Initialize AK5522 via I2C
bool AK5522_init() {
  uint8_t set;
  set = 0;
  set |= (1 << PDPLLN) | (1 << PDADLN) | (1 << PDADRN) | (1 << PDLDOAN) | (1 << RSTN);
  if (!write_reg_AK5522(POWER_MANAGEMENT, set)) return false;
  delay(10);
  // PLL
  set = 0;
  set |= (1 << PLL1) | (1 << FS3);
  if (!write_reg_AK5522(PLL_CONTROL, set)) return false;
  delay(10);
  // I2S
  set = 0;
  set |= (1 << DIF) | (1 << CM2);
  if (!write_reg_AK5522(CONTROL_1, set)) return false;
  delay(10);
  return true;
}

// Write to TAS5828 register
bool write_reg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(TAS5828_ADDR);
  Wire.write(reg);
  Wire.write(value);
  if (Wire.endTransmission() != 0) {
    Serial.print("Failed to write to register 0x");
    Serial.println(reg, HEX);
    return false;
  }
  return true;
}

// Write to AK5522 register
bool write_reg_AK5522(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(AK5522_ADDR);
  Wire.write(reg);
  Wire.write(value);
  if (Wire.endTransmission() != 0) {
    Serial.print("Failed to write to register 0x");
    Serial.println(reg, HEX);
    return false;
  }
  return true;
}

// Read from TAS5828 register
uint8_t read_reg(uint8_t reg) {
  Wire.beginTransmission(TAS5828_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    Serial.print("Failed to set register 0x");
    Serial.println(reg, HEX);
    return 0;
  }
  Wire.requestFrom(TAS5828_ADDR, 1);
  if (Wire.available()) {
    return Wire.read();
  } else {
    Serial.print("Failed to read register 0x");
    Serial.println(reg, HEX);
    return 0;
  }
}

// Read from AK5522 register
uint8_t read_reg_AK5522(uint8_t reg) {
  Wire.beginTransmission(AK5522_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    Serial.print("Failed to set register 0x");
    Serial.println(reg, HEX);
    return 0;
  }
  Wire.requestFrom(AK5522_ADDR, 1);
  if (Wire.available()) {
    return Wire.read();
  } else {
    Serial.print("Failed to read register 0x");
    Serial.println(reg, HEX);
    return 0;
  }
}

// Print byte in binary format
void print_binary(uint8_t value) {
  for (int i = 7; i >= 0; i--) {
    Serial.print((value >> i) & 1);
  }
}

// Initialize I2S
bool i2s_init() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .dma_buf_count = 8,
    .dma_buf_len = DMA_BUFFER_LENGHT,
    .use_apll = true,
    .tx_desc_auto_clear = true,
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_BCLK,
    .ws_io_num = I2S_LRCK,
    .data_out_num = I2S_DOUT,
    .data_in_num = I2S_DIN
  };

  esp_err_t ret = i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
  if (ret != ESP_OK) return false;

  ret = i2s_set_pin(I2S_NUM, &pin_config);
  if (ret != ESP_OK) return false;

  return true;
}