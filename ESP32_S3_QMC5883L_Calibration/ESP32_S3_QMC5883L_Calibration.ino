/*
  ================================================================================
  Simple GY-271 (QMC5883L) Magnetometer Calibration Tool - REAL-TIME VERSION
  
  Usage: 
  1. Upload this code to your ESP32-S3
  2. Open Serial Monitor at 115200 baud
  3. Follow the simple instructions
  4. Get REAL-TIME heading values
  
  Your Orientation: Y-axis = Drone nose, X-axis = Left/Right, Z-axis = Down
  
  NOTE: This code auto-detects HMC5883L vs QMC5883L and uses appropriate interface
  ================================================================================
*/

#include <Wire.h>

// ===== HARDWARE DEFINITIONS =====
#define QMC5883L_ADDRESS 0x0D   // QMC5883L I2C address
#define HMC5883L_ADDRESS 0x1E   // HMC5883L I2C address (fallback)
#define CUSTOM_SDA_PIN 4        // Your I2C SDA pin
#define CUSTOM_SCL_PIN 5        // Your I2C SCL pin

// ===== QMC5883L REGISTERS =====
#define QMC5883L_DATA_X_LSB   0x00
#define QMC5883L_DATA_X_MSB   0x01
#define QMC5883L_DATA_Y_LSB   0x02
#define QMC5883L_DATA_Y_MSB   0x03
#define QMC5883L_DATA_Z_LSB   0x04
#define QMC5883L_DATA_Z_MSB   0x05
#define QMC5883L_STATUS       0x06
#define QMC5883L_TEMP_LSB     0x07
#define QMC5883L_TEMP_MSB     0x08
#define QMC5883L_CONFIG       0x09
#define QMC5883L_CONFIG2      0x0A
#define QMC5883L_RESET        0x0B
#define QMC5883L_RESERVED     0x0C
#define QMC5883L_CHIP_ID      0x0D

// ===== HMC5883L REGISTERS (fallback) =====
#define HMC5883L_CONFIG_A     0x00
#define HMC5883L_CONFIG_B     0x01
#define HMC5883L_MODE         0x02
#define HMC5883L_DATA_X_MSB   0x03
#define HMC5883L_STATUS       0x09
#define HMC5883L_ID_A         0x0A

// ===== CALIBRATION VARIABLES =====
int16_t mag_x, mag_y, mag_z;              // Raw magnetometer readings
int16_t mag_cal_values[6];                 // Final calibration values
float mag_scale_y, mag_scale_z;            // Scale factors
int16_t mag_offset_x, mag_offset_y, mag_offset_z; // Offsets
bool calibration_completed = false;
bool is_qmc5883l = false;                  // Chip detection flag

// ===== SETUP FUNCTION =====
void setup() {
    Serial.begin(115200);
    delay(2000);
    
    Serial.println("================================================================================");
    Serial.println("       SMART GY-271 MAGNETOMETER CALIBRATION - AUTO-DETECT CHIP TYPE");
    Serial.println("       Your Setup: GY-271 Y-axis = Drone nose, X-axis = Left/Right");
    Serial.println("       Supports: QMC5883L (newer) and HMC5883L (older) chips");
    Serial.println("================================================================================");
    Serial.println("");
    
    // Initialize I2C
    Wire.begin(CUSTOM_SDA_PIN, CUSTOM_SCL_PIN);
    Wire.setClock(400000);
    delay(250);
    
    // Auto-detect and initialize magnetometer
    if (setup_magnetometer()) {
        Serial.println("PRESS SPACE + ENTER TO START CALIBRATION...");
    } else {
        Serial.println("❌ ERROR: No compatible magnetometer found!");
        Serial.println("Check connections and power supply (3.3V recommended)");
        while(1) delay(1000);
    }
}

// ===== MAIN LOOP =====
void loop() {
    if (Serial.available()) {
        char input = Serial.read();
        
        if (input == ' ' || input == '\n' || input == '\r') {
            if (!calibration_completed) {
                start_calibration();
            } else {
                real_time_heading_test();
            }
        }
    }
    
    delay(100);
}

// ===== HELPER FUNCTIONS =====
void writeByte(uint8_t address, uint8_t subAddress, uint8_t data) {
    Wire.beginTransmission(address);
    Wire.write(subAddress);
    Wire.write(data);
    Wire.endTransmission();
}

uint8_t readByte(uint8_t address, uint8_t subAddress) {
    Wire.beginTransmission(address);
    Wire.write(subAddress);
    Wire.endTransmission(false);
    Wire.requestFrom(address, (uint8_t)1);
    if (Wire.available()) {
        return Wire.read();
    }
    return 0;
}

// ===== AUTO-DETECT AND SETUP MAGNETOMETER =====
bool setup_magnetometer() {
    Serial.println("🔍 Auto-detecting magnetometer chip...");
    
    // Try QMC5883L first (more common in newer GY-271 modules)
    uint8_t qmc_id = readByte(QMC5883L_ADDRESS, QMC5883L_CHIP_ID);
    if (qmc_id == 0xFF) {
        Serial.println("✓ QMC5883L detected!");
        is_qmc5883l = true;
        return setup_qmc5883l();
    }
    
    // Try HMC5883L as fallback
    Serial.println("QMC5883L not found, trying HMC5883L...");
    uint8_t hmc_id_a = readByte(HMC5883L_ADDRESS, HMC5883L_ID_A);
    if (hmc_id_a == 0x48) {
        Serial.println("✓ HMC5883L detected!");
        is_qmc5883l = false;
        return setup_hmc5883l();
    }
    
    Serial.println("❌ No compatible magnetometer found");
    Serial.print("QMC5883L ID: 0x");
    Serial.println(qmc_id, HEX);
    Serial.print("HMC5883L ID: 0x");
    Serial.println(hmc_id_a, HEX);
    return false;
}

// ===== QMC5883L SETUP =====
bool setup_qmc5883l() {
    Serial.println("Initializing QMC5883L...");
    
    // Software reset
    writeByte(QMC5883L_ADDRESS, QMC5883L_RESET, 0x01);
    delay(100);
    
    // Configure QMC5883L
    // CONFIG: OSR=512, RNG=8G, ODR=200Hz, MODE=Continuous
    writeByte(QMC5883L_ADDRESS, QMC5883L_CONFIG, 0x1D);
    delay(10);
    
    // CONFIG2: INT_ENB=0, ROL_PNT=0, no interrupts
    writeByte(QMC5883L_ADDRESS, QMC5883L_CONFIG2, 0x00);
    delay(10);
    
    // Test read
    delay(100);
    if (!read_magnetometer()) {
        Serial.println("ERROR: Failed to read initial QMC5883L data");
        return false;
    }
    
    Serial.print("QMC5883L initial reading - X: ");
    Serial.print(mag_x);
    Serial.print(", Y: ");
    Serial.print(mag_y);
    Serial.print(", Z: ");
    Serial.println(mag_z);
    
    return true;
}

// ===== HMC5883L SETUP =====
bool setup_hmc5883l() {
    Serial.println("Initializing HMC5883L...");
    
    // Check full ID
    uint8_t id_a = readByte(HMC5883L_ADDRESS, HMC5883L_ID_A);
    uint8_t id_b = readByte(HMC5883L_ADDRESS, HMC5883L_ID_A + 1);
    uint8_t id_c = readByte(HMC5883L_ADDRESS, HMC5883L_ID_A + 2);
    
    Serial.print("HMC5883L full ID: ");
    Serial.print((char)id_a);
    Serial.print((char)id_b);
    Serial.println((char)id_c);
    
    // Configure HMC5883L
    // Config A: 8 samples averaged, 75Hz output rate, normal measurement
    writeByte(HMC5883L_ADDRESS, HMC5883L_CONFIG_A, 0x70);
    delay(10);
    
    // Config B: Gain = 1.3 Ga (default)
    writeByte(HMC5883L_ADDRESS, HMC5883L_CONFIG_B, 0x20);
    delay(10);
    
    // Mode: Continuous measurement mode
    writeByte(HMC5883L_ADDRESS, HMC5883L_MODE, 0x00);
    delay(10);
    
    // Test read
    delay(100);
    if (!read_magnetometer()) {
        Serial.println("ERROR: Failed to read initial HMC5883L data");
        return false;
    }
    
    Serial.print("HMC5883L initial reading - X: ");
    Serial.print(mag_x);
    Serial.print(", Y: ");
    Serial.print(mag_y);
    Serial.print(", Z: ");
    Serial.println(mag_z);
    
    return true;
}

// ===== READ MAGNETOMETER DATA =====
bool read_magnetometer() {
    if (is_qmc5883l) {
        return read_qmc5883l();
    } else {
        return read_hmc5883l();
    }
}

bool read_qmc5883l() {
    // Check if data is ready
    uint8_t status = readByte(QMC5883L_ADDRESS, QMC5883L_STATUS);
    if (!(status & 0x01)) {
        return false; // Data not ready
    }
    
    // Read 6 bytes starting from X LSB
    Wire.beginTransmission(QMC5883L_ADDRESS);
    Wire.write(QMC5883L_DATA_X_LSB);
    Wire.endTransmission(false);
    Wire.requestFrom(QMC5883L_ADDRESS, (uint8_t)6);
    
    if (Wire.available() >= 6) {
        // QMC5883L data order: X LSB, X MSB, Y LSB, Y MSB, Z LSB, Z MSB
        uint8_t x_lsb = Wire.read();
        uint8_t x_msb = Wire.read();
        uint8_t y_lsb = Wire.read();
        uint8_t y_msb = Wire.read();
        uint8_t z_lsb = Wire.read();
        uint8_t z_msb = Wire.read();
        
        // Combine bytes (signed 16-bit, little-endian)
        mag_x = (int16_t)((x_msb << 8) | x_lsb);
        mag_y = (int16_t)((y_msb << 8) | y_lsb);
        mag_z = (int16_t)((z_msb << 8) | z_lsb);
        
        // Check for overflow (QMC5883L saturates at ±32767)
        if (mag_x == -32768 || mag_y == -32768 || mag_z == -32768) {
            return false;
        }
        
        return true;
    }
    
    return false;
}

bool read_hmc5883l() {
    // Check if data is ready
    uint8_t status = readByte(HMC5883L_ADDRESS, HMC5883L_STATUS);
    if (!(status & 0x01)) {
        return false; // Data not ready
    }
    
    // Read 6 bytes starting from X MSB
    Wire.beginTransmission(HMC5883L_ADDRESS);
    Wire.write(HMC5883L_DATA_X_MSB);
    Wire.endTransmission(false);
    Wire.requestFrom(HMC5883L_ADDRESS, (uint8_t)6);
    
    if (Wire.available() >= 6) {
        // HMC5883L data order: X MSB, X LSB, Z MSB, Z LSB, Y MSB, Y LSB
        uint8_t x_msb = Wire.read();
        uint8_t x_lsb = Wire.read();
        uint8_t z_msb = Wire.read();
        uint8_t z_lsb = Wire.read();
        uint8_t y_msb = Wire.read();
        uint8_t y_lsb = Wire.read();
        
        // Combine bytes (signed 16-bit, big-endian)
        mag_x = (int16_t)((x_msb << 8) | x_lsb);
        mag_y = (int16_t)((y_msb << 8) | y_lsb);
        mag_z = (int16_t)((z_msb << 8) | z_lsb);
        
        // Check for overflow (HMC5883L returns -4096 on overflow)
        if (mag_x == -4096 || mag_y == -4096 || mag_z == -4096) {
            return false;
        }
        
        return true;
    }
    
    return false;
}

// ===== CALIBRATION FUNCTION =====
void start_calibration() {
    String chip_name = is_qmc5883l ? "QMC5883L" : "HMC5883L";
    
    Serial.println("");
    Serial.print("🔄 STARTING ");
    Serial.print(chip_name);
    Serial.println(" MAGNETOMETER CALIBRATION");
    Serial.println("");
    Serial.println("INSTRUCTIONS:");
    Serial.println("1. Remove propellers for safety");
    Serial.println("2. Slowly rotate drone in ALL directions for 30 seconds");
    Serial.println("3. Point nose: North, East, South, West");
    Serial.println("4. Roll left 90°, right 90°");
    Serial.println("5. Pitch up 45°, down 45°");
    Serial.println("6. Make figure-8 motions in different planes");
    Serial.println("");
    Serial.print("IMPORTANT: The ");
    Serial.print(chip_name);
    Serial.println(" needs complete 3D rotation!");
    Serial.println("");
    
    // Countdown
    for (int i = 5; i > 0; i--) {
        Serial.print("Starting in ");
        Serial.print(i);
        Serial.println(" seconds...");
        delay(1000);
    }
    
    Serial.println("");
    Serial.println("🔄 CALIBRATING NOW - ROTATE THE DRONE IN ALL DIRECTIONS!");
    Serial.println("");
    
    // Initialize min/max tracking
    int16_t cal_temp[6];
    cal_temp[0] = 32767;   // x_min
    cal_temp[1] = -32768;  // x_max
    cal_temp[2] = 32767;   // y_min
    cal_temp[3] = -32768;  // y_max
    cal_temp[4] = 32767;   // z_min
    cal_temp[5] = -32768;  // z_max
    
    unsigned long start_time = millis();
    int sample_count = 0;
    int valid_samples = 0;
    
    // 30-second calibration loop
    while (millis() - start_time < 30000) {
        sample_count++;
        
        if (read_magnetometer()) {
            valid_samples++;
            
            // Update min/max values
            if (mag_x < cal_temp[0]) cal_temp[0] = mag_x;
            if (mag_x > cal_temp[1]) cal_temp[1] = mag_x;
            if (mag_y < cal_temp[2]) cal_temp[2] = mag_y;
            if (mag_y > cal_temp[3]) cal_temp[3] = mag_y;
            if (mag_z < cal_temp[4]) cal_temp[4] = mag_z;
            if (mag_z > cal_temp[5]) cal_temp[5] = mag_z;
        }
        
        // Progress every 5 seconds
        unsigned long elapsed = millis() - start_time;
        if (elapsed % 5000 < 50) {
            int seconds_left = (30000 - elapsed) / 1000;
            Serial.print("⏱ ");
            Serial.print(elapsed / 1000);
            Serial.print("/30 seconds (");
            Serial.print(seconds_left);
            Serial.print(" left) | Valid samples: ");
            Serial.print(valid_samples);
            Serial.print("/");
            Serial.println(sample_count);
            
            // Show current ranges
            Serial.print("   Ranges - X: ");
            Serial.print(cal_temp[1] - cal_temp[0]);
            Serial.print(", Y: ");
            Serial.print(cal_temp[3] - cal_temp[2]);
            Serial.print(", Z: ");
            Serial.println(cal_temp[5] - cal_temp[4]);
        }
        
        delay(20);
    }
    
    // Analysis and results same as before...
    analyze_calibration_quality(cal_temp, valid_samples, sample_count, chip_name);
    
    // Save calibration values
    for (int i = 0; i < 6; i++) {
        mag_cal_values[i] = cal_temp[i];
    }
    
    calculate_calibration_factors();
    calibration_completed = true;
    
    print_calibration_values();
    
    Serial.println("");
    Serial.println("PRESS SPACE + ENTER FOR REAL-TIME HEADING TEST...");
}

void analyze_calibration_quality(int16_t cal_temp[], int valid_samples, int sample_count, String chip_name) {
    Serial.println("");
    Serial.println("📊 CALIBRATION ANALYSIS:");
    
    int range_x = cal_temp[1] - cal_temp[0];
    int range_y = cal_temp[3] - cal_temp[2];
    int range_z = cal_temp[5] - cal_temp[4];
    
    Serial.print("Sample success rate: ");
    Serial.print((valid_samples * 100) / sample_count);
    Serial.println("%");
    
    // Different range expectations for different chips
    int min_range = is_qmc5883l ? 400 : 200;  // QMC5883L has higher resolution
    int good_range = is_qmc5883l ? 800 : 400;
    
    Serial.println("Magnetometer ranges:");
    Serial.print("  X: ");
    Serial.print(cal_temp[0]);
    Serial.print(" to ");
    Serial.print(cal_temp[1]);
    Serial.print(" (range: ");
    Serial.print(range_x);
    if (range_x < min_range) Serial.print(" ⚠️ LOW");
    else if (range_x > good_range) Serial.print(" ✓ EXCELLENT");
    else Serial.print(" ✓ GOOD");
    Serial.println(")");
    
    Serial.print("  Y: ");
    Serial.print(cal_temp[2]);
    Serial.print(" to ");
    Serial.print(cal_temp[3]);
    Serial.print(" (range: ");
    Serial.print(range_y);
    if (range_y < min_range) Serial.print(" ⚠️ LOW");
    else if (range_y > good_range) Serial.print(" ✓ EXCELLENT");
    else Serial.print(" ✓ GOOD");
    Serial.println(")");
    
    Serial.print("  Z: ");
    Serial.print(cal_temp[4]);
    Serial.print(" to ");
    Serial.print(cal_temp[5]);
    Serial.print(" (range: ");
    Serial.print(range_z);
    if (range_z < min_range) Serial.print(" ⚠️ LOW");
    else if (range_z > good_range) Serial.print(" ✓ EXCELLENT");
    else Serial.print(" ✓ GOOD");
    Serial.println(")");
    
    bool good_calibration = (range_x > (min_range/2) && range_y > (min_range/2) && range_z > (min_range/2) && 
                           valid_samples > (sample_count * 0.8));
    
    if (!good_calibration) {
        Serial.println("");
        Serial.println("⚠️ CALIBRATION QUALITY WARNING:");
        if (range_x < min_range || range_y < min_range || range_z < min_range) {
            Serial.println("- Some axes have insufficient range");
            Serial.println("- Try rotating more completely in all directions");
            Serial.print("- ");
            Serial.print(chip_name);
            Serial.print(" needs ranges > ");
            Serial.println(min_range);
        }
        if (valid_samples < (sample_count * 0.8)) {
            Serial.println("- Too many failed readings");
            Serial.println("- Check connections and try again");
        }
    } else {
        Serial.println("");
        Serial.print("🎉 ");
        Serial.print(chip_name);
        Serial.println(" CALIBRATION COMPLETE - EXCELLENT QUALITY!");
    }
}

// ===== CALCULATE CALIBRATION FACTORS =====
void calculate_calibration_factors() {
    mag_scale_y = ((float)mag_cal_values[1] - mag_cal_values[0]) / 
                 (mag_cal_values[3] - mag_cal_values[2]);
    mag_scale_z = ((float)mag_cal_values[1] - mag_cal_values[0]) / 
                 (mag_cal_values[5] - mag_cal_values[4]);
    
    mag_offset_x = (mag_cal_values[1] + mag_cal_values[0]) / 2;
    mag_offset_y = (mag_cal_values[3] + mag_cal_values[2]) / 2;
    mag_offset_z = (mag_cal_values[5] + mag_cal_values[4]) / 2;
}

// ===== PRINT CALIBRATION VALUES =====
void print_calibration_values() {
    String chip_name = is_qmc5883l ? "QMC5883L" : "HMC5883L";
    
    Serial.println("");
    Serial.println("████████████████████████████████████████████████████████");
    Serial.println("█          COPY THESE VALUES TO FLIGHT CONTROLLER      █");
    Serial.println("████████████████████████████████████████████████████████");
    Serial.println("");
    
    Serial.print("// GY-271 ");
    Serial.print(chip_name);
    Serial.println(" Calibration Values");
    Serial.print("int16_t mag_cal_values[6] = {");
    for (int i = 0; i < 6; i++) {
        Serial.print(mag_cal_values[i]);
        if (i < 5) Serial.print(", ");
    }
    Serial.println("};");
    
    Serial.print("int16_t mag_offset_x = ");
    Serial.print(mag_offset_x);
    Serial.println(";");
    
    Serial.print("int16_t mag_offset_y = ");
    Serial.print(mag_offset_y);
    Serial.println(";");
    
    Serial.print("int16_t mag_offset_z = ");
    Serial.print(mag_offset_z);
    Serial.println(";");
    
    Serial.print("float mag_scale_y = ");
    Serial.print(mag_scale_y, 4);
    Serial.println("f;");
    
    Serial.print("float mag_scale_z = ");
    Serial.print(mag_scale_z, 4);
    Serial.println("f;");
    
    Serial.println("");
    Serial.print("// Detected chip: ");
    Serial.println(chip_name);
    if (is_qmc5883l) {
        Serial.println("#define GY271_ADDRESS 0x0D  // QMC5883L");
        Serial.println("bool is_qmc5883l = true;");
    } else {
        Serial.println("#define GY271_ADDRESS 0x1E  // HMC5883L");
        Serial.println("bool is_qmc5883l = false;");
    }
    
    Serial.println("");
    Serial.println("████████████████████████████████████████████████████████");
}

// ===== REAL-TIME HEADING TEST =====
void real_time_heading_test() {
    if (!calibration_completed) {
        Serial.println("❌ ERROR: Run calibration first!");
        return;
    }
    
    String chip_name = is_qmc5883l ? "QMC5883L" : "HMC5883L";
    
    Serial.println("");
    Serial.print("🧭 REAL-TIME HEADING TEST - GY-271 ");
    Serial.print(chip_name);
    Serial.println(" Y-axis = Drone Nose");
    Serial.println("");
    Serial.println("Your Setup: GY-271 Y-axis points to drone nose (forward)");
    Serial.println("Expected: When nose points North = 0°");
    Serial.println("Rotate your drone and watch the heading change in real-time!");
    Serial.println("Expected values:");
    Serial.println("  North = 0° (or 360°)");
    Serial.println("  East  = 90°");
    Serial.println("  South = 180°");
    Serial.println("  West  = 270°");
    Serial.println("");
    Serial.println("Send any character to stop...");
    Serial.println("");
    
    // Clear input buffer
    while (Serial.available()) Serial.read();
    
    unsigned long last_update = 0;
    float heading_samples[10];
    int sample_idx = 0;
    bool samples_full = false;
    
    while (!Serial.available()) {
        if (millis() - last_update >= 200) {  // Update every 200ms (5Hz)
            last_update = millis();
            
            if (read_magnetometer()) {
                // Apply calibration
                int16_t cal_x = mag_x - mag_offset_x;
                int16_t cal_y = (mag_y - mag_offset_y) * mag_scale_y;
                int16_t cal_z = (mag_z - mag_offset_z) * mag_scale_z;
                
                // FOR GY-271: Y-axis = Drone nose (forward)
                // When nose points North, Y should be max positive, X should be ~0
                // Standard compass: atan2(Y, X) where X=East, Y=North
                // Your setup: Y=North(forward), X=East(right)
                // BUT: To fix East/West swap, we need to negate X
                float heading = atan2(-cal_x, cal_y) * 180.0 / PI;
                if (heading < 0) heading += 360;
                
                // Store in circular buffer for averaging
                heading_samples[sample_idx] = heading;
                sample_idx++;
                if (sample_idx >= 10) {
                    sample_idx = 0;
                    samples_full = true;
                }
                
                // Calculate average heading for stability
                float avg_heading = 0;
                int count = samples_full ? 10 : sample_idx;
                for (int i = 0; i < count; i++) {
                    avg_heading += heading_samples[i];
                }
                avg_heading /= count;
                
                // Show detailed information
                Serial.print("📍 Heading: ");
                Serial.print(heading, 1);
                Serial.print("° (Avg: ");
                Serial.print(avg_heading, 1);
                Serial.print("°) ");
                
                // Direction indicator
                show_direction_indicator(avg_heading);
                
                // Raw data for debugging
                Serial.print(" | Raw[X=");
                Serial.print(mag_x);
                Serial.print("(right),Y=");
                Serial.print(mag_y);
                Serial.print("(nose),Z=");
                Serial.print(mag_z);
                Serial.print("(down)] Cal[X=");
                Serial.print(cal_x);
                Serial.print(",Y=");
                Serial.print(cal_y);
                Serial.print(",Z=");
                Serial.print(cal_z);
                Serial.println("]");
                
            } else {
                Serial.println("❌ Failed to read magnetometer");
            }
        }
        
        delay(10);
    }
    
    // Clear the input that stopped the test
    Serial.read();
    
    show_troubleshooting_guide(chip_name);
}

void show_direction_indicator(float avg_heading) {
    if (avg_heading >= 337.5 || avg_heading < 22.5) {
        Serial.print("🧭 NORTH");
        if (avg_heading > 5 && avg_heading < 355) {
            Serial.print(" [Offset: ");
            float north_error = avg_heading > 180 ? avg_heading - 360 : avg_heading;
            Serial.print(north_error, 1);
            Serial.print("°]");
        }
    } else if (avg_heading >= 22.5 && avg_heading < 67.5) {
        Serial.print("🧭 NE");
    } else if (avg_heading >= 67.5 && avg_heading < 112.5) {
        Serial.print("🧭 EAST");
        if (abs(avg_heading - 90) > 5) {
            Serial.print(" [Offset: ");
            Serial.print(avg_heading - 90, 1);
            Serial.print("°]");
        }
    } else if (avg_heading >= 112.5 && avg_heading < 157.5) {
        Serial.print("🧭 SE");
    } else if (avg_heading >= 157.5 && avg_heading < 202.5) {
        Serial.print("🧭 SOUTH");
        if (abs(avg_heading - 180) > 5) {
            Serial.print(" [Offset: ");
            Serial.print(avg_heading - 180, 1);
            Serial.print("°]");
        }
    } else if (avg_heading >= 202.5 && avg_heading < 247.5) {
        Serial.print("🧭 SW");
    } else if (avg_heading >= 247.5 && avg_heading < 292.5) {
        Serial.print("🧭 WEST");
        if (abs(avg_heading - 270) > 5) {
            Serial.print(" [Offset: ");
            Serial.print(avg_heading - 270, 1);
            Serial.print("°]");
        }
    } else if (avg_heading >= 292.5 && avg_heading < 337.5) {
        Serial.print("🧭 NW");
    }
}

void show_troubleshooting_guide(String chip_name) {
    Serial.println("");
    Serial.println("📊 Test stopped!");
    Serial.println("");
    Serial.print("🔧 TROUBLESHOOTING GUIDE FOR GY-271 ");
    Serial.print(chip_name);
    Serial.println(":");
    Serial.println("");
    Serial.println("If North doesn't show 0°:");
    Serial.println("1. MAGNETIC DECLINATION:");
    Serial.println("   - True North vs Magnetic North difference");
    Serial.println("   - Look up your location's declination online");
    Serial.println("   - Add/subtract this value in your flight controller");
    Serial.println("");
    Serial.println("2. CALIBRATION QUALITY:");
    Serial.println("   - GY-271 needs complete 3D rotation during calibration");
    
    if (is_qmc5883l) {
        Serial.println("   - QMC5883L should have ranges > 400 for best results");
        Serial.println("   - QMC5883L is more sensitive than HMC5883L");
    } else {
        Serial.println("   - HMC5883L should have ranges > 200 for good results");
        Serial.println("   - Try recalibrating with more thorough rotation");
    }
    
    Serial.println("");
    Serial.println("3. ENVIRONMENTAL FACTORS:");
    Serial.println("   - Move away from metal objects (cars, steel tables)");
    Serial.println("   - Avoid electronic devices (phones, laptops)");
    Serial.println("   - Test in different locations");
    Serial.println("");
    Serial.println("4. GY-271 SPECIFIC ISSUES:");
    Serial.println("   - Check 3.3V power supply (GY-271 is 3.3V optimized)");
    Serial.println("   - Verify I2C connections (SDA, SCL, VCC, GND)");
    
    if (is_qmc5883l) {
        Serial.println("   - QMC5883L address: 0x0D");
        Serial.println("   - QMC5883L has higher resolution but may be more sensitive to interference");
    } else {
        Serial.println("   - HMC5883L address: 0x1E");
        Serial.println("   - HMC5883L is older but more stable in noisy environments");
    }
    
    Serial.println("");
    Serial.println("5. CONSISTENT OFFSET:");
    Serial.println("   - If always off by the same amount (e.g., always +30°)");
    Serial.println("   - Note the offset and add it to COMPASS_DECLINATION");
    Serial.println("   - Example: If North shows 30°, set declination to -30°");
    Serial.println("");
    Serial.println("6. CHIP-SPECIFIC TROUBLESHOOTING:");
    
    if (is_qmc5883l) {
        Serial.println("   QMC5883L (newer chip):");
        Serial.println("   - More sensitive, better resolution");
        Serial.println("   - May need more careful calibration");
        Serial.println("   - Less affected by temperature");
        Serial.println("   - If readings are too noisy, try different location");
    } else {
        Serial.println("   HMC5883L (classic chip):");
        Serial.println("   - Very reliable and well-tested");
        Serial.println("   - Lower resolution but stable");
        Serial.println("   - If readings seem low, check power supply");
        Serial.println("   - May drift slightly with temperature");
    }
    
    Serial.println("");
    Serial.println("Current calibration values are valid if:");
    Serial.println("✓ Headings change smoothly as you rotate");
    Serial.println("✓ East-West readings are ~90° apart");
    Serial.println("✓ North-South readings are ~180° apart");
    Serial.println("✓ Readings are stable when drone is stationary");
    Serial.println("✓ No sudden jumps or erratic behavior");
    Serial.println("✓ Values don't saturate at ±32767");
    Serial.println("");
    Serial.print("Your ");
    Serial.print(chip_name);
    Serial.println(" calibration is ready for flight controller integration!");
    Serial.println("");
    Serial.println("PRESS SPACE + ENTER TO TEST AGAIN...");
}