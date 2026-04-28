#include <Arduino.h>
#include <SPI.h>
#include <math.h>

#define MISO 19
#define MOSI 23
#define SCK 18
#define SS 5

#define ADE_CLKIN 8000000.0
#define PERIOD_LSB (96.0 / ADE_CLKIN)
#define VRMS_FULL_SCALE_50HZ 1678210.0
#define VRMS_INPUT_FULL_SCALE 0.3535
#define VOLTAGE_DIVIDER_RATIO 350.0
#define VRMS_NOISE_THRESHOLD 10000
#define SPI_CLOCK_SPEED 1000000

#define STATUS 0x19
#define RSTATUS 0x1A
#define MMODE 0x14
#define LCYCMODE 0x17
#define FREQ 0x10
#define AVRMS 0x0D
#define BVRMS 0x0E
#define CVRMS 0x0F
#define AVRMSOS 0x0A
#define BVRMSOS 0x0B
#define CVRMSOS 0x0C
#define COMPMODE 0x16
#define WAVMODE 0x15
#define WRITE 0x80

enum PhaseConfig {
    ABC = 0x1C,  // A->A, B->B, C->C
    ACB = 0x1D,  // A->A, C->B, B->C
    BAC = 0x1E,  // B->A, A->B, C->C
    BCA = 0x1F,  // B->A, C->B, A->C
    CAB = 0x20,  // C->A, A->B, B->C
    CBA = 0x21   // C->A, B->B, A->C
};

class ADE7758 {
private:
    int CS;
    PhaseConfig currentConfig;

    void enable() {
        digitalWrite(CS, LOW);
        delayMicroseconds(1);
        SPI.beginTransaction(SPISettings(SPI_CLOCK_SPEED, MSBFIRST, SPI_MODE1));
    }

    void disable() {
        delayMicroseconds(1);
        SPI.endTransaction();
        digitalWrite(CS, HIGH);
        delayMicroseconds(1);
    }

public:
    ADE7758(int _CS) : CS(_CS), currentConfig(ABC) {}

    void begin() {
        pinMode(CS, OUTPUT);
        digitalWrite(CS, HIGH);
        SPI.begin();
        SPI.setClockDivider(SPI_CLOCK_DIV128);
        SPI.setDataMode(SPI_MODE1);
        SPI.setBitOrder(MSBFIRST);
        delay(100);
    }

    bool setPhaseConfiguration(PhaseConfig config) {
        write8(0x13, 0x40);  // Reset chip
        delay(100);
        write8(COMPMODE, config);
        delay(50);
        
        unsigned char readConfig = read8bits(COMPMODE);
        if (readConfig != config) {
            Serial.println("Lỗi: Không thể cấu hình pha");
            return false;
        }
        
        currentConfig = config;
        return true;
    }

    unsigned char read8bits(char reg) {
        enable();
        SPI.transfer(reg);
        delayMicroseconds(5);
        unsigned char value = SPI.transfer(0x00);
        disable();
        return value;
    }

    unsigned int read16bits(char reg) {
        enable();
        SPI.transfer(reg);
        delayMicroseconds(5);
        unsigned char b1 = SPI.transfer(0x00);
        delayMicroseconds(5);
        unsigned char b0 = SPI.transfer(0x00);
        disable();
        return ((unsigned int)b1 << 8) | b0;
    }

    unsigned long read24bits(char reg) {
        enable();
        SPI.transfer(reg);
        delayMicroseconds(5);
        unsigned char b2 = SPI.transfer(0x00);
        delayMicroseconds(5);
        unsigned char b1 = SPI.transfer(0x00);
        delayMicroseconds(5);
        unsigned char b0 = SPI.transfer(0x00);
        disable();
        return ((unsigned long)b2 << 16) | ((unsigned long)b1 << 8) | b0;
    }

    void write8(char reg, unsigned char data) {
        enable();
        reg |= WRITE;
        SPI.transfer(reg);
        delayMicroseconds(5);
        SPI.transfer(data);
        disable();
    }

    void write16(char reg, unsigned int data) {
        enable();
        reg |= WRITE;
        SPI.transfer(reg);
        delayMicroseconds(5);
        SPI.transfer((data >> 8) & 0xFF);
        delayMicroseconds(5);
        SPI.transfer(data & 0xFF);
        disable();
    }

    void write24(char reg, unsigned long data) {
        enable();
        reg |= WRITE;
        SPI.transfer(reg);
        delayMicroseconds(5);
        SPI.transfer((data >> 16) & 0xFF);
        delayMicroseconds(5);
        SPI.transfer((data >> 8) & 0xFF);
        delayMicroseconds(5);
        SPI.transfer(data & 0xFF);
        disable();
    }

    double getFrequency() {
        write8(MMODE, 0xFC);
        write8(LCYCMODE, 0xF8);
        write8(WAVMODE, 0x14);
        
        delay(200);
        
        unsigned long per_sum = 0;
        int valid_readings = 0;
        const int NUM_SAMPLES = 30;
        
        for (int i = 0; i < NUM_SAMPLES; i++) {
            unsigned int per = read16bits(FREQ) & 0x0FFF;
            if (per > 0 && per <= 0x0FFF) {
                per_sum += per;
                valid_readings++;
            }
            delay(200);
        }
        
        if (valid_readings == 0) {
            Serial.println("TAN SO NGOAI");
            return 0.0;
        }
        
        double period_s = (per_sum / valid_readings) * PERIOD_LSB;
        if (period_s == 0) {
            Serial.println("CHU KY 0");
            return 0.0;
        }
        
        double freq = 1 / period_s;
        
        if (freq < 20.0 || freq > 90.0) {
            Serial.println("TAN SO NGOAI");
            return 0.0;
        }
        return freq;
    }

    long getVRMS(int phase) {
        int N = 60;
        unsigned long VRMS = 0;
        int validReadings = 0;
        
        for (int i = 0; i < N; i++) {
            long raw_vrms = read24bits(AVRMS + phase);
            if (raw_vrms > 0) {
                VRMS += raw_vrms;
                validReadings++;
            }
            delayMicroseconds(16667 / N);
        }
        
        if (validReadings == 0) {
            return -1;
        }
        
        long average = VRMS / validReadings;
        if (average < VRMS_NOISE_THRESHOLD) {
            return 0;
        }
        return average;
    }

    void calibrateZeroVRMSOS(int phase) {
        long VRMS_zero_sum = 0;
        int validSamples = 0;
        const int NUM_SAMPLES = 10;
        
        for (int i = 0; i < NUM_SAMPLES; i++) {
            long sample = getVRMS(phase);
            if (sample != -1) {
                VRMS_zero_sum += sample;
                validSamples++;
            }
            delay(100);
        }
        
        if (validSamples == 0) {
            Serial.println("Lỗi hiệu chuẩn điểm 0");
            return;
        }
        
        long VRMS_zero = VRMS_zero_sum / validSamples;
        double Voffset = -(double)VRMS_zero / 64.0;
        int Voffset_int = (int)round(Voffset);
        
        if (Voffset_int > 2047) Voffset_int = 2047;
        if (Voffset_int < -2048) Voffset_int = -2048;
        
        write16(AVRMSOS + phase, (unsigned int)(Voffset_int & 0xFFFF));
    }
};

ADE7758 ade(SS);

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("Khởi tạo ADE7758...");
    ade.begin();
    
    if (!ade.setPhaseConfiguration(ABC)) {
        Serial.println("Lỗi cấu hình pha");
        return;
    }
    
    ade.write8(WAVMODE, 0x14);
    delay(50);
    ade.write8(MMODE, 0x1C);
    delay(50);
    ade.write8(LCYCMODE, 0xF8);
    delay(50);
    ade.write24(0x18, 0xE00);
    delay(200);

    Serial.println("Hiệu chuẩn điểm 0...");
    ade.calibrateZeroVRMSOS(0);
    delay(500);
    ade.calibrateZeroVRMSOS(1);
    delay(500);
    ade.calibrateZeroVRMSOS(2);
    
    Serial.println("Khởi tạo hoàn tất");
}

void loop() {
    double freq = ade.getFrequency();
    Serial.print("FREQ ");
    Serial.print(freq);
    Serial.println(" Hz");

    if (freq > 0) {
        long vrms_raw_a = ade.getVRMS(0);
        long vrms_raw_b = ade.getVRMS(1);
        long vrms_raw_c = ade.getVRMS(2);

        if (vrms_raw_a != -1) {
            double vrms_a = (vrms_raw_a / VRMS_FULL_SCALE_50HZ) * VRMS_INPUT_FULL_SCALE * VOLTAGE_DIVIDER_RATIO;
            Serial.print("VRMS (Phase A): ");
            Serial.print(vrms_a);
            Serial.println(" V");
        }

        if (vrms_raw_b != -1) {
            double vrms_b = (vrms_raw_b / VRMS_FULL_SCALE_50HZ) * VRMS_INPUT_FULL_SCALE * VOLTAGE_DIVIDER_RATIO;
            Serial.print("VRMS (Phase B): ");
            Serial.print(vrms_b);
            Serial.println(" V");
        }

        if (vrms_raw_c != -1) {
            double vrms_c = (vrms_raw_c / VRMS_FULL_SCALE_50HZ) * VRMS_INPUT_FULL_SCALE * VOLTAGE_DIVIDER_RATIO;
            Serial.print("VRMS (Phase C): ");
            Serial.print(vrms_c);
            Serial.println(" V");
        }
    } else {
       
        Serial.println("VRMS (Phase A): 0.00 V");
        Serial.println("VRMS (Phase B): 0.00 V");
        Serial.println("VRMS (Phase C): 0.00 V");
    }

    Serial.println("----------------------------------------");
    delay(1000);
}