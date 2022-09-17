
#include <Arduino.h>

#include "JmaIntensity.hpp"

#include "Filter.hpp"
#include <cmath>
#ifdef ACE_SORTING
#include <AceSorting.h>
using ace_sorting::shellSortKnuth;
#endif
#ifndef ACE_SORTING
#include <ArduinoSort.h>
#endif

#ifdef MMA8451
#include <Adafruit_MMA8451.h>
Adafruit_MMA8451 mma = Adafruit_MMA8451();
#endif
#ifdef MCP3204
#include "MCP3204.hpp"
MCP3204 ADC = MCP3204(SPISettings(115200, MSBFIRST, SPI_MODE0), D21);
#endif
#ifdef LED
#include "LED.hpp"
Led LED = Led();
#endif

#define SAMPLING_RATE 100

// 震度を計算する頻度(1 ~ 100/秒)
const int CALC_INTENSITY_RATE = 10;
// 加速度を出力する頻度(0 ~ 100/秒)
const int ACC_REPORT_RATE = 100;

auto ADJUST_PIN = 16;

const int samplingRate = SAMPLING_RATE;
const int bufferSize = samplingRate;

Filter FILTER = Filter(samplingRate);

void setup()
{
    pinMode(ADJUST_PIN, INPUT_PULLDOWN);

    Serial.begin(115200);

#ifdef MMA8451
    if (!mma.begin())
    {
        Serial.println("Couldnt start");
        while (1)
            ;
    }
    mma.setRange(MMA8451_RANGE_2_G);
    mma.writeRegister8(0x2A, 0x00 | 0x01 | 0x04); // LNOISE
    mma.writeRegister8(0x2B, 0x10);
#endif

#ifndef MMA8451
    // TODO: これクラスの中に持っていったほうが良さそう
    SPI.setRX(D20);
    SPI.setCS(D21);
    SPI.setSCK(D18);
    SPI.setTX(D19);
    SPI.begin();
#endif

#ifdef LED
    LED.wakeup();
#endif
}

// メモ: $はつけない
void printNmea(const char *format, ...)
{

    va_list arg;
    va_start(arg, format);
    char temp[64];
    char *buffer = temp;
    size_t len = vsnprintf(temp, sizeof(temp), format, arg);
    va_end(arg);
    if (len > sizeof(temp) - 1)
    {
        buffer = new char[len + 1];
        if (!buffer)
            return;
        va_start(arg, format);
        vsnprintf(buffer, len + 1, format, arg);
        va_end(arg);
    }
    byte checkSum = 0;
    for (int i = 0; buffer[i]; i++)
        checkSum ^= (byte)buffer[i];
    Serial.printf("$%s*%02X\r\n", buffer, checkSum);
    if (buffer != temp)
    {
        delete[] buffer;
    }
    Serial.flush();
}

// 3-axis adjust value
uint16_t offset[3];
// Adujusting time( offset_counter / sampling rate = sec)
int offsetCounter = bufferSize * 1;
bool isOffsetted = false;

#ifdef MCP3204
// オフセットを計算する
void calcOffset(uint16_t *buf)
{
    offset[0] = buf[0];
    offset[1] = buf[1];
    offset[2] = buf[2];
}
#endif

#ifdef MMA8451
void calcOffset(float *buf)
{
    offset[0] = buf[0];
    offset[1] = buf[1];
    offset[2] = buf[2];
}
#endif

float computePGA(float *HPFilteredData)
{
    float pga = 0;
    float acc = 0;
    for (int i = 0; i < samplingRate; i++)
    {
        acc = sqrt(HPFilteredData[i] * HPFilteredData[i] + HPFilteredData[i + 1] * HPFilteredData[i + 1] + HPFilteredData[i + 2] * HPFilteredData[i + 2]);
        if (acc > pga)
            pga = acc;
    }

    return pga;
}

const int RETENTION_MICRO_SECONDS = 60 * 10 * 1000000;
int x = 0, y = 0, z = 0;

#ifdef MCP3204
uint16_t rawData[3];
#endif

#ifdef MMA8451
float rawData[3];
#endif

float compositeData[bufferSize];
float compositeSortedData[bufferSize];
float HPFilteredData[bufferSize * 3];

unsigned long frame = 0;

unsigned long latestMaxTime;
JmaIntensity latestIntensity;
JmaIntensity maxIntensity;

void loop()
{

    auto startTime = micros();

    uint16_t index = frame % bufferSize;

// 計測した値を取得
#ifdef MMA8451
    mma.read();
    sensors_event_t event;
    mma.getEvent(&event);
#endif

    for (auto a = 0; a < 3; a++)
    {

#ifdef MMA8451
        rawData[a] = (a == 0) ? abs(event.acceleration.x) : (a == 1) ? abs(event.acceleration.y)
                                                                : abs(event.acceleration.z);
#endif
#ifdef MCP3204
        uint16_t v = ADC.read(a);
        // 極値を弾く
        if (v < 0 || v > 4096 || v == 1024 || v == 2048 || v == 3072)
            v = (uint16_t)offset[a];
        rawData[a] = v;

#endif
        //Serial.println(rawData[a]);
        // Serial.println(v);
    }

    // オフセット計算
    if (!isOffsetted)
        calcOffset(rawData);

    float offsetSample[3];
    float newHPFilteredSample[3];
    float filteredDataSample[3];

    for (auto i = 0; i < 3; i++)
    {

#ifdef MCP3204
        offsetSample[i] = (rawData[i] - offset[i]) / 1024.0f * 981;
#endif
#ifdef MMA8451
        offsetSample[i] = (rawData[i] * 100.0);
#endif
        newHPFilteredSample[i] = offsetSample[i];
        filteredDataSample[i] = offsetSample[i];
    }

    FILTER.filterHP(newHPFilteredSample);
    FILTER.filterForShindo(filteredDataSample);

    for (int i = 0; i < 3; i++)
    {
        HPFilteredData[index * 3 + i] = newHPFilteredSample[i];
    }

    if (isOffsetted && ACC_REPORT_RATE > 0 && frame % (samplingRate / ACC_REPORT_RATE) == 0)
        printNmea("XSACC,%.3f,%.3f,%.3f", HPFilteredData[index * 3], HPFilteredData[index * 3 + 1], HPFilteredData[index * 3 + 2]);

    // 3軸合成
    compositeData[index] = sqrt(
        filteredDataSample[0] * filteredDataSample[0] +
        filteredDataSample[1] * filteredDataSample[1] +
        filteredDataSample[2] * filteredDataSample[2]);

    if (isOffsetted && frame % (samplingRate / CALC_INTENSITY_RATE) == 0)
    {
        memcpy(compositeSortedData, compositeData, sizeof(compositeData[0]) * bufferSize);

#ifdef ACE_SORTING
        shellSortKnuth(compositeSortedData, bufferSize);
#endif
#ifndef ACE_SORTING
        sortArray(compositeSortedData, bufferSize);
#endif

        auto gal = compositeSortedData[(int)(samplingRate * 0.7)];
        float pga = computePGA(HPFilteredData);
        printNmea("XSPGA,%.3f", pga);

        if (gal > 0)
        {
            auto rawInt = round((2.0f * log10(gal) + 0.94f) * 10.0f) / 10.0f;
            printNmea("XSINT,%.3f,%.2f", gal, rawInt);
            latestIntensity = getJmaIntensity(rawInt);

            if (micros() - latestMaxTime > RETENTION_MICRO_SECONDS || maxIntensity <= latestIntensity)
            {
                latestMaxTime = micros();
                maxIntensity = latestIntensity;
            }
#ifdef LED
            LED.blinkScale(latestIntensity, maxIntensity);
#endif
        }
        else
        {
            printNmea("XSINT,%.3f,", gal);
#ifdef LED
            LED.clear();
#endif
        }
    }

    if (isOffsetted && frame % (samplingRate / 4) == 0 && latestIntensity < maxIntensity)
#ifdef LED
        LED.toggle(maxIntensity);
#endif

    if (!isOffsetted)
    {
        auto v = (JmaIntensity)(offsetCounter * 100 / (samplingRate * 5) % 10);
#ifdef LED
        LED.blinkScale(v, v);
#endif
    }

    if (frame >= samplingRate * 60)
        frame = 0;

    frame++;

    auto workTime = micros() - startTime;
    auto sleepTime = 1000000 / samplingRate - workTime;
    if (sleepTime > 0)
#ifdef ESP32
        delayMicroseconds(sleepTime);
#endif
#ifndef ESP32
    sleep_us(sleepTime);
#endif

    // if (frame % samplingRate == 0)
    // {
    //     printNmea("XTIME,%d,%d", workTime, sleepTime);
    // }
    if (digitalRead(ADJUST_PIN) && isOffsetted)
    {
        isOffsetted = false;
        offsetCounter = samplingRate * 2;
        latestMaxTime = micros();
        printNmea("XSOFF,1");
    }
    else if (!isOffsetted && offsetCounter-- <= 0)
    {
        maxIntensity = JMA_INT_0;
        FILTER.reset();
        isOffsetted = true;
        printNmea("XSOFF,0");
    }
}
