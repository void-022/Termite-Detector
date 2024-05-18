#include <TFT_eSPI.h>
#include <SPI.h>
#include <Wire.h>
#include "termite.h"
#include "no_termite.h"
#include "logo.h"
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"
#include "font.h"

#define TFT_CS 15
#define TFT_DC 2
#define TFT_MOSI 23
#define TFT_CLK 18
#define TFT_RST 4
#define TFT_GREY 0x5AEB

#define analogPin 34
TFT_eSPI tft = TFT_eSPI();

const byte MLX90640_address = 0x33;

#define TA_SHIFT 8

static float mlx90640To[768];
static float mlx90640scaled[768];
paramsMLX90640 mlx90640;
int xPos, yPos;
int R, G, B;
int i, j;
float T_max, T_min;
float T_center;
float sumMLX;
float T_avg;

float sumInside = 0.0;
float sumOutside = 0.0;
float avgInside;
float avgOutside;
float diffAvg;
bool tempVar = 0;

byte upButtonPin = 27;
byte downButtonPin = 17;
boolean upButtonState = LOW;
boolean downButtonState;
byte mode = 0;

boolean stopFlag = 0;
boolean finalButtonState;
int finalAvgdB;
float rcwlPercent = 0.0;
float maxPercent = 0.0;
float mlxPercent = 0.0;
float overallPercent = 0.0;

byte rcwlPin = 32;
const unsigned long loadingInterval = 1000;
unsigned long loadingDuration = 0;
unsigned long previousTime = 0;
int loadingXAxis = 120;
byte loadingDotsCounter = 0;

unsigned int analogValue;
int scaled;
const int sampleWindow = 50;
float peakToPeak = 0;
unsigned long startMillis;
unsigned int signalMax;
unsigned int signalMin;

const int listeningInterval = 180;
long sum = 0;
int soundSamples = 0;
float ltx = 0;
uint16_t osx = 120, osy = 120;
uint32_t updateTime = 0;
int old_analog = -999;
int old_digital = -999;
int value[6] = { 0, 0, 0, 0, 0, 0 };
int old_value[6] = { -1, -1, -1, -1, -1, -1 };
int d = 0;
int avg;

void setup() {
  Serial.begin(115200);

  Wire.begin(21, 22);
  Wire.setClock(400000);

  pinMode(rcwlPin, INPUT);
  pinMode(upButtonPin, INPUT_PULLUP);
  pinMode(downButtonPin, INPUT_PULLUP);
  tft.init();
  tft.setSwapBytes(true);
  tft.setRotation(3);
  tft.fillScreen(TFT_WHITE);

  while (!Serial)
    ;
  Serial.println("MLX90640 IR Array Example");
  if (isConnected() == false) {
    Serial.println("MLX90640 not detected at default I2C address. Please check wiring. Freezing.");
    while (1)
      ;
  }

  Serial.println("MLX90640 online!");
  int status;
  uint16_t eeMLX90640[832];
  status = MLX90640_DumpEE(MLX90640_address, eeMLX90640);

  if (status != 0)
    Serial.println("Failed to load system parameters");
  status = MLX90640_ExtractParameters(eeMLX90640, &mlx90640);

  if (status != 0) {
    Serial.println("Parameter extraction failed");
    Serial.print(" status = ");
    Serial.println(status);
  }
  MLX90640_I2CWrite(0x33, 0x800D, 6401);
  MLX90640_SetRefreshRate(MLX90640_address, 0x04);
}

void loop() {
  Serial.print("   Mode: ");
  Serial.print(mode);
  Serial.print("  ");

  boolean buttonState = digitalRead(rcwlPin);
  unsigned long currentTime = millis();

  if (debounceButton(upButtonState, upButtonPin) == LOW && upButtonState == HIGH && mode < 4) {
    mode++;
    upButtonState = LOW;
  } else if (debounceButton(upButtonState, upButtonPin) == HIGH && upButtonState == LOW) {
    upButtonState = HIGH;
  } else if (debounceButton(downButtonState, downButtonPin) == LOW && downButtonState == HIGH && mode > 1) {
    mode--;
    downButtonState = LOW;
  } else if (debounceButton(downButtonState, downButtonPin) == HIGH && downButtonState == LOW) {
    downButtonState = HIGH;
  }

  if (mode == 0) {
    tft.pushImage(0, 0, 320, 240, logo);
  } else if (mode == 1) {
    if ((debounceButton(upButtonState, upButtonPin) == LOW && mode == 1) || (debounceButton(downButtonState, downButtonPin) == LOW && mode == 1)) {
      tft.fillScreen(TFT_WHITE);
      tft.pushImage(0, 0, 320, 240, termite);
      tft.setFreeFont(&Roboto_Medium_Italic_13);
      tft.setTextColor(TFT_BLACK, TFT_WHITE);
      tft.drawString("Please hold the device steady", 63, 220);
      loadingDotsCounter = 0;
      loadingXAxis = 120;
      loadingDuration = currentTime + 8000;
      previousTime = currentTime;
      buttonState = 0;
    }

    Serial.print("   RCWL output: ");
    Serial.print(buttonState);

    if (currentTime < loadingDuration) {
      if ((currentTime - previousTime >= loadingInterval) && loadingDotsCounter < 3) {
        tft.fillCircle(loadingXAxis, 190, 10, ILI9341_GREEN);
        loadingXAxis = loadingXAxis + 40;
        loadingDotsCounter++;
        previousTime = currentTime;
      } else if ((currentTime - previousTime >= loadingInterval) && (loadingDotsCounter > 2)) {
        tft.fillCircle(120, 190, 10, ILI9341_WHITE);
        tft.fillCircle(160, 190, 10, ILI9341_WHITE);
        tft.fillCircle(200, 190, 10, ILI9341_WHITE);
        previousTime = currentTime;
        loadingXAxis = 120;
        loadingDotsCounter = 0;
      }
    }

    else if (buttonState == LOW) {
      if (stopFlag == 0) {
        finalButtonState = LOW;
        tft.pushImage(0, 0, 320, 240, no_termite);
        tft.setTextColor(TFT_GREEN);
        tft.setFreeFont(&Roboto_Medium_15);
        tft.drawString("No Termite motion detected", 55, 190);
        tft.setTextColor(TFT_BLACK);
        tft.setFreeFont(&Roboto_Medium_Italic_13);
        tft.drawString("Press button for next mode", 72, 210);
      }
      stopFlag = 1;
    } else if (buttonState == HIGH) {
      if (stopFlag == 0) {
        finalButtonState = HIGH;
        tft.pushImage(0, 0, 320, 240, termite);
        tft.setTextColor(TFT_RED);
        tft.setFreeFont(&Roboto_Medium_15);
        tft.drawString("Termite motion detected", 67, 190);
        tft.setTextColor(TFT_BLACK);
        tft.setFreeFont(&Roboto_Medium_Italic_13);
        tft.drawString("Press button for next mode", 72, 210);
      }
      stopFlag = 1;
    }

  } else if (mode == 2) {

    if ((debounceButton(upButtonState, upButtonPin) == LOW && mode == 2) || (debounceButton(downButtonState, downButtonPin) == LOW && mode == 2)) {
      tft.fillScreen(TFT_BLACK);
      loadingDotsCounter = 0;
      loadingXAxis = 120;
      loadingDuration = currentTime + 9000;
      previousTime = currentTime;
      sumInside = 0.0;
      sumOutside = 0.0;
      avgInside = 0.0;
      avgOutside = 0.0;
      stopFlag = 0;
      tempVar = 0;
      diffAvg = 0;
    }
    sumMLX = 0.0;
    T_avg = 0.0;
    for (byte x = 0; x < 2; x++) {
      uint16_t mlx90640Frame[834];
      int status = MLX90640_GetFrameData(MLX90640_address, mlx90640Frame);

      if (status < 0) {
        Serial.print("GetFrame Error: ");
        Serial.println(status);
      }

      float vdd = MLX90640_GetVdd(mlx90640Frame, &mlx90640);
      float Ta = MLX90640_GetTa(mlx90640Frame, &mlx90640);
      float tr = Ta - TA_SHIFT;
      float emissivity = 0.95;
      MLX90640_CalculateTo(mlx90640Frame, &mlx90640, emissivity, tr, mlx90640To);
    }

    T_min = mlx90640To[0];
    T_max = mlx90640To[0];

    for (i = 1; i < 768; i++) {
      if ((mlx90640To[i] > -41) && (mlx90640To[i] < 301)) {
        if (mlx90640To[i] < T_min) {
          T_min = mlx90640To[i];
        }

        if (mlx90640To[i] > T_max) {
          T_max = mlx90640To[i];
        }
      } else if (i > 0) {
        mlx90640To[i] = mlx90640To[i - 1];
      } else {
        mlx90640To[i] = mlx90640To[i + 1];
      }
    }

    if (currentTime < loadingDuration) {
      if ((currentTime - previousTime >= loadingInterval) && loadingDotsCounter < 3) {
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.setFreeFont(&Roboto_Light_Italic_12);
        tft.drawString("Please hold the device steady", 68, 5);
        tft.fillCircle(loadingXAxis, 215, 10, TFT_GREEN);
        loadingXAxis = loadingXAxis + 40;
        loadingDotsCounter++;
        previousTime = currentTime;
      } else if ((currentTime - previousTime >= loadingInterval) && (loadingDotsCounter > 2)) {
        tft.fillCircle(120, 215, 10, TFT_BLACK);
        tft.fillCircle(160, 215, 10, TFT_BLACK);
        tft.fillCircle(200, 215, 10, TFT_BLACK);
        previousTime = currentTime;
        loadingXAxis = 120;
        loadingDotsCounter = 0;
      }
      stopFlag = 0;
    } else {
      stopFlag = 1;
    }

    if (stopFlag == 1 && tempVar == 0) {
      for (i = 0; i < 24; i++) {
        for (j = 0; j < 32; j++) {
          if (isInside(i, j)) {

            sumInside += mlx90640To[i * 32 + j];
          } else {

            sumOutside += mlx90640To[i * 32 + j];
          }
        }
      }

      avgInside = sumInside / 260;
      avgOutside = sumOutside / 508;
      diffAvg = abs(avgInside - avgOutside);

      tft.setTextColor(TFT_GREEN, TFT_BLACK);
      tft.setFreeFont(&Roboto_Light_Italic_12);
      tft.drawString("Please hold the device flush against the wood", 15, 5);

      tft.setTextColor(TFT_GREEN, TFT_BLACK);
      tft.setFreeFont(&Roboto_Medium_15);
      tft.drawString("Temp. Avg.: " + String(avgInside), 92, 200);

      tft.setTextColor(TFT_WHITE, TFT_BLACK);
      tft.setFreeFont(&Roboto_Medium_Italic_13);
      tft.drawString("Press button for next mode", 72, 220);

      tempVar = 1;
    }

    for (i = 0; i < 24; i++) {
      for (j = 0; j < 32; j++) {
        mlx90640scaled[i * 32 + j] = 180.0 * (mlx90640To[i * 32 + j] - T_min) / (T_max - T_min);
        getColor(mlx90640scaled[i * 32 + j]);
        sumMLX = sumMLX + mlx90640To[i * 32 + j];
        tft.fillRect((217 - j * 7) + 48, 20 + i * 7, 7, 7, tft.color565(R, G, B));
      }
    }
    stopFlag = 0;

    tft.drawLine(90, 55, 112, 55, TFT_GREEN);
    tft.drawLine(90, 55, 90, 77, TFT_GREEN);
    tft.drawLine(210, 55, 230, 55, TFT_GREEN);
    tft.drawLine(230, 55, 230, 77, TFT_GREEN);
    tft.drawLine(90, 126, 90, 146, TFT_GREEN);
    tft.drawLine(90, 146, 112, 146, TFT_GREEN);
    tft.drawLine(230, 126, 230, 146, TFT_GREEN);
    tft.drawLine(230, 146, 210, 146, TFT_GREEN);
    tft.drawCircle(160, 102, 4, TFT_GREEN);

    Serial.print("    Outside Temp: ");
    Serial.print(avgOutside);
    Serial.print("    Inside Temp: ");
    Serial.print(avgInside);

  } else if (mode == 3) {
    Serial.print(maxPercent);
    if ((debounceButton(upButtonState, upButtonPin) == LOW && mode == 3) || (debounceButton(downButtonState, downButtonPin) == LOW && mode == 3)) {
      tft.fillScreen(TFT_WHITE);
      analogMeter();
      finalAvgdB = 0;
      sum = 0;
      avg = 0;
      finalAvgdB = 0;
      peakToPeak = 0;
      signalMax = 0;
      signalMin = 4095;
      soundSamples = 0;
      loadingDotsCounter = 0;
      loadingXAxis = 120;
      loadingDuration = currentTime + 9000;
      previousTime = currentTime;
    }

    d += 4;
    if (d >= 360) d = 0;

    signalMax = 0;
    signalMin = 4095;
    startMillis = millis();
    while (millis() - startMillis < sampleWindow) {
      analogValue = analogRead(analogPin);
      if (analogValue <= 4095) {
        if (analogValue > signalMax) {
          signalMax = analogValue;
        } else if (analogValue < signalMin) {
          signalMin = analogValue;
        }
      }
    }

    peakToPeak = signalMax - signalMin;
    scaled = map(peakToPeak, 0, 4095, 10, 100);

    Serial.print(analogValue);
    Serial.print("      Scaled Value: ");
    Serial.print(scaled);
    plotNeedle(scaled, 0);
    Serial.print("    Average dB value: ");
    Serial.print(avg);

    if (currentTime < loadingDuration) {
      if ((currentTime - previousTime >= listeningInterval) && loadingDotsCounter < 3) {
        tft.setTextColor(TFT_BLACK, TFT_WHITE);
        tft.setFreeFont(&Roboto_Medium_Italic_13);
        tft.drawString("Please hold the device flush against the wood", 10, 200);
        sum = sum + scaled;
        soundSamples++;
        avg = sum / soundSamples;
        previousTime = currentTime;
        tft.fillCircle(loadingXAxis, 180, 10, TFT_GREEN);
        loadingXAxis = loadingXAxis + 40;
        loadingDotsCounter++;
        previousTime = currentTime;
      } else if ((currentTime - previousTime >= listeningInterval) && (loadingDotsCounter > 2)) {
        sum = sum + scaled;
        soundSamples++;
        avg = sum / soundSamples;
        previousTime = currentTime;
        tft.fillCircle(120, 180, 10, TFT_WHITE);
        tft.fillCircle(160, 180, 10, TFT_WHITE);
        tft.fillCircle(200, 180, 10, TFT_WHITE);
        previousTime = currentTime;
        loadingXAxis = 120;
        loadingDotsCounter = 0;
      }
    } else {
      finalAvgdB = avg;
      tft.setTextColor(TFT_WHITE);
      tft.setFreeFont(&Roboto_Medium_Italic_13);
      tft.drawString("Please hold the device flush against the wood", 10, 200);
      tft.fillCircle(120, 180, 10, TFT_WHITE);
      tft.fillCircle(160, 180, 10, TFT_WHITE);
      tft.fillCircle(200, 180, 10, TFT_WHITE);
      tft.setTextColor(TFT_GREEN);
      tft.setFreeFont(&Roboto_Medium_15);
      tft.drawString("Sound registered successfully", 45, 170);
      tft.setTextColor(TFT_BLACK);
      tft.setFreeFont(&Roboto_Medium_Italic_13);
      tft.drawString("Press button for results", 75, 190);
    }

  } else if (mode == 4) {
    if ((debounceButton(upButtonState, upButtonPin) == LOW && mode == 4) || (debounceButton(downButtonState, downButtonPin) == LOW && mode == 4)) {
      overallPercent = 0;
      tft.fillScreen(TFT_WHITE);
      tft.setTextColor(TFT_BLACK);
      tft.setFreeFont(&Roboto_Black_28);

      tft.drawString("F I N D I N G S", (tft.width() - tft.textWidth("F I N D I N G S")) / 2, 16);
      if (finalButtonState == 0) {
        rcwlPercent = 0;
        tft.setTextColor(TFT_BLACK);
        tft.setFreeFont(&Roboto_Medium_20);
        tft.drawString("Motion: ", 25, 52);
        tft.setTextColor(TFT_GREEN);
        tft.setFreeFont(&Roboto_Medium_20);
        tft.drawString("INACTIVE", 104, 52);
      } else if (finalButtonState == 1) {
        rcwlPercent = 1;
        tft.setTextColor(TFT_BLACK);
        tft.setFreeFont(&Roboto_Medium_20);
        tft.drawString("Motion: ", 25, 52);
        tft.setTextColor(TFT_RED);
        tft.setFreeFont(&Roboto_Medium_20);
        tft.drawString("ACTIVE", 104, 52);
      }

      if ((diffAvg) <= 2 && diffAvg >= 1.3) {
        mlxPercent = 1;
        tft.setTextColor(TFT_BLACK);
        tft.setFreeFont(&Roboto_Medium_20);
        tft.drawString("Temp. Diff.: ", 25, 80);
        tft.setTextColor(TFT_RED);
        tft.setFreeFont(&Roboto_Medium_20);
        tft.drawString(String(diffAvg) + " C", 147, 80);
      } else if (diffAvg > 2) {
        mlxPercent = (1 - ((diffAvg - 2) / 2));
        tft.setTextColor(TFT_BLACK);
        tft.setFreeFont(&Roboto_Medium_20);
        tft.drawString("Temp. Diff.: ", 25, 80);
        if (mlxPercent >= 0.5) {
          tft.setTextColor(TFT_ORANGE);
          tft.setFreeFont(&Roboto_Medium_20);
          tft.drawString(String(diffAvg) + " C", 147, 80);
        } else if ((mlxPercent < 0.5) && (mlxPercent > 0)) {
          tft.setTextColor(TFT_GREEN);
          tft.setFreeFont(&Roboto_Medium_20);
          tft.drawString(String(diffAvg) + " C", 147, 80);
        } else {
          mlxPercent = 0;
          tft.setTextColor(TFT_GREEN);
          tft.setFreeFont(&Roboto_Medium_20);
          tft.drawString(String(diffAvg) + " C", 147, 80);
        }
      } else if (diffAvg < 1.3 && diffAvg > 0) {
        mlxPercent = 1 - abs((diffAvg - 1.3) / 1.3);
        tft.setTextColor(TFT_BLACK);
        tft.setFreeFont(&Roboto_Medium_20);
        tft.drawString("Temp. Diff.: ", 25, 80);
        if (mlxPercent >= 0.5) {
          tft.setTextColor(TFT_ORANGE);
          tft.setFreeFont(&Roboto_Medium_20);
          tft.drawString(String(diffAvg) + " C", 147, 80);
        } else {
          tft.setTextColor(TFT_GREEN);
          tft.setFreeFont(&Roboto_Medium_20);
          tft.drawString(String(diffAvg) + " C", 147, 80);
        }
      } else {
        mlxPercent = 0;
        tft.setTextColor(TFT_BLACK);
        tft.setFreeFont(&Roboto_Medium_20);
        tft.drawString("Temp. Diff.: ", 25, 80);
        tft.setTextColor(TFT_GREEN);
        tft.setFreeFont(&Roboto_Medium_20);

        tft.drawString(String(diffAvg) + " C", 147, 80);
      }

      if (finalAvgdB > 22) {
        maxPercent = 1 - ((finalAvgdB - 22) / 22.0);
        maxPercent = max(0.0f, min(maxPercent, 1.0f));
        tft.setTextColor(TFT_BLACK);
        tft.setFreeFont(&Roboto_Medium_20);
        tft.drawString("Sound: ", 25, 108);
        if (maxPercent >= 0.5) {
          tft.setTextColor(TFT_ORANGE);
          tft.setFreeFont(&Roboto_Medium_20);
          tft.drawString(String(finalAvgdB) + " dB", 97, 108);
        } else if ((maxPercent < 0.5) && (maxPercent > 0)) {
          tft.setTextColor(TFT_GREEN);
          tft.setFreeFont(&Roboto_Medium_20);
          tft.drawString(String(finalAvgdB) + " dB", 97, 108);
        } else {
          maxPercent = 0;
          tft.setTextColor(TFT_GREEN);
          tft.setFreeFont(&Roboto_Medium_20);
          tft.drawString(String(finalAvgdB) + " dB", 97, 108);
        }
      } else if (finalAvgdB < 18) {
        maxPercent = 1 - ((18 - finalAvgdB) / 18.0);
        maxPercent = max(0.0f, min(maxPercent, 1.0f));
        tft.setTextColor(TFT_BLACK);
        tft.setFreeFont(&Roboto_Medium_20);
        tft.drawString("Sound: ", 25, 108);
        if (maxPercent >= 0.8) {
          tft.setTextColor(TFT_ORANGE);
          tft.setFreeFont(&Roboto_Medium_20);
          tft.drawString(String(finalAvgdB) + " dB", 97, 108);
        } else {
          tft.setTextColor(TFT_GREEN);
          tft.setFreeFont(&Roboto_Medium_20);
          tft.drawString(String(finalAvgdB) + " dB", 97, 108);
        }
      } else {
        maxPercent = 1;
        tft.setTextColor(TFT_BLACK);
        tft.setFreeFont(&Roboto_Medium_20);
        tft.drawString("Sound: ", 25, 108);
        tft.setTextColor(TFT_RED);
        tft.setFreeFont(&Roboto_Medium_20);
        tft.drawString(String(finalAvgdB) + " dB", 97, 108);
      }

      overallPercent = ((rcwlPercent * 33.04) + (mlxPercent * 34.36) + (maxPercent * 32.6));

      if (overallPercent <= 33.34) {
        tft.fillRect(48, 144, ((217 / 100 * overallPercent) + 18), 35, TFT_GREEN);
        tft.setTextColor(TFT_BLACK);
        tft.setFreeFont(&Roboto_Medium_16);
        tft.drawString(String(overallPercent) + " %", 125, 154);
        tft.setTextColor(TFT_GREEN);
        tft.setFreeFont(&Roboto_Black_24);
        tft.drawString("Low Termite Activity", (tft.width() - tft.textWidth("LOW Termite Activity")) / 2, 196);
      } else if ((overallPercent <= 66.67) && (overallPercent > 33.34)) {
        tft.fillRect(48, 144, ((217 / 100 * overallPercent) + 18), 35, TFT_ORANGE);
        tft.setTextColor(TFT_BLACK);
        tft.setFreeFont(&Roboto_Medium_16);
        tft.drawString(String(overallPercent) + " %", 125, 154);
        tft.setTextColor(TFT_ORANGE);
        tft.setFreeFont(&Roboto_Black_24);
        tft.drawString("Moderate Termite Activity", (tft.width() - tft.textWidth("Moderate Termite Activity")) / 2, 196);
      } else {
        tft.fillRect(48, 144, ((217 / 100 * overallPercent) + 18), 35, TFT_RED);
        tft.setTextColor(TFT_BLACK);
        tft.setFreeFont(&Roboto_Medium_16);
        tft.drawString(String(overallPercent) + " %", 125, 154);
        tft.setTextColor(TFT_RED);
        tft.setFreeFont(&Roboto_Black_24);
        tft.drawString("High Termite Activity", (tft.width() - tft.textWidth("High Termite Activity")) / 2, 196);
      }
    }

    Serial.print("  RCWL percent: ");
    Serial.print(rcwlPercent);
    Serial.print("  MLX percent: ");
    Serial.print(mlxPercent);
    Serial.print("  MAX percent: ");
    Serial.print(maxPercent);
    Serial.print("  Total anay pirsint: ");
    Serial.print(overallPercent);
  } else {
    Serial.print("   Error: Uknown mode");
  }
  Serial.print("        Final RCWL value: ");
  Serial.print(finalButtonState);
  Serial.print("   Final MLX value: ");
  Serial.print(diffAvg);
  Serial.print("   Final MAX value: ");
  Serial.println(finalAvgdB);
}

void getColor(int j) {
  if (j >= 0 && j < 30) {
    R = 0;
    G = 0;
    B = 20 + (120.0 / 30.0) * j;
  }

  if (j >= 30 && j < 60) {
    R = (120.0 / 30) * (j - 30.0);
    G = 0;
    B = 140 - (60.0 / 30.0) * (j - 30.0);
  }

  if (j >= 60 && j < 90) {
    R = 120 + (135.0 / 30.0) * (j - 60.0);
    G = 0;
    B = 80 - (70.0 / 30.0) * (j - 60.0);
  }

  if (j >= 90 && j < 120) {
    R = 255;
    G = 0 + (60.0 / 30.0) * (j - 90.0);
    B = 10 - (10.0 / 30.0) * (j - 90.0);
  }

  if (j >= 120 && j < 150) {
    R = 255;
    G = 60 + (175.0 / 30.0) * (j - 120.0);
    B = 0;
  }

  if (j >= 150 && j <= 180) {
    R = 255;
    G = 235 + (20.0 / 30.0) * (j - 150.0);
    B = 0 + 255.0 / 30.0 * (j - 150.0);
  }
}

boolean isConnected() {
  Wire.beginTransmission((uint8_t)MLX90640_address);

  if (Wire.endTransmission() != 0)
    return (false);

  return (true);
}

boolean isInside(int i, int j) {
  int left = 6;
  int right = 25;
  int top = 5;
  int bottom = 17;

  if (j >= left && j <= right && i >= top && i <= bottom) {
    return true;
  } else {
    return false;
  }
}

boolean debounceButton(boolean state, byte buttonPin) {
  boolean stateNow = digitalRead(buttonPin);
  if (state != stateNow) {
    delay(50);
    stateNow = digitalRead(buttonPin);
  }
  return stateNow;
}

void analogMeter() {
  tft.fillRect(40, 10, 239, 126, TFT_GREY);
  tft.fillRect(45, 13, 230, 119, TFT_WHITE);
  tft.setTextColor(TFT_BLACK);

  for (int i = -50; i < 51; i += 5) {
    int tl = 15;
    float sx = cos((i - 90) * 0.0174532925);
    float sy = sin((i - 90) * 0.0174532925);
    uint16_t x0 = sx * (100 + tl) + 120;
    uint16_t y0 = sy * (100 + tl) + 140;
    uint16_t x1 = sx * 100 + 120;
    uint16_t y1 = sy * 100 + 140;
    float sx2 = cos((i + 5 - 90) * 0.0174532925);
    float sy2 = sin((i + 5 - 90) * 0.0174532925);
    int x2 = sx2 * (100 + tl) + 120;
    int y2 = sy2 * (100 + tl) + 140;
    int x3 = sx2 * 100 + 120;
    int y3 = sy2 * 100 + 140;

    if (i >= -40 && i < -15) {
      tft.fillTriangle(x0 + 40, y0 + 10, x1 + 40, y1 + 10, x2 + 40, y2 + 10, TFT_ORANGE);
      tft.fillTriangle(x1 + 40, y1 + 10, x2 + 40, y2 + 10, x3 + 40, y3 + 10, TFT_ORANGE);
    }

    if (i % 25 != 0) tl = 8;

    x0 = sx * (100 + tl) + 120;
    y0 = sy * (100 + tl) + 140;
    x1 = sx * 100 + 120;
    y1 = sy * 100 + 140;

    tft.drawLine(x0 + 40, y0 + 10, x1 + 40, y1 + 10, TFT_BLACK);

    if (i % 25 == 0) {
      x0 = sx * (100 + tl + 10) + 120;
      y0 = sy * (100 + tl + 10) + 140;
      switch (i / 25) {
        case -2:
          tft.drawString("0", x0 + 35, y0 + 10 - 12, 2);
          break;
        case -1:
          tft.drawString("25", x0 + 40, y0 + 10 - 9, 2);
          break;
        case 0:
          tft.drawString("50", x0 + 40, y0 + 10 - 6, 2);
          break;
        case 1:
          tft.drawString("75", x0 + 40, y0 + 10 - 9, 2);
          break;
        case 2:
          tft.drawString("100", x0 + 28, y0 + 8 - 12, 2);
          break;
      }
    }

    sx = cos((i + 5 - 90) * 0.0174532925);
    sy = sin((i + 5 - 90) * 0.0174532925);
    x0 = sx * 100 + 120;
    y0 = sy * 100 + 140;

    if (i < 50) tft.drawLine(x0 + 40, y0 + 10, x1 + 40, y1 + 10, TFT_BLACK);
  }

  tft.drawString("dB", 5 + 230 - 40 + 40, 119 - 20 + 10, 2);
  tft.drawString("dB", 120 + 20, 70 + 10, 4);
  tft.drawRect(5 + 40, 3 + 10, 230, 119, TFT_BLACK);
  plotNeedle(0, 0);
}

void plotNeedle(int value, byte ms_delay) {
  tft.setTextColor(TFT_BLACK, TFT_WHITE);
  char buf[8];
  dtostrf(value, 4, 0, buf);
  tft.drawRightString(buf, 40 + 40, 119 - 20 + 10, 2);

  if (value < -10) value = -10;
  if (value > 110) value = 110;

  while (!(value == old_analog)) {
    if (old_analog < value) old_analog++;
    else old_analog--;

    if (ms_delay == 0) old_analog = value;

    float sdeg = map(old_analog, -10, 110, -150, -30);
    float sx = cos(sdeg * 0.0174532925);
    float sy = sin(sdeg * 0.0174532925);
    float tx = tan((sdeg + 90) * 0.0174532925);

    tft.drawLine(120 + 20 * ltx - 1 + 40, 140 - 20 + 10, osx - 1 + 40, osy + 10, TFT_WHITE);
    tft.drawLine(120 + 20 * ltx + 40, 140 - 20 + 10, osx + 40, osy + 10, TFT_WHITE);
    tft.drawLine(120 + 20 * ltx + 1 + 40, 140 - 20 + 10, osx + 1 + 40, osy + 10, TFT_WHITE);
    tft.setTextColor(TFT_BLACK);
    tft.drawString("dB", 120 + 20, 70 + 10, 4);

    ltx = tx;
    osx = sx * 98 + 120;
    osy = sy * 98 + 140;

    tft.drawLine(120 + 20 * ltx - 1 + 40, 140 - 20 + 10, osx - 1 + 40, osy + 10, TFT_RED);
    tft.drawLine(120 + 20 * ltx + 40, 140 - 20 + 10, osx + 40, osy + 10, TFT_MAGENTA);
    tft.drawLine(120 + 20 * ltx + 1 + 40, 140 - 20 + 10, osx + 1 + 40, osy + 10, TFT_RED);

    if (abs(old_analog - value) < 10) ms_delay += ms_delay / 5;
    delay(ms_delay);
  }
}