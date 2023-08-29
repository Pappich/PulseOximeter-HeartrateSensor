#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define SCREEN_ADDRESS 0x3c ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

#define OLED_RESET 4
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#define maxperiod_siz 80 // max number of samples in a period 80
#define measures 8 // number of periods stored 10
#define samp_siz 4 // number of samples for average 4
#define rise_threshold 3 // number of rising measures to determine a peak 3
// a liquid crystal displays BPM
//LiquidCrystal_I2C lcd(0x3F, 16, 2);
int T = 20; // slot milliseconds to read a value from the sensor
int sensorPin = A1;
int REDLed = 6;
int IRLed = 7;
int SpO2;
int avBPM;

//indexfinger
const unsigned char myBitmap [] PROGMEM = {
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0x8f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xe0, 0x0f, 0xff, 0xff, 0xc0, 0x7f, 0xe0, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0x8f, 0xfc, 0x7c, 0x00, 0xff, 0xff, 0x8f, 0xff, 0xff, 0xc7, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xfe, 0x3f, 0xff, 0xfc, 0x01, 0xff, 0xfc, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xfc, 0x78, 0x00, 0x31, 0xf8, 0x80, 0x1c, 0x71, 0xfc, 0x60, 0xe2, 0x00, 0x71, 0xc1, 0xff,
  0xff, 0xfc, 0x7f, 0xf8, 0xf1, 0xf8, 0xff, 0x8c, 0x71, 0xfc, 0x70, 0x87, 0xfe, 0x18, 0xf8, 0xff,
  0xff, 0xfc, 0x78, 0x00, 0xf1, 0xf8, 0xff, 0x84, 0x71, 0xfc, 0x63, 0xe2, 0x0f, 0x0c, 0x78, 0xff,
  0xff, 0xfc, 0x71, 0xf8, 0xf1, 0xf0, 0xff, 0x8c, 0x71, 0xf8, 0x63, 0xe2, 0x3f, 0x1c, 0x38, 0xff,
  0xff, 0xfc, 0x70, 0x78, 0xf0, 0x00, 0xf0, 0x1c, 0x78, 0x00, 0x60, 0x02, 0x00, 0x3e, 0x01, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x00, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0x00, 0x0f, 0xff, 0xff, 0xfc, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xf0, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xc1, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xfc, 0x00, 0x3f, 0x03, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xc0, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xff,
  0xc3, 0xff, 0xc0, 0xff, 0xff, 0xff, 0xff, 0xf0, 0xff, 0xff, 0x0f, 0xf8, 0x00, 0x00, 0x00, 0x0f,
  0x87, 0xff, 0xc3, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x03, 0xff, 0x0f, 0xff, 0xff, 0xff, 0xff, 0x83,
  0x87, 0xff, 0xc3, 0xff, 0xff, 0xff, 0xff, 0xc3, 0xc3, 0xff, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xe1,
  0x87, 0xff, 0xc3, 0xff, 0xff, 0xff, 0xfe, 0x00, 0x01, 0xff, 0x0f, 0xff, 0xff, 0xff, 0xfe, 0x03,
  0x87, 0xff, 0xc3, 0xff, 0xff, 0xff, 0xf0, 0x3f, 0x80, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0x1f,
  0x87, 0xff, 0xc3, 0xf8, 0x00, 0x00, 0x00, 0x7f, 0xf8, 0x3f, 0x87, 0xf0, 0xff, 0xff, 0xff, 0xff,
  0x87, 0xff, 0xc3, 0xf8, 0x00, 0x01, 0xfc, 0x1f, 0xfe, 0x0f, 0x87, 0xf8, 0x7f, 0xff, 0xff, 0xff,
  0x87, 0xff, 0xc3, 0xff, 0xff, 0xff, 0xf0, 0x00, 0xff, 0x80, 0x0f, 0xf0, 0xff, 0xff, 0xff, 0xff,
  0x87, 0xff, 0xc3, 0xff, 0xff, 0xff, 0xe1, 0xf0, 0x03, 0xff, 0xfc, 0x01, 0xff, 0xff, 0xff, 0xff,
  0x87, 0xff, 0xc3, 0xff, 0xff, 0xff, 0xe1, 0xff, 0xc0, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xff, 0xff,
  0x87, 0xff, 0xc3, 0xff, 0xff, 0xff, 0xf0, 0x3f, 0xff, 0xff, 0xfc, 0x3f, 0xff, 0xff, 0xff, 0xff,
  0x87, 0xff, 0xc3, 0xff, 0xff, 0xff, 0x80, 0x0f, 0xff, 0xff, 0xfc, 0x3f, 0xff, 0xff, 0xff, 0xff,
  0xc7, 0xff, 0xc1, 0xff, 0xff, 0xff, 0x87, 0xc0, 0xff, 0xff, 0xc0, 0x7f, 0xff, 0xff, 0xff, 0xff,
  0xc3, 0xff, 0xc0, 0x1f, 0xff, 0xff, 0x83, 0xf8, 0x00, 0x00, 0x03, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xe0, 0x38, 0x07, 0x00, 0x3f, 0xff, 0xe0, 0xff, 0xff, 0xf8, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xf8, 0x00, 0x3f, 0xfc, 0x00, 0x1f, 0xf8, 0x1f, 0xff, 0xf0, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xfe, 0x00, 0x07, 0x00, 0x00, 0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};

byte sym[3][8] = {{B00000, B01010, B11111, B11111, B01110, B00100, B00000, B00000},
  {B00000, B00000, B00000, B11000, B00100, B01000, B10000, B11100},
  {B00000, B00100, B01010, B00010, B00100, B00100, B00000, B00100}
};

void setup() {
  Serial.begin(115200);
  Serial.flush();
  pinMode(sensorPin, INPUT);
  pinMode(REDLed, OUTPUT);
  pinMode(IRLed, OUTPUT);

  // turn off leds
  digitalWrite(REDLed, LOW);
  digitalWrite(IRLed, LOW);
  // for(int i=0;i<8;i++) lcd.createChar(i, sym[i]);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Don't proceed, loop forever
  }
}
void loop ()
{
  bool finger_status = true;
  float readsIR[samp_siz], sumIR, lastIR, reader, start;
  float readsRED[samp_siz], sumRED, lastRED;
  int period, samples;
  period = 0; samples = 0;
  int samplesCounter = 0;
  float readsIRMM[maxperiod_siz], readsREDMM[maxperiod_siz]; //ประกาศตัวแปรไว้เก็บค่าที่วัดได้จาก IR กับ LED red
  int ptrMM = 0;
  for (int i = 0; i < maxperiod_siz; i++) {
    readsIRMM[i] = 0;
    readsREDMM[i] = 0;
  }
  float IRmax = 0;
  float IRmin = 0;
  float REDmax = 0;
  float REDmin = 0;
  double R = 0;
  float measuresR[measures];
  int measuresPeriods[measures];
  int m = 0;
  for (int i = 0; i < measures; i++) {
    measuresPeriods[i] = 0;
    measuresR[i] = 0;
  }
  int ptr;
  float beforeIR;
  bool rising;
  int rise_count;
  int n;
  long int last_beat;
  for (int i = 0; i < samp_siz; i++) {
    readsIR[i] = 0;
    readsRED[i] = 0;
  }
  sumIR = 0; sumRED = 0;
  ptr = 0;
  while (1)
  {
    // turn on IR LED
    digitalWrite(REDLed, LOW);
    digitalWrite(IRLed, HIGH);
    // calculate an average of the sensor
    n = 0;
    start = millis();
    reader = 0.;
    do
    {
      reader += analogRead (sensorPin);
      n++;
    }
    while (millis() < start + T);
    reader /= n; // we got an average
    // Add the newest measurement to an array
    // and subtract the oldest measurement from the array
    // to maintain a sum of last measurements
    sumIR -= readsIR[ptr];
    sumIR += reader;
    readsIR[ptr] = reader;
    lastIR = sumIR / samp_siz;

    // TURN ON RED LED //
    digitalWrite(REDLed, HIGH);
    digitalWrite(IRLed, LOW);
    n = 0;
    start = millis();
    reader = 0.;
    do
    {
      reader += analogRead (sensorPin);
      n++;
    }
    while (millis() < start + T);
    reader /= n; // we got an average
    // Add the newest measurement to an array
    // and subtract the oldest measurement from the array
    // to maintain a sum of last measurements
    sumRED -= readsRED[ptr];
    sumRED += reader;
    readsRED[ptr] = reader;
    lastRED = sumRED / samp_siz;

    // R CALCULATION
    // save all the samples of a period both for IR and for RED//
    readsIRMM[ptrMM] = lastIR;
    readsREDMM[ptrMM] = lastRED;
    ptrMM++;
    ptrMM %= maxperiod_siz; //modค่าmaxperiod เพื่อ store ค่าptrMMใหม่ไว้วนลูป
    samplesCounter++;


    // max and min values and calculate R parameter
    if (samplesCounter >= samples) {
      samplesCounter = 0;
      IRmax = 0; IRmin = 1023; REDmax = 0; REDmin = 1023;
      for (int i = 0; i < maxperiod_siz; i++) {
        if ( readsIRMM[i] > IRmax) IRmax = readsIRMM[i];
        if ( readsIRMM[i] > 0 && readsIRMM[i] < IRmin ) IRmin = readsIRMM[i];
        readsIRMM[i] = 0;
        if ( readsREDMM[i] > REDmax) REDmax = readsREDMM[i];
        if ( readsREDMM[i] > 0 && readsREDMM[i] < REDmin ) REDmin = readsREDMM[i]; //หาค่าต่ำสุดที่รับได้ของ LED สีแดง
        readsREDMM[i] = 0;
      }
      R = ( (REDmax - REDmin) / REDmin) / ( (IRmax - IRmin) / IRmin ) ; คำนวนหาค่า R ออกมาเพื่อนำค่า R ที่ได้ไปคำนวนต่อ
    }
    // check that the finger is placed inside the sensor. If the finger is missing
    // RED curve is under the IR.

    if (lastRED < lastIR)
    {
      if (finger_status == true)
      {
        finger_status = false;
      }
    }
    else {
      if (finger_status == false) {
        finger_status = true;
      }
    }

    float avR = 0;
    avBPM = 0;
    if (finger_status == true) {
      // lastIR holds the average of the values in the array
      // check for a rising curve (= a heart beat)
      if (lastIR > beforeIR)
      {
        rise_count++; // count the number of samples that are rising
        if (!rising && rise_count > rise_threshold)
        {
          rising = true;
          measuresR[m] = R;
          measuresPeriods[m] = millis() - last_beat;
          last_beat = millis();
          int period = 0;
          for (int i = 0; i < measures; i++) period += measuresPeriods[i]; //เก็บค่าperiod จาก ค่าที่วัดได้เพื่อเอาไปคำนวนต่อ
          // calculate average period and number of samples to store to find min and max values
          period = period / measures;
          samples = period / (2 * T);
          int avPeriod = 0;
          int c = 0;
          // c stores the number of good measures (not floating more than 10%)
          for (int i = 1; i < measures; i++) {
            if ( (measuresPeriods[i] < measuresPeriods[i - 1] * 1.1) &&
                 (measuresPeriods[i] > measuresPeriods[i - 1] / 1.1) ) {
              c++;
              avPeriod += measuresPeriods[i];
              avR += measuresR[i]; //เอาไว้หาค่าที่วัดได้และตัดค่าที่แกว่งเกิน10% ออกจะได้เหลือแต่ค่าที่คงที่ไว้คำนวนต่อได้
            }
          }
          m++;
          m %= measures;
          avBPM = 60000 / ( avPeriod / c) ;
          avR = avR / c ;
          if (c == 0)  Serial.println(" ");
          else Serial.println(" ");
          // if there are at least 5 good measures...
          if (c > 4) {
            SpO2 = -19 * R + 99 + 26; //สูตรคำนวนหาค่าอิ่มตัวของOxigen
            if (avBPM > 40 && avBPM < 220) Serial.println(String(avBPM) + " ");
            dis();
            if (SpO2 > 70 && SpO2 < 110) Serial.println( " " + String(SpO2) + "% ");
            //แสดงผลค่าขึ้นจอ Arduino
            dis();
          }
          else {
            if (c < 3) {
              display.clearDisplay();
              display.setTextSize(1); // Draw 2X-scale text
              display.setTextColor(SSD1306_WHITE);
              display.setCursor(20, 10);
              display.println("Insert Finger ");
              display.drawBitmap(0, 0, myBitmap, 128, 64, WHITE);
              display.display();
            }
          }
        }
      }
      else
      {
        rising = false;
        rise_count = 0;
      }
      beforeIR = lastIR;
    } // finger is inside
    // PLOT everything
    Serial.print(",");
    Serial.println();
    ptr++;
    ptr %= samp_siz;
  } // loop while 1
}

void dis()
{
  int ccc = 0;
  display.clearDisplay();
  display.setTextSize(1); // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10, 0);
  display.println("SpO2%");
  display.setCursor(90, 0);
  display.println("BpM");
  display.setTextSize(2);
  display.setCursor(10, 11);
  display.print(SpO2);
  display.println("%");
  display.setCursor(80, 11);
  display.println(avBPM);
  display.setTextSize(1);
  for (ccc = 0; ccc < 1000; ccc++) {
    display.setCursor(50, 41);
    display.println("ccc");
  }
  display.display();
}
