#include <SPI.h>
#include <RH_RF95.h>
#include <RHReliableDatagram.h>

// ---------- Pins ----------
#define RFM95_CS   4
#define RFM95_RST  2
#define RFM95_INT  3

// ---------- Radio ----------
#define RF95_FREQ 915.0

// ---------- Addresses ----------
#define MAIN_ADDR      1
#define SECONDARY_ADDR 2

RH_RF95 rf95(RFM95_CS, RFM95_INT);
RHReliableDatagram manager(rf95, SECONDARY_ADDR);

bool waitingForDone = false;
unsigned long triggerSentAt = 0;
const unsigned long DONE_TIMEOUT_MS = 30000;  // how long to wait for DONE
int samplesCollected = 0;
int SC = 0;
int startButton = 7;
int state = 0;
bool lastButtonState = HIGH;
bool stableButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

// Screen Code
const unsigned char *point;

const unsigned char OLED_init_cmd[25]=
{
/*0xae,0X00,0X10,0x40,0X81,0XCF,0xff,0xa1,0xa4,
0xA6,0xc8,0xa8,0x3F,0xd5,0x80,0xd3,0x00,0XDA,0X12,
0x8d,0x14,0xdb,0x40,0X20,0X02,0xd9,0xf1,0xAF*/
0xAE,// Turn off display
0xD5,// Set clock divide ratio/oscillator frequency
0x80,// [3:0], divide ratio; [7:4], oscillator frequency

0xA8,// Set multiplex ratio
0X3F,// Default 0x3F (1/64)
0xD3,// Set display offset
0X00,// Default is 0
0x40,// Set display start line [5:0], line number.
0x8D,// Charge pump setting
0x14,// bit2, enable/disable
0x20,// Set memory addressing mode
0x02,// [1:0], 00: column addressing mode; 01: row addressing mode; 10: page addressing mode; Default is 10;
0xA1,// Set segment re-map, bit0: 0, 0->0; 1, 0->127;
0xC8,// Set COM output scan direction; bit3: 0, normal mode; 1, remapped mode COM[N-1]->COM0; N: multiplex ratio
0xDA,// Set COM pins hardware configuration
0x12,// [5:4] configuration
0x81,// Set contrast control
0xEF,// 1~255; Default 0x7F (Brightness setting, the larger the brighter)
0xD9,// Set pre-charge period
0xf1,// [3:0], PHASE1; [7:4], PHASE2;
0xDB,// Set VCOMH deselect level
0x30,// [6:4] 000: 0.65*VCC; 001: 0.77*VCC; 011: 0.83*VCC;
0xA4,// Entire display on; bit0: 1, on; 0, off; (White screen / Black screen)
0xA6,// Set display mode; bit0: 1, inverse display; 0, normal display
0xAF,// Turn on display
};

// Standard 5x8 pixel ASCII Font (Characters ' ' through '~')
const unsigned char font5x8[96][5] = {
  {0x00,0x00,0x00,0x00,0x00}, {0x00,0x00,0x5F,0x00,0x00}, {0x00,0x07,0x00,0x07,0x00}, {0x14,0x7F,0x14,0x7F,0x14},
  {0x24,0x2A,0x7F,0x2A,0x12}, {0x23,0x13,0x08,0x64,0x62}, {0x36,0x49,0x55,0x22,0x50}, {0x00,0x05,0x03,0x00,0x00},
  {0x00,0x1C,0x22,0x41,0x00}, {0x00,0x41,0x22,0x1C,0x00}, {0x08,0x2A,0x1C,0x2A,0x08}, {0x08,0x08,0x3E,0x08,0x08},
  {0x00,0x50,0x30,0x00,0x00}, {0x08,0x08,0x08,0x08,0x08}, {0x00,0x60,0x60,0x00,0x00}, {0x20,0x10,0x08,0x04,0x02},
  {0x3E,0x51,0x49,0x45,0x3E}, {0x00,0x42,0x7F,0x40,0x00}, {0x42,0x61,0x51,0x49,0x46}, {0x21,0x41,0x45,0x4B,0x31},
  {0x18,0x14,0x12,0x7F,0x10}, {0x27,0x45,0x45,0x45,0x39}, {0x3C,0x4A,0x49,0x49,0x30}, {0x01,0x71,0x09,0x05,0x03},
  {0x36,0x49,0x49,0x49,0x36}, {0x06,0x49,0x49,0x29,0x1E}, {0x00,0x36,0x36,0x00,0x00}, {0x00,0x56,0x36,0x00,0x00},
  {0x00,0x08,0x14,0x22,0x41}, {0x14,0x14,0x14,0x14,0x14}, {0x41,0x22,0x14,0x08,0x00}, {0x02,0x01,0x51,0x09,0x06},
  {0x32,0x49,0x79,0x41,0x3E}, {0x7E,0x11,0x11,0x11,0x7E}, {0x7F,0x49,0x49,0x49,0x36}, {0x3E,0x41,0x41,0x41,0x22},
  {0x7F,0x41,0x41,0x22,0x1C}, {0x7F,0x49,0x49,0x49,0x41}, {0x7F,0x09,0x09,0x01,0x01}, {0x3E,0x41,0x41,0x51,0x32},
  {0x7F,0x08,0x08,0x08,0x7F}, {0x00,0x41,0x7F,0x41,0x00}, {0x20,0x40,0x41,0x3F,0x01}, {0x7F,0x08,0x14,0x22,0x41},
  {0x7F,0x40,0x40,0x40,0x40}, {0x7F,0x02,0x04,0x02,0x7F}, {0x7F,0x04,0x08,0x10,0x7F}, {0x3E,0x41,0x41,0x41,0x3E},
  {0x7F,0x09,0x09,0x09,0x06}, {0x3E,0x41,0x51,0x21,0x5E}, {0x7F,0x09,0x19,0x29,0x46}, {0x46,0x49,0x49,0x49,0x31},
  {0x01,0x01,0x7F,0x01,0x01}, {0x3F,0x40,0x40,0x40,0x3F}, {0x1F,0x20,0x40,0x20,0x1F}, {0x7F,0x20,0x18,0x20,0x7F},
  {0x63,0x14,0x08,0x14,0x63}, {0x03,0x04,0x78,0x04,0x03}, {0x61,0x51,0x49,0x45,0x43}, {0x00,0x7F,0x41,0x41,0x00},
  {0x02,0x04,0x08,0x10,0x20}, {0x00,0x41,0x41,0x7F,0x00}, {0x04,0x02,0x01,0x02,0x04}, {0x40,0x40,0x40,0x40,0x40},
  {0x00,0x01,0x02,0x04,0x00}, {0x20,0x54,0x54,0x54,0x78}, {0x7F,0x48,0x44,0x44,0x38}, {0x38,0x44,0x44,0x44,0x20},
  {0x38,0x44,0x44,0x48,0x7F}, {0x38,0x54,0x54,0x54,0x18}, {0x08,0x7E,0x09,0x01,0x02}, {0x08,0x14,0x54,0x54,0x3C},
  {0x7F,0x08,0x04,0x04,0x38}, {0x00,0x44,0x7D,0x40,0x00}, {0x20,0x40,0x44,0x3D,0x00}, {0x7F,0x10,0x28,0x44,0x00},
  {0x00,0x41,0x7F,0x40,0x00}, {0x7C,0x04,0x18,0x04,0x78}, {0x7C,0x08,0x04,0x04,0x38}, {0x38,0x44,0x44,0x44,0x38},
  {0x7C,0x14,0x14,0x14,0x08}, {0x08,0x14,0x14,0x18,0x7C}, {0x7C,0x08,0x04,0x04,0x08}, {0x48,0x54,0x54,0x54,0x20},
  {0x04,0x3F,0x44,0x40,0x20}, {0x3C,0x40,0x40,0x20,0x7C}, {0x1C,0x20,0x40,0x20,0x1C}, {0x3C,0x40,0x30,0x40,0x3C},
  {0x44,0x28,0x10,0x28,0x44}, {0x0C,0x50,0x50,0x50,0x3C}, {0x44,0x64,0x54,0x4C,0x44}, {0x00,0x08,0x36,0x41,0x00},
  {0x00,0x00,0x7F,0x00,0x00}, {0x00,0x41,0x36,0x08,0x00}, {0x08,0x08,0x2A,0x1C,0x08}, {0x08,0x08,0x08,0x08,0x08}
};

/**************************IIC Module Sending Function************************************************

 *************************************************************************/
// Write. Finally, pull SDA high to wait for the slave device to generate an ACK.
void IIC_write(unsigned char date)
{
  unsigned char i, temp;
  temp = date;
      
  for(i=0; i<8; i++)
  { digitalWrite(5,LOW);
    
                if ((temp&0x80)==0)
                  digitalWrite(6,LOW);
                else digitalWrite(6,HIGH);
    temp = temp << 1;
              // Minimum 250ns delay
    digitalWrite(5,HIGH);
    
  }
  digitalWrite(5,LOW);
  
  digitalWrite(6,HIGH);
  
  digitalWrite(5,HIGH);
  // Do not perform ACK detection
  digitalWrite(5,LOW);
}

void IIC_start()
{
  digitalWrite(6,HIGH);
  
  digitalWrite(5,HIGH);
  digitalWrite(6,LOW);
  
  digitalWrite(5,LOW);
  
  IIC_write(0x78);
}

void IIC_stop()
{
  digitalWrite(6,LOW);
  
  digitalWrite(5,HIGH);
  
  digitalWrite(6,HIGH);
}

void OLED_send_cmd(unsigned char o_command)
{
  IIC_start();
  IIC_write(0x00); 
  IIC_write(o_command);
  IIC_stop();
}

void OLED_send_data(unsigned char o_data)
{ 
  IIC_start();
  IIC_write(0x40);
  IIC_write(o_data);
  IIC_stop();
}

void Column_set(unsigned char column)
{
  OLED_send_cmd(0x10|(column>>4));
  OLED_send_cmd(0x00|(column&0x0f));
}

void Page_set(unsigned char page)
{
  OLED_send_cmd(0xb0+page);
}

void OLED_clear(void)
{
  unsigned char page,column;
  for(page=0;page<8;page++)
  {
    Page_set(page);
    Column_set(0);
    for(column=0;column<128;column++)
    {
      OLED_send_data(0x00);
    }
  }
}

void OLED_full(void)
{
  unsigned char page,column;
  for(page=0;page<8;page++)
  {
    Page_set(page);
    Column_set(0);
    for(column=0;column<128;column++)
    {
      OLED_send_data(0xff);
    }
  }
}

void OLED_init(void)
{
  for (unsigned char i = 0; i < sizeof(OLED_init_cmd); i++)
  {
    OLED_send_cmd(OLED_init_cmd[i]);
  }
}

void Picture_display(const unsigned char *ptr_pic)
{
  unsigned char page,column;
  for(page=0;page<(64/8);page++)
  {
    Page_set(page);
    Column_set(0);
    for(column=0;column<128;column++)
    {
      OLED_send_data(*ptr_pic++);
    }
  }
}

void Picture_ReverseDisplay(const unsigned char *ptr_pic)
{
  unsigned char page,column,data;
  for(page=0;page<(64/8);page++)
  {
    Page_set(page);
    Column_set(0);
    for(column=0;column<128;column++)
    {
      data=*ptr_pic++;
      data=~data;
      OLED_send_data(data);
    }
  }
}

void IO_init(void)
{
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
}

void OLED_ShowChar(unsigned char page, unsigned char column, char ch) {
  unsigned char c = ch - ' ';
  
  Page_set(page);
  Column_set(column);
  
  for(unsigned char i = 0; i < 5; i++) {
    OLED_send_data(font5x8[c][i]);
  }
  OLED_send_data(0x00);
}

void OLED_ShowString(unsigned char page, unsigned char column, const char *str) {
  while(*str != '\0') {
    OLED_ShowChar(page, column, *str);
    column += 6;
    
    if(column > 122) { 
      column = 0;
      page += 1;
    }
    str++;
  }
}

void OLED_ShowNum(unsigned char page, unsigned char column, int num) {
  char buffer[10];
  itoa(num, buffer, 10);
  OLED_ShowString(page, column, buffer);
}

void hardResetRadio() {
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
}

void setup() {
  Serial.begin(9600);
  pinMode(startButton, INPUT_PULLUP);

  lastButtonState = digitalRead(startButton);
  stableButtonState = lastButtonState;

  delay(1200);
  Serial.println(F("SECONDARY boot"));

  hardResetRadio();

  if (!manager.init()) {
    Serial.println(F("LoRa init failed"));
    while (1);
  }
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println(F("setFrequency failed"));
    while (1);
  }
  rf95.setTxPower(23, false);

  IO_init();
  OLED_init();
  OLED_full();
  delay(500);
  OLED_clear();
  
  Serial.println(F("SECONDARY ready. Push Button for Triggering."));
  OLED_ShowString(5, 8, "Push Button to Start");
}

void sendTrigger() {
  uint8_t trig[1] = {'T'};

  Serial.println(F("Sending trigger..."));
  OLED_ShowString(5, 9, "    Triggering    ");

  bool ok = manager.sendtoWait(trig, sizeof(trig), MAIN_ADDR);

  if (ok) {
    Serial.println(F("Trigger delivered, waiting for DONE..."));
    OLED_ShowString(5, 9, "      Triggered         ");
    waitingForDone = true;
    triggerSentAt = millis();
  } else {
    Serial.println(F("Trigger FAILED (no ACK from main)"));
    waitingForDone = false;
  }

  Serial.print(F("After sendTrigger, waitingForDone = "));
  Serial.println(waitingForDone);
}

void handleIncoming(int &samplesCollected) {
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  uint8_t from;

  if (manager.recvfromAckTimeout(buf, &len, 50, &from)) {
   
    if (len < 1) return;

    char type = (char)buf[0];

    if (type == 'D') {
      if (len >= 1 + sizeof(float)) {
        float depth;
        memcpy(&depth, &buf[1], sizeof(float));

        Serial.print(F("Depth: "));
        Serial.print(depth, 3);
        Serial.print(F(" m (RSSI "));
        Serial.print(rf95.lastRssi());
        Serial.println(F(" dBm)"));

        if (depth < 50)
        {
          Serial.println(F("Go Up"));
          OLED_ShowString(3, 40, "Go  Up ");
        }
        else if (depth > 180)
        {
          Serial.println(F("Go Down"));
          OLED_ShowString(3,40, "Go Down");
        }
        else
        {
          Serial.println(F("Ready to Collect"));
          OLED_ShowString(3, 40, " Ready ");
        }
      }
    }

    else if (type == 'K') {
      uint32_t counter = 0;
      uint16_t receivedSamples = 0;

      if (len >= 1 + sizeof(uint32_t) + sizeof(uint16_t)) {
        memcpy(&counter, &buf[1], sizeof(uint32_t));
        memcpy(&receivedSamples, &buf[1 + sizeof(uint32_t)], sizeof(uint16_t));

        samplesCollected = (int)receivedSamples;

        Serial.print(F("New Sample Count: "));
        Serial.println(samplesCollected);

        waitingForDone = false;
      } else {
        Serial.println(F("K packet received but length was wrong"));
      }
    }

    else if (type == 'E') { 
      if (waitingForDone) {
        Serial.println(F("************************************"));
        Serial.println(F("TRIGGER REJECTED: Depth is out of bounds!"));
        Serial.println(F("************************************")); 
        OLED_ShowString(5, 0, "     Triggered Failed       ");

        waitingForDone = false;
      } 
    }  
  }
}

void loop() {
  bool reading = digitalRead(startButton);

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 200) {
    Serial.print(F("Button reading: "));
    Serial.print(reading);
    Serial.print(F(" | stable: "));
    Serial.print(stableButtonState);
    Serial.print(F(" | waitingForDone: "));
    Serial.println(waitingForDone);
    lastPrint = millis();
  }

  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != stableButtonState) {
      stableButtonState = reading;

      if (stableButtonState == LOW && !waitingForDone) {
        Serial.println(F("Button press detected -> sending trigger"));
        sendTrigger();
      }
    }
  }

  lastButtonState = reading;

  handleIncoming(samplesCollected);

  static int lastDisplayed = -1;
  if (samplesCollected != lastDisplayed) {
    OLED_ShowString(0, 110, "                        ");
    OLED_ShowString(0, 0, "Samples Collected:");
    OLED_ShowNum(0, 110, samplesCollected);
    lastDisplayed = samplesCollected;
    OLED_ShowString(5, 9, "Ready for reTriggering");
  }

  if (waitingForDone && (millis() - triggerSentAt > DONE_TIMEOUT_MS)) {
    Serial.println(F("Timed out waiting for DONE."));
    waitingForDone = false;
  }
}
