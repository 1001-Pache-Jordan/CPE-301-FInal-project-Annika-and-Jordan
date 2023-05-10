//Annika Valdez and Jordan Pache

int SENSOR = A1;
int RED = 9;
int GREEN = 10;
int VALUE;
int TEMP;
int SENSORvalue;
int SETvalue;

#include <LiquidCrystal.h>
#include <Stepper.h>
#include <IRremote.h>
#include <DHT.h>
#include <DHT_U.h>
#include <RTClib.h>

/*----- Variables, Pins -----*/
#define STEPS  25 
#define STEPPER_PIN1 30
#define STEPPER_PIN2 31
#define STEPPER_PIN3 32
#define STEPPER_PIN4 33

//fan
#define MOTOR_PIN 3

#define DHT_PIN 2
#define DHT_TYPE DHT11

#define WATER_LEVEL 5

//leds
#define LED_PINR 36
#define LED_PINY 37
#define LED_PING 38
#define LED_PINB 39

#define RDA 0x80
#define TBE 0x20  

LiquidCrystal lcd(30, 31, 32, 33, 34, 35);

RTC_DS1307 RTC;

DHT dht(DHT_PIN, DHT_TYPE);

Stepper stepper(2048, STEPPER_PIN1, STEPPER_PIN2, STEPPER_PIN3, STEPPER_PIN4);

volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;

enum States {
  IDLE,
  DISABLED,
  RUNNING,
  ERROR,
  START,
};

#define WATER_THRESHOLD 320 
#define TEMP_THRESHOLD 10 

States currentState = DISABLED;
States prevState = START;

int  Steps2Take; 
int receiver = 6;

Stepper small_stepper(STEPS, 33, 29, 31, 27);
IRrecv irrecv(receiver); 
decode_results results; 

void setup()
{
  irrecv.enableIRIn(); 
  PORTD |= (1 << PD0) | (1 << PD1);
  RTC.begin();
  DateTime now = DateTime(2022, 12, 9, 0, 0, 0);
  RTC.adjust(now);

  DDRE |= (0x01 << LED_PING | 0x01 << LED_PINY);
  DDRG |= (0x01 << LED_PINB);
  DDRH |= (0x01 << LED_PINR | 0x01 << WATER_LEVEL | 0x01 << MOTOR_PIN);

  //adc_init();
  //U0init(9600);
  dht.begin();
  lcd.begin(16, 2);
  lcd.print("System Starting");
}

int lastTempPrint = 0;
float temp = 0;
float hum = 0;
bool fanOn = false;
int ledC = -1;
bool displayTempHum = false;
bool monitorWater = false;

void loop()
{
  int button = A0;
  while(digitalRead(button)==HIGH){

    if (irrecv.decode(&results))
    {
      switch(results.value)
      {
        case 0xFFA857:
                      small_stepper.setSpeed(500);
                      Steps2Take=2048;
                      small_stepper.step(Steps2Take);
                      delay(2000); 
                      break;
        case 0xFF629D:
                      small_stepper.setSpeed(500);
                      Steps2Take=-2048; 
                      small_stepper.step(Steps2Take);
                      delay(2000); 
                      break;    
      }
    
        irrecv.resume(); 
                 digitalWrite(33, LOW);
                 digitalWrite(31, LOW);
                 digitalWrite(29, LOW);
                 digitalWrite(27, LOW);       
    } 

    DateTime now = RTC.now();
    if(displayTempHum){
      temp = dht.readTemperature();
      hum = dht.readHumidity();
    }

    currentState = decideState(temp, hum, currentState);
    if(currentState != prevState){
      writeTimeStampTransition(now, prevState, currentState);
      switch (currentState) {
        case DISABLED:
          fanOn = false;
          ledC = 3;
          displayTempHum = false;
          monitorWater = false;
          break;
        case IDLE:
          fanOn = false;
          ledC = 2;
          displayTempHum = true;
          monitorWater = true;
          break;
        case RUNNING:
          fanOn = true;
          ledC = 1;
          displayTempHum = true;
          monitorWater = true;
          break;
        case ERROR:
          lcd.clear();
          lcd.print("Error,low water");
          fanOn = false;
          ledC = 0;
          displayTempHum = true;
          monitorWater = true;
          break;
        case START:
          break;
        }
    }

    setFanMotor(fanOn);
    turnLEDOn(ledC);
    if(displayTempHum && abs(lastTempPrint - now.minute()) >= 1){ 
      lcd.clear();
      lastTempPrint = now.minute(); 
      temp = dht.readTemperature();
      hum = dht.readHumidity();
      lcd.print("Temp, Humidity");
      delay(1000);
      lcd.clear();
      lcd.print(temp); 
      lcd.print(hum);
    }

    prevState = currentState;
    if(monitorWater){
      int waterLvl = adc_read(WATER_LEVEL);
      if(waterLvl <= WATER_THRESHOLD){
        currentState = ERROR;
      }
    }
    delay(500);
    
  }
}
int waterRead(int pin){
  return PINH & (0x01 << pin);
}

int pinRead(int pin) {
  return PINA & (0x01 << pin);
}

void writeStepperPos(DateTime now, States prevState, States currentState){
  U0putchar('S');
  U0putchar('T');
  U0putchar('E');
  U0putchar('P');
  
  U0putchar(' ');
  writeTimeStampTransition(now, prevState, currentState);
}

void writeTimeStampTransition(DateTime now, States prevState, States currentState){
  unsigned char pState = (prevState == DISABLED ? 'd' : (prevState == IDLE ? 'i' : (prevState == RUNNING ? 'r' : (prevState == ERROR ? 'e' : 'u'))));
  unsigned char cState = (currentState == DISABLED ? 'd' : (currentState == IDLE ? 'i' : (currentState == RUNNING ? 'r' : (currentState == ERROR ? 'e' : 'u')))); 
  
  U0putchar(pState);
  U0putchar(':');
  U0putchar(cState);

  U0putchar(' ');

  int year = now.year();
  int month = now.month();
  int day = now.day();
  int hour = now.hour();
  int minute = now.minute();
  int second = now.second();
  char numbers[10] = {'0','1','2','3','4','5','6','7','8','9'};
  int onesYear = year % 10;
  int tensYear = year / 10 % 10;
  int onesMonth = month % 10;
  int tensMonth = month / 10 % 10;
  int onesDay = day % 10;
  int tensDay = day / 10 % 10;
  int onesHour = hour % 10;
  int tensHour = hour / 10 % 10;
  int onesMinute = minute % 10;
  int tensMinute = minute / 10 % 10;
  int onesSecond = second % 10;
  int tensSecond = second / 10 % 10;
  
  U0putchar('M');
  U0putchar(':');
  U0putchar('D');
  U0putchar(':');
  U0putchar('Y');
  U0putchar(' ');
  U0putchar('H');
  U0putchar(':');
  U0putchar('M');
  U0putchar(':');
  U0putchar('S');
  U0putchar(' ');
  U0putchar(numbers[tensMonth]);
  U0putchar(numbers[onesMonth]);
  U0putchar(':');
  U0putchar(numbers[tensDay]);
  U0putchar(numbers[onesDay]);
  U0putchar(':');
  U0putchar(numbers[tensYear]);
  U0putchar(numbers[onesYear]);
  U0putchar(' ');
  U0putchar(numbers[tensHour]);
  U0putchar(numbers[onesHour]);
  U0putchar(':');
  U0putchar(numbers[tensMinute]);
  U0putchar(numbers[onesMinute]);
  U0putchar(':');
  U0putchar(numbers[tensSecond]);
  U0putchar(numbers[onesSecond]);
  U0putchar('\n');
}

States decideState(float temp, int waterLvl, States currentState){
  States state;
  if(temp <= TEMP_THRESHOLD && currentState == RUNNING){
    state = IDLE;
  }else if(temp > TEMP_THRESHOLD && currentState == IDLE){
    state = RUNNING;
  }else{
    state = currentState;
  }
  return state;
}

void turnLEDOn(int ledPin) {
  PORTH &= ~(0x01 << LED_PINR);
  PORTG &= ~(0x01 << LED_PINB);
  PORTE &= ~(0x01 << LED_PING);
  PORTE &= ~(0x01 << LED_PINY);

  switch (ledPin) {
    case 0:
      PORTH |= 0x01 << LED_PINR;
      break;
    case 1:
      PORTG |= 0x01 << LED_PINB;
      break;
    case 2:
      PORTE |= 0x01 << LED_PING;
      break;
    case 3:
      PORTE |= 0x01 << LED_PINY;
      break;
  }
}

void setFanMotor(bool on){
  if(on){
    PORTH |= (0x01 << MOTOR_PIN);
  }else {
    PORTH &= ~(0x01 << MOTOR_PIN);
  }
}

void adc_init(){
     ADCSRA = 0x80;
     ADCSRB = 0x00;
     ADMUX = 0x40;
}

unsigned int adc_read(unsigned char adc_channel){
     ADCSRB &= 0xF7; 
     ADCSRB |= (adc_channel & 0x08);
     ADMUX &= 0xF8; // Reset MUX2:0.
     ADMUX |= (adc_channel & 0x07); 

     ADCSRA |= 0x40;
     while (ADCSRA & 0x40) {} 
     return ADC;
}

void U0init(unsigned long U0baud)
{
 unsigned long FCPU = 16000000;
 unsigned int tbaud;
 tbaud = (FCPU / 16 / U0baud - 1);
 *myUCSR0A = 0x20;
 *myUCSR0B = 0x18;
 *myUCSR0C = 0x06;
 *myUBRR0  = tbaud;
}
unsigned char U0kbhit()
{
  return (RDA & *myUCSR0A);
}
unsigned char U0getchar()
{
  return *myUDR0;
}
void U0putchar(unsigned char U0pdata)
{
  while(!(TBE & *myUCSR0A));
  *myUDR0 = U0pdata;
}
