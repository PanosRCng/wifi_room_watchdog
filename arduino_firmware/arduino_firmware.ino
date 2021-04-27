#include <SoftwareSerial.h>
#include "Dht11.h"
#include "NBDelay.h"

#define LED_PIN 11
#define POT_PIN A1

#define LIGHT_PIN A0
#define PIR_PIN 2
#define MSWITCH_PIN 3
#define DHT_DATA_PIN 4
#define RX_PIN 8
#define TX_PIN 9

#define SERIAL_BAUD 9600
#define WAIT_MINUTES 5
#define RESAMPLE_NUMBER 3
#define DELAY_BETWEEN_SAMPLES 1000
#define PIR_CALIBRATION_SECONDS 30
#define PIR_FALL_DELAY 1000
#define MSWTICH_READ_DELAY 200
#define SEND_DELAY 1000
#define LIGHT_DELAY_MINUTES 30

#define SAMPLE 0
#define SEND_MSG 1
#define WAIT 2

#define LIGHT_ON 3
#define LIGHT_OFF 4
#define LIGHT_MANUAL 5



SoftwareSerial mySerial(RX_PIN, TX_PIN);

static Dht11 sensor(DHT_DATA_PIN);

unsigned long sample_delay_millis;
unsigned long wait_delay_millis;

volatile bool pir_rise;
unsigned long pir_fall_delay_millis;

volatile bool mswitch_change;
unsigned long mswitch_read_delay_millis;

int state;
int light_state;
String msg;
int resample_number;
int t_sample;
int h_sample;
int l_sample;
int wait_minutes;

int previous_pot_read;
bool pot_changed;
int light_minutes;
unsigned long light_delay_millis;
unsigned long pot_check_delay_millis;


void sampleLight()
{
  l_sample = analogRead(LIGHT_PIN);
}


void sampleTempHumi()
{
  switch( sensor.read() )
  {
    case Dht11::OK:
      t_sample = sensor.getTemperature();
      h_sample = sensor.getHumidity();    
      break;
    default:
      t_sample = -1;
      h_sample = -1;
      l_sample = -1;
      break;  
  }
}


void sampling()
{
  sampleLight();

  sampleTempHumi();

  resample_number++;
}


void getInfoMsg()
{  
  msg = "{";
  msg += "\"type\": \"info\",";
  msg += "\"data\": ";
  
  msg += "{";
  msg += "\"Temperature\": ";
  msg += t_sample;
  msg += ", ";
  msg += "\"Humidity\": ";
  msg += h_sample;
  msg += ", ";
  msg += "\"Light\": ";
  msg += l_sample;
  msg += "}";
  
  msg += "}";
  msg += "#";
}


void sendEvent(String event)
{
  String event_msg = "{\"type\": \"event\", \"data\": {\"";
  event_msg += event;
  event_msg += "\": \"true\"}}#";
  
  mySerial.print(event_msg);
}


void calibratePir()
{  
  for(int i=0; i<PIR_CALIBRATION_SECONDS; i++)
  {
    delay(1000);
  }
}


void movementEvent()
{
  if(nb_delay(&pir_fall_delay_millis, PIR_FALL_DELAY) == false)
  {
    return;
  }
  
  sendEvent("Movement");
    
  pir_rise = false;
}


void lockEvent()
{
  if(nb_delay(&mswitch_read_delay_millis, MSWTICH_READ_DELAY) == false)
  {
    return;
  }
  
  if(digitalRead(MSWITCH_PIN) == HIGH)
  {
    sendEvent("Lock opened");
  }
  else
  {
    sendEvent("Lock closed");
  }

  mswitch_change = false;
}


void wait()
{
  if(nb_delay(&wait_delay_millis, 60000UL) == false)
  {
    return;
  }

  if(++wait_minutes >= WAIT_MINUTES)
  {
    wait_minutes = 0;
    move2FirstState();
    return;
  }
}


void sendMsg()
{  
  mySerial.print(msg);

  wait_delay_millis = 0;
  wait_minutes = 0;
  
  state = WAIT;
}


void sample()
{
  if(resample_number >= RESAMPLE_NUMBER)
  {
    getInfoMsg();
    
    state = SEND_MSG;
    return;
  }

  if(nb_delay(&sample_delay_millis, DELAY_BETWEEN_SAMPLES) == false)
  {
    return;
  }

  sampling();
}


void light_off()
{
  if(pir_rise)
  {
    light_delay_millis = 0;
    light_minutes = 0;
    light_state = LIGHT_ON;
    return;
  }

  if(pot_changed)
  {
    light_delay_millis = 0;
    light_minutes = 0;
    light_state = LIGHT_MANUAL;
    return;
  }
  
  analogWrite(LED_PIN, 0);
}


void light_on()
{   
  if(pot_changed)
  {
    light_delay_millis = 0;
    light_minutes = 0;
    light_state = LIGHT_MANUAL;
    return;
  }

  if(pir_rise)
  {
    light_delay_millis = 0;
    light_minutes = 0;
  }
  
  if(nb_delay(&light_delay_millis, 60000UL) == true)
  {
    light_minutes++;
  }

  if(light_minutes >= LIGHT_DELAY_MINUTES)
  {
    light_state = LIGHT_OFF;
    return;
  }
  
  analogWrite(LED_PIN, 255);
}


void light_manual()
{
  if(pir_rise)
  {
    light_delay_millis = 0;
    light_minutes = 0;
  }
  
  if(nb_delay(&light_delay_millis, 60000UL) == true)
  {
    light_minutes++;
  }

  if(light_minutes >= LIGHT_DELAY_MINUTES)
  {
    light_state = LIGHT_OFF;
    return;
  }
  
  analogWrite(LED_PIN, map(analogRead(POT_PIN), 0, 1023, 0, 255));
}


bool pot_check()
{
  pot_changed = false;
  
  if(nb_delay(&pot_check_delay_millis, 100) == false)
  {
    return;
  }

  int pot_read = map(analogRead(POT_PIN), 0, 1023, 0, 255);
  
  if(abs(previous_pot_read - pot_read) > 10)
  {
    pot_changed = true;
  }

  previous_pot_read = pot_read;
  pot_check_delay_millis = 0;
}


void light()
{
  pot_check();
  
  switch(light_state)
  {
    case LIGHT_OFF:
      light_off();
      break;

    case LIGHT_ON:
      light_on();
      break;

    case LIGHT_MANUAL:
      light_manual();
      break;
  }
}


void move2FirstState()
{  
  t_sample = 0;
  h_sample = 0;
  l_sample = 0;
  
  resample_number = 0;
  
  msg = "";
  
  state = SAMPLE;
}


void setup()
{ 
  Serial.begin(9600);
  
  mySerial.begin(SERIAL_BAUD);

  light_state = LIGHT_ON;
  light_delay_millis = 0;
  pot_check_delay_millis = 0;
  previous_pot_read = map(analogRead(POT_PIN), 0, 1023, 0, 255);

  sample_delay_millis = 0;
  wait_delay_millis = 0;

  cli();

  attachInterrupt(digitalPinToInterrupt(MSWITCH_PIN), mswitch_isr, CHANGE);
  mswitch_change = false;
  mswitch_read_delay_millis = 0;

  attachInterrupt(digitalPinToInterrupt(PIR_PIN), pir_isr, RISING);
  pir_rise = false;
  pir_fall_delay_millis = 0;

  calibratePir();

  sei();

  move2FirstState();
}


void loop()
{ 
  if(mswitch_change == true)
  {
    lockEvent();
  }

  if(pir_rise == true)
  {
    movementEvent();
  }

  light();

  switch(state)
  {
    case SAMPLE:
      sample();
      break;

    case SEND_MSG:
      sendMsg();
      break;

    case WAIT:
      wait();
      break;
  }
}


void pir_isr()
{
  pir_rise = true;
}


void mswitch_isr()
{
  mswitch_change = true;
}
