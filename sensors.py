#include <dht.h>

dht DHT;

#define DHT11_PIN 5  //hum-temp sensor

int relay_pin = 7 ; //relay control pin
int DOPin = 8 ; //smoke input digital
int incoming_state = 0 ; //android command receive


void setup(){
  Serial.begin(9600);
  pinMode(relay_pin, OUTPUT);
  
  pinMode(DOPin, INPUT);
}

void loop()
{
  if (Serial.available() > 0){  //Looking for incoming data
    incoming_state = Serial.read() - '0';  //Reading the data
    digitalWrite(relay_pin, incoming_state);  //Making the LED light up or down
  }
  
  
  int chk = DHT.read11(DHT11_PIN);
  Serial.print("Temperature = ");
  Serial.println(DHT.temperature);
  Serial.print("Humidity = ");
  Serial.println(DHT.humidity);
  Serial.print("smoke = ");
  Serial.println( digitalRead(DOPin) ) ;

  delay(1000);
  
}

#===== imports =====
import paho.mqtt.client as paho
import serial
import time, logging

#===== logging =====
logging.basicConfig(level=logging.DEBUG)

#===== serial arduino =====
ser = serial.Serial('/dev/ttyACM0',9600)  # Watchout for address

#====== MQTT ====

def on_connect(client, userdata, flags, rc):
    print('ConnAck received with code %d.' %(rc) )
    client.subscribe('masud_pi/switch')


def on_message(client, userdata, msg):
    print(msg.topic+" "+str(msg.payload))
    data = int(msg.payload)
    data = str(data)
    logging.debug(data)
    ser.write(data.encode())

client = paho.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect('test.mosquitto.org', 1883)

client.loop_start()
 
#===== Imports =====
import paho.mqtt.client as paho
from gpiozero import MotionSensor
import logging

#===== pir set up =====
pir = MotionSensor(4)

#===== logger setup =====
logging.basicConfig(level = logging.DEBUG)

#===== MQTT client setup ======
def on_connect(client, userdata, flags, rc):
    print('ConnAck received with code %d.' %(rc) )
    client.subscribe('masud_pi/pir')
    
client = paho.Client()
client.on_connect = on_connect
client.connect('test.mosquitto.org', 1883)


#===== motion detection and sending loop and live video link =====
while True:
    client.publish('masud_pi/live', payload='http://192.168.192.1:8081')
    pir.wait_for_motion()
    print('Motion Detected')
    client.publish('masud_pi/pir', payload='someone is near')
    pir.wait_for_no_motion()
    client.publish('masud_pi/pir', payload='moved away')
    print('No Motion')
