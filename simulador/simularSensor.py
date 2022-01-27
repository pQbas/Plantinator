#!/usr/bin/python3
#coding=utf-8

import paho.mqtt.client as mqtt
import time

topicToSubscribe = "robot/bandejas/alimentadora/cantidad"


def on_message(client, userdata, message):
  try:
    print("message received " ,str(message.payload.decode("utf-8")))
    print("message topic=",message.topic)
    print("message qos=",message.qos)
    print("message retain flag=",message.retain)
  except e:
    print("error con el mensaje")


broker_address="127.0.0.1" #"iot.eclipse.org"
print("creating new instance")
client = mqtt.Client("P1") #create new instance mqtt.Client()
client.on_message = on_message
print("connecting to broker")
client.connect(broker_address, 1883, 60) #connect to broker

client.loop_start() #start the loop
print("Subscribing to topic",topicToSubscribe)
client.subscribe(topicToSubscribe)
print("Publishing message to topic",topicToSubscribe)

#time.sleep(4) # wait




while True:
  time.sleep(1)
  print ("ingrese temperatura a enviar:")
  #temperatura = input()
  temperatura = raw_input( )
  print ("se enviará el texto: " + str(temperatura))
  client.publish(topicToSubscribe,str(temperatura))

#client.loop_forever()
#client.loop_stop() #stop the loop