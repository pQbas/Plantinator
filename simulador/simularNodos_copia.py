#!/usr/bin/python3
#coding=utf-8

''' este programa lee el fichero asociado, y cuando detecta un cambio envía el valor
a todos los nodos'''

import paho.mqtt.client as mqtt
import os, time


fichero = "nodos.txt"

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

#este proceso no se usa
def leer_fichero_suscribir_topic():
  try:
    with open(fichero) as file: 
      for line in file.readlines():
        line = line.replace('\r', '').replace('\n', '')
        if not line.startswith("#"):
          topic, value = line.split("=")
          print("Suscribiendo a",topic)
          client.subscribe(topic)
  except: 
    print('error leyendo fichero')


def leer_fichero_envia_topic():
  try:
    with open(fichero) as file: 
      for line in file.readlines():
        line = line.replace('\r', '').replace('\n', '')
        if not line.startswith("#") and len(line)>5:
          topic, value = line.split("=")
          print("Enviando a",topic,value)
          client.publish(topic,str(value))
  except: 
    print('error leyendo fichero')


leer_fichero_envia_topic()
cont = 0
moddate = os.stat(fichero)[8]
while 1:
  time.sleep(2)
  if moddate != os.stat(fichero)[8]:
    cont = cont + 1
    moddate = os.stat(fichero)[8]
    print("cambio el fichero " + str(cont))
    leer_fichero_envia_topic()


