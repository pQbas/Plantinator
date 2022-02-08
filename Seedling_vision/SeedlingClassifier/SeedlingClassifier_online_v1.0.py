import cv2
import numpy as np
import pyrealsense2 as rs
from pyleafarea import pyAreaCalc,pyTriangulateAndArea
from sklearn.svm import LinearSVC
from time import time,sleep,localtime
import pickle
import sys
from numpy.fft import fft2,fftshift,ifft2,ifftshift
from skimage.exposure import match_histograms
from paho.mqtt.client import Client as mqttClient
import warnings
warnings.filterwarnings("ignore")

sys.path.insert(1,"../")
from modbus_mqtt.libseedlingmodbus import SeedlingModbusClient
from modbus_mqtt import libseedlingmodbus as lsmodb
from common_functions import *
from Ericks_system import ericks_functions


def on_connect_(self, userdata, flags, rc):
    print("Connected MQTT with result code " + str(rc))
    main_mqtt_client.subscribe("PUT_HERE_THE_TOPIC_OF_INTEREST_tinterest")

def on_message_(self, userdata, msg):
    topic = str(msg.topic)
    payload = str(msg.payload.decode('utf-8'))
    print("payload: " + payload)
    print("topic: " + topic)
    global CV_system_switch
    if topic is "SysP":
        CV_system_switch = "SysP"
        print("CV system switched to Paulo's system")
    elif topic is "SysE":
        print("CV system switched to Erick's system")
        CV_system_switch = "SysE"
    else:
        print("CV system switched to default")
        CV_system_switch = "SysP"

args = sys.argv

if "-debug" in args:
    print("Entering debug mode ...")

#GUI and mouse
#cv2.namedWindow("Results")
#cv2.setMouseCallback("Results", click_depth_rgb)

#SOME GLOBAL VARIABLES AND CONSTANTS
#set depth camera parameters
distance=0.359
depth_scale=9.999999747378752e-05
intrinsics=rs.intrinsics()
intrinsics.width=1280
intrinsics.height=720
intrinsics.ppx=639.399
intrinsics.ppy=350.551
intrinsics.fx=906.286
intrinsics.fy=905.369
intrinsics.model=rs.distortion.inverse_brown_conrady
intrinsics.coeffs=[0.0,0.0,0.0,0.0,0.0]
CV_system_switch = "SysP"
ODD_RGB = cv2.imread("Offline_files/IMG_17_50_40.jpg",cv2.IMREAD_COLOR)
ODD_DEPTH = np.load("Offline_files/IMG_17_50_40.npy")
#ODD_RGB = cv2.imread("../datasets/seedlings_18_06_2021/IMG_15_5_36.jpg",cv2.IMREAD_COLOR)
#ODD_DEPTH = np.load("../datasets/seedlings_18_06_2021/IMG_15_5_36.npy")
EVEN_RGB = cv2.imread("Offline_files/IMG_15_38_14.jpg",cv2.IMREAD_COLOR)
EVEN_DEPTH = np.load("Offline_files/IMG_15_38_14.npy")
#EVEN_RGB = cv2.imread("../datasets/seedlings_18_06_2021/IMG_14_32_44.jpg",cv2.IMREAD_COLOR)
#EVEN_DEPTH = np.load("../datasets/seedlings_18_06_2021/IMG_14_32_44.npy")
CV_MODE = "online"
SAVE_IMAGES = False

## OPEN MODELS
#Paulo's CV system related models
#Erick's CV system related models


args = sys.argv

modServerIp = "192.168.1.103"
modServerPort = 502
mqttBrokerIp = "192.168.1.103"
mqttBrokerPort = 1883
plugTrayNum = None

if "-serverIp" in args:
    idx = args.index("-serverIp")
    try:
        modServerIp = args[idx+1]
    except:
        raise Exception("Server's IP wasn't specified")

if "-serverPort" in args:
    idx = args.index("-serverPort")
    try:
        modServerPort = int(args[idx+1])
    except:
        raise Exception("Server's Port wasn't specified or is not valid" )

if "-brokerIp" in args:
    idx = args.index("-brokerIp")
    try:
        modServerIp = args[idx+1]
    except:
        raise Exception("Broker's IP wasn't specified")

if "-brokerPort" in args:
    idx = args.index("-brokerPort")
    try:
        modServerPort = int(args[idx+1])
    except:
        raise Exception("Broker's Port wasn't specified or is not valid" )

if "-trayNum" in args:
    idx = args.index("-trayNum")
    try:
        plugTrayNum = int(args[idx + 1])
    except:
        raise Exception("Tray number wasn't specified or is not valid")

#INITIALIZE MAIN MQTT CLIENT
main_mqtt_client = mqttClient()
#try:
#    if main_mqtt_client.connect(mqttBrokerIp,mqttBrokerPort) is True:
#        print("MQTT connection -> Successful")
#    else:
#        print("MQTT connection -> Failed")
#except:
#    print("MQTT connection -> Failed")

if main_mqtt_client.is_connected():
    main_mqtt_client.on_connect = on_connect_
    main_mqtt_client.on_message = on_message_

modbusClient = SeedlingModbusClient(modServerIp,modServerPort)
modbusClientConnectedFlag = modbusClient.connectToServer()
if modbusClientConnectedFlag is True:
    print("Modbus Client's connection -> successful")
else:
    print("Modbus Client's connection -> failed")

#INITIALIZE COMPUTER VISION SYSTEMS
#Paulo's CV
cvSystem = seedlingClassifier(intrinsics)
with open("colors_ellipsoids_dict.pkl","rb") as file:
    cvSystem.ellipsoids_dict = pickle.load(file)
    file.close()
file = open("leaf_area_seedling_classifier.pkl", "rb")
cvSystem.seedlingClassifierModel = pickle.load(file)
file.close()

file = open("color_segmentation_svc.pkl", "rb")
cvSystem.colorSegmentationModel = pickle.load(file)
file.close()

cvSystem.modbusConnect(modbusClient)
if CV_MODE is "online":
    cvSystem.cameraInitialize()
print("Computer vision system initialization -> successfull")
#Erick's CV
cvSystem2 = ericks_functions.ErickSeedlingClassifier(modbusClient)


if modbusClientConnectedFlag is True:
    modbusClient.writeCvStatus(lsmodb.CV_WAITING_STAT)
plcInstruction = lsmodb.PLC_PROCODD_INST

while True:
    if modbusClientConnectedFlag is True:
        plcInstruction = modbusClient.getPLCInstruction()
    if plcInstruction == lsmodb.PLC_PROCODD_INST:
        if CV_system_switch == "SysP":
            if cvSystem.modbusConnectedFlag is True:
                cvSystem.modbusClient.writeCvStatus(lsmodb.CV_PROCESSING_STAT)
            if CV_MODE == "offline":
                cvSystem.rgbImg = ODD_RGB
                cvSystem.depthImg = ODD_DEPTH
            rgbGUI = cvSystem.processSeedlings("odd",CV_MODE) # IM CHANGING CONSTANTLY THIS PARAMETER
            if CV_MODE == "online":
                #SAVE IMAGES
                if SAVE_IMAGES is True:
                    ltime = localtime()
                    if plugTrayNum is not None:
                        name = "images/IMG{}_{}_{}_{}".format(plugTrayNum,ltime.tm_hour, ltime.tm_min,ltime.tm_sec)
                    else:
                        name = "images/IMG_{}_{}_{}".format(ltime.tm_hour, ltime.tm_min,ltime.tm_sec)
                    cv2.imwrite(name+".png",cvSystem.rgbImg)
                    np.save(name+".npy",cvSystem.depthImg)
        elif CV_system_switch is "SysE":
            if CV_MODE is "offline":
                cvSystem.rgbImg = ODD_RGB
                cvSystem.depthImg = ODD_DEPTH
            seedlings_mask,cones_mask = cvSystem.onlysegmentation(CV_MODE)
            segmentedImg = cv2.bitwise_and(cvSystem.rgbImg,cvSystem.rgbImg,mask=seedlings_mask)
            cvSystem2.processImage(segmentedImg)
            rgbGUI = segmentedImg.copy()
        else:
            print("WARNING: Seedling Classifier system wasn't specified")
    elif plcInstruction == lsmodb.PLC_PROCEVEN_INST:
        if CV_system_switch is "SysP":
            if cvSystem.modbusConnectedFlag is True:
                cvSystem.modbusClient.writeCvStatus(lsmodb.CV_PROCESSING_STAT)
            if CV_MODE is "offline":
                cvSystem.rgbImg = EVEN_RGB
                cvSystem.depthImg = EVEN_DEPTH
            rgbGUI = cvSystem.processSeedlings("even",CV_MODE)
            if CV_MODE == "online":
                # SAVE IMAGES
                if SAVE_IMAGES is True:
                    ltime = localtime()
                    if plugTrayNum is not None:
                        name = "images/IMG{}_{}_{}_{}".format(plugTrayNum, ltime.tm_hour,ltime.tm_min, ltime.tm_sec)
                    else:
                        name = "images/IMG_{}_{}_{}".format(ltime.tm_hour, ltime.tm_min,ltime.tm_sec)
                    cv2.imwrite(name + ".png", cvSystem.rgbImg)
                    np.save(name + ".npy", cvSystem.depthImg)
        elif CV_system_switch is "SysE":
            if CV_MODE is "offline":
                cvSystem.rgbImg = EVEN_RGB
                cvSystem.depthImg = EVEN_DEPTH
            seedlings_mask, cones_mask = cvSystem.onlysegmentation(CV_MODE)
            segmentedImg = cv2.bitwise_and(cvSystem.rgbImg, cvSystem.rgbImg, mask=seedlings_mask)
            cvSystem2.processImage(segmentedImg)
            rgbGUI = segmentedImg.copy()
        else:
            print("WARNING: Seedling Classifier system wasn't specified")
    try:
        #cv2.imshow("Results",rgbGUI)
        cv2.waitKey(15)
    except:
        pass
cv2.destroyAllWindows()
