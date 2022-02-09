# **Plantinator**


## **Requisitos**

### Node-Red Package Requirements:

- node-red v1.3.5
- node-red-contrib-fs-ops v1.6.0
- node-red-contrib-google-cloud v0.0.24
- node-red-dashboard v2.29.1
- node-red-node-rbe v0.5.0
- node-red-node-tail v0.3.1

### Requirements del equipo

- NodeJS v12.x
- Brokker Mosquitto


### Requirements de Python

- opencv-python~=4.4.0.46
- numpy~=1.19.4
- pyrealsense2~=2.40.0.2483
- Pillow~=8.1.0
- pymodbus~=2.4.0
- future~=0.18.2
- paho-mqtt~=1.5.1
- scikit-learn~=0.24.0
- scikit-image~=0.17.2
- matplotlib~=3.3.3
- open3d~=0.12.0

--------------------

## **Pasos para iniciar el programa**

                
Inicializar el programa node-red

`$ node-red-start`

Inicializar el simulador de envio de datos ubicado en la carpeta **. /simulador**

`$ python3 simularNodos.py`


----------------------
## **Problemas relacionados al programa**

- ***¿ No se cargan las imagenes en la plataforma ?***

     Revise el archivo **/home/user/.node-red/settings.js** y asegurese que dentro de **module.exports** este escrita la siguiente linea:

     ```
     httpStatic: 'home/user/Plantinator/nodered-static/'
     ```

- ***¿ No se logra la comunicación MQTT ?***
     
     Asegurese de haber instalado el broker Mosquito.

     ```
     sudo apt-add-repository ppa:mosquitto-dev/mosquitto-ppa.
     sudo apt-get update.
     sudo apt-get install mosquitto.
     sudo apt-get install mosquitto-clients.
     sudo apt clean
     ```
    

## Dudas frecuentes durante la instalación

- ***¿Cómo desinstalar node-red?***
     ```
     sudo npm -g remove node-red
     sudo npm -g remove node-red-admin
     rm -r ~/.node-red
     ```
- ***¿Cómo instalar una versión especifica de node red?***

    ```
    npm install -g --unsafe-perm node-red@1.3.5
    ```
- ***¿Cómo instalar una versión especifica de algun paquete de node red?***
        
    Para que el paquete sea reconocido, el comando se debe ejecutar en /home/user/.node-red/node_modules

    ```
    npm install <package>@<version>
    ```
