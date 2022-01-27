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

- paho.mqtt

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