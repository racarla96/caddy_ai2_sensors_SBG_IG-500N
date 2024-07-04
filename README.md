# caddy_ai2_sensors_SBG_IG-500N

Este repositorio tiene el objetivo de guardar los documentos, CADs, programas, código del sensor y tener un driver funcional para ROS 2.

## Instalación del driver

```bash
cd caddy_ai2_sensors_SBG_IG-500N/sdk/sbgCom/
mkdir build
cd build
# Si la arquitectura es BIG_ENDIAN: cmake -DSBG_PLATFORM_ENDIANNESS=BIG ..
cmake .. 
make
sudo make install
```

## Instalar un alias al puerto serie de la IMU

```bash
cd caddy_ai2_sensors_SBG_IG-500N/startup
sudo chmod +x initenv.sh
sudo sh initenv.sh
```

Si estaba conectado previamente, conectar y desconectar y comprobar que efectivamente aparece el nombre de sbg

```bash

```

## ROS 2 Driver


## Testear la IMU con el paquete oficial

```bash
cd caddy_ai2_sensors_SBG_IG-500N/docs_official/SDK_3.2/USB/IG-Devices/Software\ Development/sbgCom/
cd projects/unix/
chmod +x build.sh
sudo ./build.sh SBG_PLATFORM_LITTLE_ENDIAN
```
### Como probar los ejemplos

```bash
cd ../../../Examples/ig500Continuous/
sed -i 's/\(sbgComInit("\)COM6\(", 115200, &protocolHandle) == SBG_NO_ERROR\)/\1\/dev\/ttyUSB0\2/' src/ig500Continuous.c
chmod +x projects/unix/build.sh
sudo ./projects/unix/build.sh
```

Nota: Ajuste /dev/ttyUSB0 según el puerto serie de su dispositivo. Para ver los dispositivos dispositivos recientemente conectados puedes usar:

```bash
sudo dmesg | grep tty
```