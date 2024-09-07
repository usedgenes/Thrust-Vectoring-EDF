# Thrust Vectoring EDF
This is a project to build a device that uses an EDF (electronic ducted fan) to hover. Thrust vectoring vanes are used to redirect the fan's thrust, allowing the device to adjust its attitude to always remain straight up. The majority of the parts are 3D printed, although a 64mm EDF with an electronic speed controller (ESC) is required. 4 mini-servos with metal servo push rods are also used to control the vanes. A BNO08X is used to get data about the device's orientation, and an ESP32 is used to carry out the necessary computations. I have also designed an app (https://github.com/usedgenes/ESP-32-Interface) to communicate with the ESP32 through Bluetooth, allowing wireless control of the device. 

Video of the device in action: https://github.com/usedgenes/Thrust-Vectoring-EDF/blob/main/Testing/oops.MOV

Link to CAD files: https://cad.onshape.com/documents/8191817ad9d12dde26bb6902/w/bd0a7511ae38f79b886bed82/e/ad6374b1410adcaaa3a0f5ad?renderMode=0&uiState=66dbf606461f47676f2cd713

![](https://github.com/usedgenes/Thrust-Vectoring-EDF/blob/main/Picture.png)
