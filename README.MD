# 📚 Communicate And Control Relay, Read Sensor Data With ESP NOW And LVGL.

## 🗯️ About this project  
- 🌟ESP-NOW is a form of IOT local, that is a network of devices that communicate directly without going over the Internet.
- 🌟I'm using two ESP32 board for communicate.Change your board type in [platformio.ini](https://github.com/pangcrd/espnow-lvgl/blob/main/ScreenLVGL-master/platformio.ini) file.  
- 🌟Change your DHT PIN and relay GPIO in [main.cpp](https://github.com/pangcrd/espnow-lvgl/blob/main/ESP-Slave/src/main.cpp) file.
- 🌟Using ESP32-2432S028R v3 Cheap Yellow Display for master device. And another ESP32 board for slave.  
- 🌟Get the MAC address for 2 devices by running the code in [GetMACaddress](https://github.com/pangcrd/espnow-lvgl/blob/main/GetMACaddress/main.cpp) folder.
- 🌟In this test i use DHT22 temperature and Humidity sensor + 4 relay channel for board ESP32 slave.
- 🌟Design UI on [Squareline Studio](https://squareline.io/)V1.4 

## 🛠 Some function
- ⚡Enable or disable all switches. Each can be turned on and off individually.
- ⚡Displays board ID and connection status.
- ⚡Displays temperature and humidity values ​​and graphs.  
  
## 🔎 Pinout
![schematic](https://github.com/pangcrd/espnow-lvgl/blob/main/images/cyd.png)  
![schematic](https://github.com/pangcrd/espnow-lvgl/blob/main/images/ESP32-WROOM-1.png)

## 📷 Some picture
<table>
  <tr>
    <td><img src="https://github.com/pangcrd/espnow-lvgl/blob/main/images/20250210_111911.jpg" alt="Image 1" width="400"/></td>
    <td><img src="https://github.com/pangcrd/espnow-lvgl/blob/main/images/20250210_111823.jpg" alt="Image 2" width="400"/></td> 
  </tr>
</table>  

<table>
  <tr>
    <td><img src="https://github.com/pangcrd/espnow-lvgl/blob/main/images/20250210_111833.jpg" alt="Image 1" width="400"/></td>
    <td><img src="https://github.com/pangcrd/espnow-lvgl/blob/main/images/20250210_111846.jpg" alt="Image 2" width="400"/></td>  
  </tr>
</table>   
## 🎬 Demo video test 👇  

📺 [Youtube](https://youtube.com/shorts/Y3YMU5yD0dk).

