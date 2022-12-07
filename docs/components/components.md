# Component List
<!-- 
##### Microcontroller 
  - Chip [ATmega328]( https://www.microchip.com/wwwproducts/en/ATmega328)
    - [Arduino Uno Rev 3]( https://www.electrokit.com/produkt/arduino-uno-mega328-rev-3/ )
    - [Adafruit Feather 328P]( https://www.electrokit.com/produkt/adafruit-feather-328p/ ) 
  - [Teensy LC]( https://www.electrokit.com/produkt/teensy-lc/ )
##### Altimeter (Acceleration / Altitude)
  - Barometer: [BMP280]( https://www.electrokit.com/produkt/temperatur-och-lufttryckssensor-pa-kort-bmp280/ )
  - Accelerometer: [ADXL345 3-axis accelerometer breakout]( https://www.electrokit.com/en/product/adxl345-3-axis-accelerometer-breakout/ ) / [MPU-6050 accelerometer 3-axis & gyro breakout]( https://www.electrokit.com/en/product/mpu-6050-accelerometer-3-axel-monterad-pa-kort-2/ ) / [ADXL377](https://www.electrokit.com/produkt/adxl377-accelerometer-3-axel-%c2%b1200g/) /
  - IMU(absolute): [Rörelsegivare 10DOF MPU6050 MMC5883 BMP085]( https://www.electrokit.com/produkt/imu-10dof-mpu-6050-hmc5883l-bmp085/ ) / [6DOF LSM6DSO32](https://www.electrokit.com/produkt/rorelsegivare-6dof-lsm6dso32/)/
##### Recovery system
  - GPS: [GM862 GSM/GPRS/3G-module]( https://www.electrokit.com/en/product/telit-gm862-gsm-gprs-3g-modul-med-gps/ ) / 
  - Ping: 
  - Camera: [Arduino Camera ov7670]( https://www.electrokit.com/produkt/kameramodul-640x480-ov7670-med-fifo/ ) /
-->

| Part            | Module                   | Datasheet                        | Other                                        | Used in design           |
| --------------- | ------------------------ | -------------------------------- | -------------------------------------------- | ------------------------ |
| Microcontroller | [LilyGo TTGO LoRa32]     | [Datasheet][TTGO datasheet]      | [Pin Diagram][Pin Diagram], [TTGO GitHub]    | :heavy_check_mark:       |
| IMU             | [MPU6050 MMC5883 BMP085] |                                  | Out of stock                                 | :heavy_multiplication_x: |
| IMU             | [LSM6DSO32]              | [Datasheet][LSM6DSO32 datasheet] |                                              | :heavy_check_mark:       |
| Barometer       | [BMP280]                 | [Datasheet][BMP280 datasheet]    |                                              | :heavy_check_mark:       |
| Accelerometer   | [MPU6050]                | [Datasheet][MPU6050 datasheet]   |                                              | :heavy_check_mark:       |
| GPS             | TBD                      |                                  |                                              |                          |
| Flashminne      | [23LC1024]               | [Datasheet][23LC1024 datasheet]  | |                                            |                          |


[LilyGo TTGO LoRa32]: http://www.lilygo.cn/prod_view.aspx?TypeId=50060&Id=1326&FId=t3:50060:3
[TTGO datasheet]: https://www.espressif.com/sites/default/files/documentation/esp32-pico-d4_datasheet_en.pdf
[TTGO Pin Diagram]: https://raw.githubusercontent.com/LilyGO/TTGO-LORA32/master/images/T3_2_0.jpg
[TTGO Github]: https://github.com/LilyGO/TTGO-LORA32-V2.0
[Pin Diagram]: https://raw.githubusercontent.com/LilyGO/TTGO-LORA32/master/images/T3_2_0.jpg
[23LC1024]: https://www.electrokit.com/en/product/23lc1024-dip-8-sram-1mbit-spi/
[23LC1024 datasheet]: https://www.electrokit.com/uploads/productfile/41017/23LC1024.pdf

[MPU6050 MMC5883 BMP085]: https://www.electrokit.com/produkt/imu-10dof-mpu-6050-hmc5883l-bmp085/

[LSM6DSO32]: https://www.electrokit.com/produkt/rorelsegivare-6dof-lsm6dso32/
[LSM6DSO32 datasheet]: https://www.st.com/resource/en/datasheet/lsm6dso32.pdf

[BMP280]: https://www.electrokit.com/produkt/temperatur-och-lufttryckssensor-pa-kort-bmp280/
[BMP280 datasheet]: https://www.electrokit.com/uploads/productfile/41013/BMP280.pdf

[ADXL345]: https://www.electrokit.com/en/product/adxl345-3-axis-accelerometer-breakout/
[ADXL345 datasheet]: https://www.analog.com/media/en/technical-documentation/data-sheets/ADXL345.pdf

[ADXL377]: https://www.electrokit.com/produkt/adxl377-accelerometer-3-axel-%c2%b1200g/
[ADXL377 datasheet]: https://www.analog.com/media/en/technical-documentation/data-sheets/ADXL377.pdf

[MPU6050]: https://www.electrokit.com/en/product/mpu-6050-accelerometer-3-axel-monterad-pa-kort-2/
[MPU6050 datasheet]: https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf

[OV5647 datasheet]:  https://cdn.sparkfun.com/datasheets/Dev/RaspberryPi/ov5647_full.pdf
