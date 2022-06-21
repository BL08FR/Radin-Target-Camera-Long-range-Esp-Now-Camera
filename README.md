# Radin target camera (Long-range-Esp-Now-Camera)

Camera sending photo over ESP-NOW (long range) and display it on webserver (classic wifi).
The camera must be able to see the "router AP" (box or phone classic wifi AP) at startup not to connect on it but to find the correct wifi channel.
The receiver connect to the "router AP" and spread webserver on it to show photos, it receive the photo sent by the camera and update the web page.

Made with ESP32CAM modules (ai thinker / HW818).

Inspired from Randomnerdtutorial for the webserver: https://randomnerdtutorials.com/esp32-esp-now-wi-fi-web-server/

Inspired from TenoTrash for the RSSI calculation: https://github.com/TenoTrash/ESP32_ESPNOW_RSSI

Inspired from Tal Ofer for Esp-now transfert: https://github.com/talofer99/ESP32CAM-Capture-and-send-image-over-esp-now

This is the screen portion of the code for more informations:
https://www.youtube.com/watch?v=0s4Bm9Ar42U

Horizontal and vertical boxes for HW818 : https://www.thingiverse.com/thing:5253848

/!\Note: it don't use the LR (low rate/ long range) mode of Esp-Now, first because low rate and second because not compatible with classic wifi on the receiver.

[23/02/2022] Receiver update to use directionnal antenna and stay within the law, user settings more accessible.

[21/06/2022] CameraESP-NOW_v1_1spiffs : Camera module use now SPIFFS memory to save the picture inducing picture refresh faster (+-5s for 640p) and it's no more necessary to use a micro-SD card.

NB: To handle ESP32 arduino IDE need to have python installed!

NB2: Some other's chinese board gave me worst results (approx. -20% of missing signal force that significantly reduce the range), choose quality board's from recognized seller's and be very careful when you modify it for the external antenna
