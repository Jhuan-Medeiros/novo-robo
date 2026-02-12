# Teleoperated robot with forks elevator 

Video for visualizing it: https://www.youtube.com/shorts/v5waDBxI6mc

This project consists in a controlled robot that acts like a fork-lift and has a camera that can be used to read ArUco's.

## Components

- 1 RaspberryPi 3b+
- 4 motors with magnetic encoders
- 2 servos
- 1 Arduino UNO
- 2 ultrassonic distance sensors
- 2 infrared distance sensors
- 1 Webcam
- 1 buzzer
- 1 RGB Led
- 1 H-bridge
- 2 Line sensors

## Libraries

  - Pigpio
  - pygame
  - openCV


  # How to use

Install the Rpi libraries and in a separated computer install python with pygame, upload the arduino code in the board and connect it in the Rasp USB port along with the camera. Connect the raspberry in a WI-FI network and connect a computer in the same network to use the ssh bridge to communicate with de micro-computer. Compile all archives and use `sudo ./<archiveName>` to run it, in another terminal in Rpi use `source venv/bin/activate` `python receive.py` to connect the controller and in your pc use `python envio.py` to send the controller data to the Rasp.


## Usable information

estadoLinha: The reading of the line sensors. When receiveing `1` -> line is in the right; `2` -> line is in the left; `3` -> line is centered.
estadoLinha threshold: >= 600 `black`; < 600 `white`.
