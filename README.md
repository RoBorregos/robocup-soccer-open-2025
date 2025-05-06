
# Robocup Soccer Open 2025

Code for Robocup Soccer Open 2025 participating with Roborregos

## Authors

* @emilwinkp
* @brinez-juan

## Features

🤖 Processing Breakdown:
- Teensy 4.1: Organices data recolected and processed by camara, phototransistors, BNO, and manages them for core logic and movement. 
- OpenMV: Camaras with algorithms for ball and both goals detection using computer vision.

🎮 Movement Cotrol: Used 2 PID for a smother control ensuring a precise movement:

- Translational PID: Guides robot to the ball or goal with camara.
- Angular PID: Corrects the direction of the robot with angular velocity correction.
- Dribbler PID: Adjust robot movement for a soft approach to the ball with the robots front. 
- Ominidirectional movement: Omnidirectional movement using 4 wheels with 90 degree separation. 

⚽Ball detection: Image processing for pixel mapping using LAB values and merged blobs tracking to determine ball distance and angles in a [180, -180] angle normalization. 

🥅Goal detection: Using merged blobs to encapsulate goal in a rectangle with a corresponding center used as reference for distance and angle measurements. 

## Structure
<pre> ROBOCUP-SOCCER-OPEN-2025 ├── Striker └── Vision └── Teensy ├──
</pre>
## Tools

Make sure to have the corresponding dependencies for the teensy 4.1 in your enviroment, as well as the OpenMV IDE for vision codes in micropython. 

* [OpenMV IDE](https://openmv.io/pages/download?gad_source=1&gad_campaignid=21060752326&gclid=Cj0KCQjww-HABhCGARIsALLO6XzSvYDVwilt_5g-71gYgo7v3H16fBtwGKIoBNfmP_kiLpWk4qG5masaArXuEALw_wcB)

* [Adafruit](https://github.com/adafruit/Adafruit_BNO055/blob/master/Adafruit_BNO055.cpp)


## Documentation

[Poster for TMR competition](https://www.canva.com/design/DAGmp68Unzo/DkLrrPsporc31XkUFyraQg/edit?utm_content=DAGmp68Unzo&utm_campaign=designshare&utm_medium=link2&utm_source=sharebutton)

## Install dependencies

In order to use the board in the Arduino enviroment follow the instructions: 

1. Open Preferences in the menu bar of ArduinoIDE:
    - Windows/Linux: File > Preferences
    - macOS: Arduino IDE > Preferences
2. Locate the Additional board manager URLs setting toward the bottom.
3. To open the Additional Bards Manager URLs windows, click on the buton to the right of the text field.
4. There should be only one URL for each line, if not, change or remove URLs in the window. 
5. Add the URLs wanted and click Ok to confirm changes

Board Package for Teensy 4.1:

* [Teensy 4.1](https://www.pjrc.com/teensy/td_download.html)

## License

* [MIT](https://choosealicense.com/licenses/mit/)
