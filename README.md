# Dancing Hexapod
This was my final project in Principles of Robotics. I enabled a hexapod (six-legged) mobile robot to detect the tempo of an audio signal being played, and perform a set of dance moves that are synchronized to the beat.

## Implementation
### Hardware
- Powered by a 7.2V rechargeable battery pack, controlled by Ardino UNO and Lynxmotion SSC-32U control boards
- Serial connection allows Arduino to offload computational burden of simultaneous control of servo motors to SSC-32
<img src="https://github.com/sebtona/dancing-hexapod/blob/main/control_boards.jpg" alt="Control boards" height="500"/>

- Servo motors for "shoulder" (stride) and "elbow" (lift) joints. 12 motors total
- Ultrasonic sensor mounted to front
- Electret microphones on front and back

<img src="https://github.com/sebtona/dancing-hexapod/blob/main/hexapod_front.jpg" alt="Front View" width="500"/> <img src="https://github.com/sebtona/dancing-hexapod/blob/main/hexapod_top.jpg" alt="Top View" width="500"/>

### Behavioral Flowchart

![Flowchart](https://github.com/sebtona/dancing-hexapod/blob/main/flowchart.png)

### C++ Arduino Script
- Establish serial communication with SSC-32
- Explored tripod, wave, and ripple gaits for walking and turning
- Modular "dance moves" for a customizable routine
- Algorithm to acquire audio signal and estimate tempo
  1. Capture and store audio samples from a microphone periodically
  2. Apply a Hamming window to the audio signal to improve clarity in the frequency domain
  3. Apply the Fast Fourier Transform to the windowed audio signal. The result is the frequency spectrum of the sample.
  4. Determine the highest peak of the frequency spectrum. This is the most prevalent frequency in the signal.
  5. Transform the peak frequency (f) to a reasonable one, if needed.\*
  6. Set the delay between robot frame transformations accordingly. (Delay T = 1/f)
  
  \* Sometimes, the most prevalent frequency detected is actually a harmonic of the true tempo of the audio signal. To achieve an accurate estimate of the true tempo, it may be necessary to multiply or divide the detected peak frequency by an even integer. To simplify the determination of whether this is necessary, I only tested the algorithm on audio signals with tempos in the 1-3 Hz (60-180 BPM) range; most musical pieces fall within this range. Any detected peak frequency outside of this range was multiplied or divided by 2 until a value within the proper range was reached.
- Libraries used:
  - "arduinoFFT" by Enrique Condes
  - "Vector" by Peter Polidoro
  
## Installation
1. Install the latest version of Arduino IDE on your machine.
2. Fork this repo or download the `sebton_hexapod_final_project` folder. This folder comes with the SSC-32 library included.
3. If for any reason the library does not work, you can download a C++ library for SSC-32 [here](https://github.com/RobotGrrl/Arduino-SSC32) and the SSC-32 Servo Sequencer Utility [here](http://www.lynxmotion.com/p-895-free-download-ssc-32-servo-sequencer-utility-created-using-flowbotics-studio.aspx).
4. Open the .ino file in the Arduino IDE. The SSC32 library files should automatically appear as tabs in your Arduino project.
5. In the Arduino IDE, navigate to Tools -> Manage Libraries. Search for and install the latest versions of the Arduino libraries "arduinoFFT" by Enrique Condes and "Vector" by Peter Polidoro.
6. Alternatively, you can download the arduinoFFT library [here](https://www.arduino.cc/reference/en/libraries/arduinofft/) and the Vector library [here](https://www.arduino.cc/reference/en/libraries/vector/). If you choose this option, place the library files in the same src folder as the .ino file.
7. You are now ready to connect your Arduino to your machine, compile and upload the code.

## Deliverables
- Operation Demonstration

https://user-images.githubusercontent.com/46403390/169385117-0a123f55-1380-4e40-82d6-676523af822a.mp4
- Project Presentation: https://github.com/sebtona/dancing-hexapod/blob/main/presentation.pptx
