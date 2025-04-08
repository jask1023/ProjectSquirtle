# SEP600 - Squirtle Stabilization & Object Detection

This project combines MPU6050-based motion sensing, I2C LCD display, and computer vision using YOLOv8 with a Flask web interface. The system is used to detect fire and respond accordingly.

---

## Project Setup

###  1. Keil Studio (for K66F Microcontroller)

1. **Open [Keil Studio Cloud](https://keil.arm.com/studio)**  
   (or use the desktop version if you're set up locally).

2. **Add the following libraries to your project**:

   - [MPU6050 Library](https://os.mbed.com/users/hudakz/code/MPU6050//file/9c2bb0f94c31/MPU6050.h/)
   - [mbedLCDi2c Library](https://os.mbed.com/users/sstaub/code/mbedLCDi2c/)

3. **Use `main.cpp` to program your K66F board**.

---

### 2. Arduino IDE (for `.ino` files)

For any Arduino `.ino` files in the repo, open and edit them using the [Arduino IDE](https://www.arduino.cc/en/software) or the desktop app.
These files are run on the ESP32 Wrover module to detect fire.

---

## 3. Flask + YOLOv8 Server

This section handles object detection.

### Run the Flask App

1. Open terminal and navigate to the folder with `web_server.py`.

2. Create a virtual environment (optional but recommended):
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
3. Install required packages:
   ```bash
   pip install opencv-python flask ultralytics
4. Run the app:
     ```bash
   python web_server.py
