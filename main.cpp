#include "mbed.h"
#include "MPU6050.h"
#include "LCDi2c.h"

////////////////////////////////////////////////////////////
// Threads
////////////////////////////////////////////////////////////
// Stopping ESP32 camera
Thread stopEspCamThread;
// Reading data from AI Model
Thread dataAIModelThread;
// MPU Thread
Thread readMPUThread;
Thread printLCD;

/////////////////////////////////////////////////////////////////
//Variables
////////////////////////////////////////////////////////////////
static bool stopEsp = false;
Mutex coordinates_mutex;
const int tolerance = 1.0f;
DigitalOut ledGreen(PTD12);
DigitalOut ledRed(PTD13);

//////////////////////////////////////////////////////////////////
/// Interrupt Service 
// from sw 2 to stop esp 32 cam
////////////////////////////////////////////////////////////////////
void button_isr(){
    stopEsp = true;
}
// Button input pin
InterruptIn button(PTD11);  // switch 2 button

/////////////////////////////////////////////////////////////////////////
//ESP 32 Camera Control
//////////////////////////////////////////////////////////////////////////
static BufferedSerial serial_port(PTC4, PTC3, 9600); // UART pins
void stopEspCam() {
    while (true) {
        // Wait for the stop signal to be true
        if (stopEsp) {
            // Send stop command to ESP32
            static char c = 's';
            serial_port.write(&c, 1);
            printf("Stop command sent to ESP32\n");
            ledRed = 0;
            wait_us(1000000); //1s
            ledRed = 1;
            // Reset the stop signal to prevent sending it repeatedly
            stopEsp= false;
        }
        // Sleep for a while before checking again
        ThisThread::sleep_for(1000ms);  // 1 second delay
    }
}

/////////////////////////////////////////////////////////////////////////
// AI Communication
// USB for now (mqtt later implementation)
//////////////////////////////////////////////////////////////////////////
// Create a USB serial interface
BufferedSerial usb_serial(PTB17, PTB16, 9600);
char recv_buf[256];
int buf_idx = 0;

static float diff_x = 0;
static float diff_y = 0;

void processMsg() {
    float x, y;
    int parsed = sscanf(recv_buf, "Diff_X: %f, Diff_Y: %f", &x, &y);
    if (parsed == 2) {
        coordinates_mutex.lock();
        diff_x = x;
        diff_y = y;
        coordinates_mutex.unlock();
        printf ("%f, %f", diff_x, diff_y);
    }
}
void readFromUSB() {
    while (true) {
        if (usb_serial.readable()) {
             // Read one byte at a time
            char c;
            usb_serial.read(&c, 1);
            if (c == '\n') {
                recv_buf[buf_idx] = '\0';
                // process message
                processMsg();
                buf_idx = 0;
            }
            else if (buf_idx < sizeof(recv_buf) - 1) {
                    recv_buf[buf_idx++] = c;
            }
        }
    }
}

///////////////////////////////////////////////////////////////////////
// LCD
///////////////////////////////////////////////////////////////////////
LCDi2c lcd(PTC11, PTC10, LCD16x2); // for I2C LCD: SDA, SCL
void displayLCD() {
    const char* message = "Hi, I am Squirtle!\n"; 
    int len = 19;
    char buffer[17]; // 16 chars + null terminator

    while (true) {
        for (int i = 0; i < len; i++) {
            // Create a 16-char window for the message
            for (int j = 0; j < 16; j++) {
                buffer[j] = message[(i + j) % len];
            }
            buffer[16] = '\0';

            lcd.cls();
            lcd.locate(0, 0);
            lcd.printf("%s", buffer);

            ThisThread::sleep_for(300ms); // adjust for scroll speed
        }
    }

}
///////////////////////////////////////////////////////////////////////
//MPU - Accelerometer and Gyoscope
///////////////////////////////////////////////////////////////////////
MPU6050 mpu;

// The data would be used to stabilise the robot when it has a chassis
// for now just show it is being received
void readMPUData() {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
     if (!mpu.init()) {
        printf("MPU6050 failed to initialize!\n");
    } 
    while(true) {
        mpu.accel();
        mpu.gyro();

        printf("Accel: X=%f Y=%f Z=%f | Gyro: X=%f Y=%f Z=%f\n",
               mpu.accelX, mpu.accelY, mpu.accelZ,
               mpu.gyroX, mpu.gyroY, mpu.gyroZ);
        ledGreen = 0; //led on to show data is being received      
    }

}
///////////////////////////////////////////////////////////////////////
//Motor Control
////////////////////////////////////////////////////////////////////////
//Define output pins for A4988 (Motor 1)
DigitalOut dirPin(PTB3);   // Direction control
DigitalOut stepPin(PTB2);  // Step control

//Define output pins for A4988 (Motor 2)
DigitalOut dirPin2(PTB4);   // Direction control
DigitalOut stepPin2(PTB5);  // Step control
// PRINT FAILED HAVE TO AIT FOR SECOND MOTOR

// Define step delay
#define STEP_DELAY 10 // Microseconds

void stepper_forward_x(int steps) {
    dirPin = 1; // Set direction forward
    for (int i = 0; i < steps; i++) {
        stepPin = 1;
        wait_us(STEP_DELAY); // Pulse width
    }
}

void stepper_backward_x(int steps) {
    dirPin = 0; // Set direction backward
    for (int i = 0; i < steps; i++) {
        stepPin = 1;
        wait_us(STEP_DELAY);
    }
}

void stepper_forward_y(int steps) {
    dirPin2 = 1; // Set direction forward
    for (int i = 0; i < steps; i++) {
        stepPin2 = 1;
        wait_us(STEP_DELAY); // Pulse width
    }
}

void stepper_backward_y(int steps) {
    dirPin2 = 0; // Set direction backward
    for (int i = 0; i < steps; i++) {
        stepPin2 = 1;
        wait_us(STEP_DELAY);
    }
}

/////////////////////////////////////////////////////////////////
// Main Thread
/////////////////////////////////////////////////////////////////
int main() {
    button.rise(&button_isr);

    stopEspCamThread.start(stopEspCam); // stop esp cam
    dataAIModelThread.start(readFromUSB); // read AI data (coordinate difference)
    readMPUThread.start(readMPUData);
    printLCD.start(displayLCD);
    while (true) {
        float local_x, local_y;

        coordinates_mutex.lock();
        local_x = diff_x;
        local_y = diff_y;
        coordinates_mutex.unlock();

        if (local_x > tolerance) {
            stepper_forward_x(local_x);
        }
        else if (local_x < -tolerance) {
             stepper_backward_x(local_x);
        }

        if (local_y > tolerance) {
            stepper_forward_x(local_y);
        }
        else if (local_y < -tolerance) {
             stepper_backward_x(local_y);
        }

        ThisThread::sleep_for(10ms); // Small delay    
    }
}
