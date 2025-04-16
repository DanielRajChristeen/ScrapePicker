import cv2
import serial
import time
import random
import RPi.GPIO as GPIO
import threading

# ------------------ Bluetooth Serial Communication ------------------ #
# Make sure HC-05 is paired and bound to /dev/rfcomm0
arduino = serial.Serial('/dev/rfcomm0', 9600)  # Bluetooth serial port

def send_command(cmd):
    arduino.write(cmd.encode())
    print(f"Command Sent: {cmd}")

# ------------------ Servo Setup for Robotic Arm ------------------ #
servo_base = 17
servo_grip = 18

GPIO.setmode(GPIO.BCM)
GPIO.setup(servo_base, GPIO.OUT)
GPIO.setup(servo_grip, GPIO.OUT)

pwm_base = GPIO.PWM(servo_base, 50)
pwm_grip = GPIO.PWM(servo_grip, 50)

pwm_base.start(7.5)
pwm_grip.start(7.5)

def pick_scrap():
    print("Picking up scrap...")
    pwm_base.ChangeDutyCycle(6.5)  # Lower arm
    time.sleep(1)
    pwm_grip.ChangeDutyCycle(10)   # Close gripper
    time.sleep(1)
    pwm_base.ChangeDutyCycle(7.5)  # Lift arm
    time.sleep(1)

# ------------------ Ultrasonic Obstacle Detection ------------------ #
# Single signal pin for both trigger and echo
SIGNAL_PIN = 23  # Set to your signal pin GPIO pin

GPIO.setup(SIGNAL_PIN, GPIO.OUT)

obstacle_detected = False

def check_obstacle():
    global obstacle_detected
    while True:
        # Send a trigger pulse
        GPIO.output(SIGNAL_PIN, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(SIGNAL_PIN, GPIO.LOW)
        
        # Measure pulse duration (using GPIO.input)
        start_time = time.time()
        while GPIO.input(SIGNAL_PIN) == GPIO.LOW:
            start_time = time.time()
        
        # Wait for pulse to come back
        while GPIO.input(SIGNAL_PIN) == GPIO.HIGH:
            end_time = time.time()
        
        # Calculate distance (time taken for pulse to travel back)
        duration = end_time - start_time
        distance = duration * 17150  # Speed of sound in cm
        obstacle_detected = distance < 15
        time.sleep(0.2)

# Start ultrasonic in background
threading.Thread(target=check_obstacle, daemon=True).start()

# ------------------ Vision System for Scrap Detection ------------------ #
cap = cv2.VideoCapture(0)

def center_scrap_and_pick():
    send_command('S')
    pick_scrap()
    send_command('B')
    time.sleep(0.5)
    send_command('S')

# ------------------ Main Autonomous Loop ------------------ #
try:
    while True:
        if obstacle_detected:
            send_command('S')
            time.sleep(0.5)
            send_command(random.choice(['L', 'R']))
            time.sleep(0.5)
            send_command('F')
        else:
            ret, frame = cap.read()
            if not ret:
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(gray, (5, 5), 0)
            _, thresh = cv2.threshold(blur, 160, 255, cv2.THRESH_BINARY_INV)

            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            scrap_found = False

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if 100 < area < 1500:
                    x, y, w, h = cv2.boundingRect(cnt)
                    cx = x + w // 2
                    cy = y + h // 2

                    if cx < 280:
                        send_command('L')
                    elif cx > 360:
                        send_command('R')
                    else:
                        send_command('F')
                        time.sleep(0.7)
                        center_scrap_and_pick()

                    scrap_found = True
                    break

            if not scrap_found:
                send_command('F')

            cv2.imshow("Scrap Detection", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

except KeyboardInterrupt:
    print("Stopped by user")
finally:
    send_command('S')
    cap.release()
    cv2.destroyAllWindows()
    pwm_base.stop()
    pwm_grip.stop()
    GPIO.cleanup()