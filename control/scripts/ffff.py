import cv2
import numpy as np
import time
from jetracer.nvidia_racecar import NvidiaRacecar

car = NvidiaRacecar()

def initialize_trackbars():
    cv2.namedWindow('HSV_settings')
    cv2.resizeWindow('HSV_settings', 400, 250)
    trackbars = ['H_MAX', 'H_MIN', 'S_MAX', 'S_MIN', 'V_MAX', 'V_MIN']
    ranges = [180, 180, 255, 255, 255, 255]
    initial_values = [50, 12, 255, 0, 255, 200]
    for trackbar, range_max, init_val in zip(trackbars, ranges, initial_values):
        cv2.createTrackbar(trackbar, 'HSV_settings', 0, range_max, lambda x: None)
        cv2.setTrackbarPos(trackbar, 'HSV_settings', init_val)

def get_trackbar_values():
    values = {}
    for param in ['H_MAX', 'H_MIN', 'S_MAX', 'S_MIN', 'V_MAX', 'V_MIN']:
        values[param] = cv2.getTrackbarPos(param, 'HSV_settings')
    return values

def calculate_steering_direction(thresh):
    weighted_vertical = np.arange(300).reshape(300,1)
    sum_weights = np.sum(thresh / 255 * weighted_vertical, axis=0)
    direction = np.dot(sum_weights, (np.arange(300) - 150)**3) / 1000000
    return direction

initialize_trackbars()
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Can't open camera.")
    exit()

last_time = time.time()

while True:
    ret, frame = cap.read()
    if not ret:
        print('No frame')
        break

    frame_resized = cv2.resize(frame, (300, 300))
    hsv_frame = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2HSV)
    trackbar_values = get_trackbar_values()

    lower_bounds = np.array([trackbar_values['H_MIN'], trackbar_values['S_MIN'], trackbar_values['V_MIN']], dtype=np.uint8)
    upper_bounds = np.array([trackbar_values['H_MAX'], trackbar_values['S_MAX'], trackbar_values['V_MAX']], dtype=np.uint8)
    mask = cv2.inRange(hsv_frame, lower_bounds, upper_bounds)

    line_direction = calculate_steering_direction(mask)
    steering_d = np.clip(line_direction / 20, -0.5, 0.5)

    Throttle = 0.23
    car.steering = steering_d
    car.throttle = Throttle

    # Calculate and print frequency
    current_time = time.time()
    elapsed_time = current_time - last_time
    last_time = current_time
    frequency = 1 / elapsed_time if elapsed_time else 0
    print(f"lineDirection: {line_direction}, steering_d: {steering_d}, Throttle: {Throttle}, Frequency: {frequency:.2f} Hz")

    cv2.imshow('camera', frame_resized)
    cv2.imshow('mask', mask)

    if cv2.waitKey(1) != -1:
        car.steering = 0
        car.throttle = 0
        break

cap.release()
cv2.destroyAllWindows()
