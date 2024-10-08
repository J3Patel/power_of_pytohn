import cv2
import mediapipe as mp
import math
import serial
import time

# Initialize MediaPipe Hand model and drawing utility
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils

# Initialize the webcam
cap = cv2.VideoCapture(0)

# Initialize serial communication with Arduino (modify the COM port as needed)
# On macOS, the port might be something like '/dev/tty.usbmodem14101' or similar
arduino = serial.Serial('/dev/cu.usbmodem21401', 9600)  # Replace with your Arduino's serial port
time.sleep(2)  # Wait for the serial connection to initialize

# Function to calculate the Euclidean distance between two points
def calculate_distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

# Function to calculate the normalized distance between two points
def is_hands_far_apart(distance, frame_width, frame_height, threshold=0.8):
    # Calculate the diagonal length of the frame
    diagonal_length = math.sqrt(frame_width**2 + frame_height**2)
    
    # Normalize the distance by dividing by the diagonal length
    normalized_distance = distance / diagonal_length
    
    # Check if the normalized distance exceeds the threshold
    return distance > 800

# Setup Hand model
with mp_hands.Hands(min_detection_confidence=0.7, min_tracking_confidence=0.7) as hands:
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        # Flip the frame horizontally for a mirrored effect
        frame = cv2.flip(frame, 1)

        # Get the frame dimensions
        height, width, _ = frame.shape

        # Convert the BGR image to RGB
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Process the frame and detect hands
        result = hands.process(rgb_frame)

        # Draw the hand annotations on the frame
        if result.multi_hand_landmarks:
            palms = []

            for hand_landmarks in result.multi_hand_landmarks:
                # Draw landmarks on the frame
                mp_drawing.draw_landmarks(
                    frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

                # Getting the position of the wrist (landmark 0, center of palm)
                palm_center = hand_landmarks.landmark[0]
                x = int(palm_center.x * width)
                y = int(palm_center.y * height)

                # Append the palm center coordinates
                palms.append((x, y))

                # Draw a circle at the palm center
                cv2.circle(frame, (x, y), 10, (0, 255, 0), -1)

            # If two hands are detected, calculate and check the distance between palms
            if len(palms) == 2:
                x1, y1 = palms[0]
                x2, y2 = palms[1]
                distance = calculate_distance(x1, y1, x2, y2)

                # Check if the hands are far apart using a normalized threshold
                if is_hands_far_apart(distance, width, height):
                    cv2.putText(frame, 'Hands are far apart', (50, 100),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                    arduino.write(b'1')  # Send '1' to Arduino
                else:
                    cv2.putText(frame, 'Hands are close', (50, 100),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    arduino.write(b'0')  # Send '0' to Arduino

                # Display the actual distance for debugging purposes
                cv2.putText(frame, f'Distance: {int(distance)}', (50, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

        # Display the resulting frame
        cv2.imshow('Palm Detection', frame)

        # Break loop with 'q' key
        if cv2.waitKey(5) & 0xFF == ord('q'):
            break

# Release the capture and close OpenCV windows
cap.release()
cv2.destroyAllWindows()

# Close the Arduino serial connection
arduino.close()
