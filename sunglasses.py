import cv2
import numpy as np

# Load Haar cascades
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
eye_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_eye.xml')

# Load sunglasses image with alpha channel
sunglasses = cv2.imread('sunglasses.png', cv2.IMREAD_UNCHANGED)
if sunglasses is None:
    print("Error: sunglasses.png not found!")
    exit()

# Start webcam
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect faces
    faces = face_cascade.detectMultiScale(gray, 1.1, 5)
    for (x, y, w, h) in faces:
        roi_gray = gray[y:y+h, x:x+w]
        roi_color = frame[y:y+h, x:x+w]

        # Detect eyes within face
        eyes = eye_cascade.detectMultiScale(roi_gray, 1.1, 10)
        if len(eyes) >= 2:
            # Sort eyes by x coordinate (left to right)
            eyes = sorted(eyes, key=lambda e: e[0])
            ex, ey, ew, eh = eyes[0]
            ex2, ey2, ew2, eh2 = eyes[1]

            # Ensure left eye < right eye
            if ex > ex2:
                ex, ex2 = ex2, ex

            # Compute sunglasses size
            sung_w = max(1, ex2 + ew2 - ex)
            sung_h = max(1, int(sung_w * sunglasses.shape[0] / sunglasses.shape[1]))
            y1 = min(ey, ey2)
            y2 = y1 + sung_h
            x1 = ex
            x2 = ex + sung_w

            # Prevent overlay going outside the face rectangle
            if y2 > h:
                y2 = h
            if x2 > w:
                x2 = w

            # Resize sunglasses
            sung_resized = cv2.resize(sunglasses, (x2-x1, y2-y1))

            # Overlay with alpha channel if available
            if sung_resized.shape[2] == 4:
                for c in range(0,3):
                    alpha = sung_resized[:, :, 3] / 255.0
                    roi_color[y1:y2, x1:x2, c] = alpha * sung_resized[:, :, c] + \
                                                 (1 - alpha) * roi_color[y1:y2, x1:x2, c]
            else:
                roi_color[y1:y2, x1:x2] = sung_resized

    cv2.imshow('Sunglasses Filter', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
