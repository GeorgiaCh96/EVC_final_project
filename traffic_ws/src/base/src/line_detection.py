import cv2
import numpy as np


cam=cv2.VideoCapture(0)

# Get the default frame width and height
width = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))
print("frame dims: ", width, height)

# Define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
#out = cv2.VideoWriter('output.mp4', fourcc, 20.0, (width, height))

while True:
    ret, frame = cam.read()

    # Write the frame to the output file
    #out.write(frame)

    # Display the captured frame
    #cv2.imshow('Camera', frame)

    crop_start = int(height * 0.6)

    # Crop bottom portion and process
    crop = frame[crop_start:, :]
    #crop = frame[:, :]

    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray, 120, 255, cv2.THRESH_BINARY_INV)
    #thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 81,50)
    #thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, \
    #                           cv2.THRESH_BINARY, 15, 20)

    M = cv2.moments(thresh)
    annotated = frame.copy()
    cv2.imshow('threshold image', thresh)

    if M["m00"] > 0:
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"]) + crop_start

        image_center = width // 2
        error = cx - image_center
        direction = "CENTERED"
        if abs(error) > 15:
            direction = "LEFT" if error < 0 else "RIGHT"

        # Visual annotations
        cv2.circle(annotated, (cx, cy), 10, (0, 0, 255), -1)
        cv2.line(annotated, (image_center, crop_start), (image_center, height), (255, 255, 0), 2)
        cv2.line(annotated, (cx, crop_start), (cx, height), (0, 255, 0), 2)
        cv2.putText(annotated, f"Offset: {error} px ({direction})", (10, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
        cv2.imshow('Threshold image', annotated)
    else:
        cv2.putText(annotated, "No line detected", (10, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

    # Add threshold preview inset
    thresh_bgr = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)
    inset = cv2.resize(thresh_bgr, (160, 120))
    annotated[10:130, -170:-10] = inset

    # Press 'q' to exit the loop
    if cv2.waitKey(1) == ord('q'):
        break



# Release the capture and writer objects
cam.release()
#out.release()
cv2.destroyAllWindows()