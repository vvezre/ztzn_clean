import cv2
import numpy as np

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, 50, 150)

    lines = cv2.HoughLinesP(edges, 1, np.pi/180, 100, minLineLength=100, maxLineGap=10)

    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            angle = np.degrees(np.arctan2(y2 - y1, x2 - x1))
            angle_from_vertical = abs(90 - abs(angle))
            angle_from_vertical = min(angle_from_vertical, 90)

            cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            mid_x, mid_y = (x1 + x2) // 2, (y1 + y2) // 2
            cv2.putText(frame, "{:.1f}".format(angle_from_vertical),(mid_x, mid_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    cv2.imshow('Frame', frame)
    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()