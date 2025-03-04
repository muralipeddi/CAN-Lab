import cv2
import apriltag
import numpy as np

# Camera matrix and distortion coefficients
camera_matrix = np.array([[960.1350599799081, 0, 949.7145687302444],
                          [0, 936.2885579258001, 549.0063604187408],
                          [0, 0, 1]])
dist_coeffs = np.array([0.04220757, -0.03474944,  0.00216912,  0.0005294,  -0.03428984])  

options = apriltag.DetectorOptions(families="tag36h11")
detector = apriltag.Detector(options)

tag_size = 0.073  # Size of the AprilTag in meters
box_height = 0.05  # Height of the 3D box above the tag

# Video capture (replace 'demo.mp4' with your video file)
cap = cv2.VideoCapture(0)

while True:

    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    detections = detector.detect(gray)

    for detection in detections:
        # 3D object points for solvePnP (we are assuming the tag is on a flat plane)
        obj_points = np.array([[-tag_size / 2, -tag_size / 2, 0],
                               [tag_size / 2, -tag_size / 2, 0],
                               [tag_size / 2, tag_size / 2, 0],
                               [-tag_size / 2, tag_size / 2, 0]], dtype=np.float32)

        # 2D points from the detected corners
        img_points = detection.corners.astype(np.float32)

        # Estimate the pose of the tag
        retval, rvec, tvec = cv2.solvePnP(obj_points, img_points, camera_matrix, dist_coeffs)

        # Get the translation vector (position of the tag)
        x, y, z = tvec.ravel()

        # Display the x, y, z position on the window
        position_text = f"Position: x={x:.2f} m, y={y:.2f} m, z={z:.2f} m"
        cv2.putText(frame, position_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # Define the 3D box points relative to the tag's position
        box_points = np.array([
            [-tag_size / 2, -tag_size / 2, 0],  # Bottom corners (same as tag corners)
            [tag_size / 2, -tag_size / 2, 0],
            [tag_size / 2, tag_size / 2, 0],
            [-tag_size / 2, tag_size / 2, 0],
            [-tag_size / 2, -tag_size / 2, -box_height],  # Top corners (above the tag)
            [tag_size / 2, -tag_size / 2, -box_height],
            [tag_size / 2, tag_size / 2, -box_height],
            [-tag_size / 2, tag_size / 2, -box_height]
        ], dtype=np.float32)

        img_box_points, _ = cv2.projectPoints(box_points, rvec, tvec, camera_matrix, dist_coeffs)

        img_box_points = img_box_points.astype(int).reshape(-1, 2)

        color = (0, 255, 0)  # Set box color to green
        cv2.polylines(frame, [img_box_points[:4]], isClosed=True, color=color, thickness=2)  # Bottom face
        cv2.polylines(frame, [img_box_points[4:]], isClosed=True, color=color, thickness=2)  # Top face
        for i in range(4):
            cv2.line(frame, tuple(img_box_points[i]), tuple(img_box_points[i + 4]), color, 2)  # Vertical edges
            
        center_x = int(np.mean(img_points[:, 0]))
        center_y = int(np.mean(img_points[:, 1]))
        cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)


    cv2.imshow("AprilTag Detection", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

