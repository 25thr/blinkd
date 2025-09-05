import cv2
import numpy as np
import mediapipe as mp
import time

def compute_ear(eye):
    A = np.linalg.norm(eye[1] - eye[5])
    B = np.linalg.norm(eye[2] - eye[4])
    C = np.linalg.norm(eye[0] - eye[3])
    return (A + B) / (2.0 * C)

# MediaPipe landmark indices
LEFT_EYE_IDX = [362, 385, 387, 263, 373, 380]
RIGHT_EYE_IDX = [33, 160, 158, 133, 153, 144]

# EAR normalization range (calibrate for real use)
EAR_MIN = 0.15
EAR_MAX = 0.30

def normalize_ear(ear):
    return np.clip((ear - EAR_MIN) / (EAR_MAX - EAR_MIN), 0, 1)

# Init
mp_face_mesh = mp.solutions.face_mesh
face_mesh = mp_face_mesh.FaceMesh(static_image_mode=False, max_num_faces=1, refine_landmarks=True)

cap = cv2.VideoCapture(0)

start_time = time.time()

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    h, w = frame.shape[:2]
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = face_mesh.process(rgb)

    if results.multi_face_landmarks:
        face = results.multi_face_landmarks[0]
        landmarks = np.array([(lm.x * w, lm.y * h) for lm in face.landmark])

        left_eye = landmarks[LEFT_EYE_IDX]
        right_eye = landmarks[RIGHT_EYE_IDX]

        left_ear = compute_ear(left_eye)
        right_ear = compute_ear(right_eye)

        openL = normalize_ear(left_ear)
        openR = normalize_ear(right_ear)

        t_ms = int((time.time() - start_time) * 1000)

        # Output in format: t_ms openL openR
        print(f"{t_ms} {openL:.3f} {openR:.3f}", flush=True)

cap.release()
