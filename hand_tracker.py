import math                          # angle math (acos, degrees)
import socket                        # UDP (sends data to Blender)
import time                          # clock for the READY wait

import cv2                           # OpenCV: camera + drawing on the video window
import mediapipe as mp               # Google's hand-landmark detector (21 points per hand)
import numpy as np                   # vector math
import serial                        # PySerial: talks to the Nano
from serial.tools import list_ports  # lets us search for the Nano's USB port automatically

# ================= SETTINGS (edit these) =================
CAMERA_INDEX = 0                     # 0 = MacBook camera. If you see your iPhone instead, try 1
SERIAL_PORT = "auto"                 # "auto" = find the Nano by itself, or type e.g. "/dev/cu.usbserial-1410"
                                     # (Wokwi simulator: "rfc2217://localhost:4000")
BAUD_RATE = 115200                   # must match Serial.begin() in the Arduino code
READY_TIMEOUT = 8                    # seconds to wait for the Nano to finish booting and say READY
UDP_ADDR = ("127.0.0.1", 5005)       # Blender listens here (harmless if Blender is closed)
SMOOTHING_ALPHA = 0.35               # 0..1: smaller = smoother but laggier, larger = snappier but jittery

# For each finger: (joint A, joint B, joint C, angle when straight, angle when fully curled)
# We measure the angle AT joint B between segments B->A and B->C.
# MediaPipe landmark ids: thumb 2,3,4 | index 5,6,8 | middle 9,10,12 | ring 13,14,16 | pinky 17,18,20
# CALIBRATION: watch the "raw" value on screen with your finger straight, then curled, and edit the last two numbers.
FINGERS = {
    "thumb":  (2, 3, 4,    170, 120),
    "index":  (5, 6, 8,    170, 60),
    "middle": (9, 10, 12,  170, 60),
    "ring":   (13, 14, 16, 170, 60),
    "pinky":  (17, 18, 20, 170, 60),
}

# ================= HELPER FUNCTIONS =================
def joint_angle(a, b, c):
    """Return the angle in degrees at point b, formed by points a-b-c."""
    ba = a - b                                                        # vector from b to a
    bc = c - b                                                        # vector from b to c
    cos_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc) + 1e-9)  # 1e-9 avoids divide-by-zero
    return math.degrees(math.acos(np.clip(cos_angle, -1.0, 1.0)))    # clip keeps acos input legal

def find_nano_port():
    """Look through the Mac's serial ports for a USB one (the Nano shows up as /dev/cu.usb...)."""
    for p in list_ports.comports():
        if p.device.startswith("/dev/cu.") and "usb" in p.device.lower():
            return p.device
    return None                                                       # nothing found

def wait_for_ready(port_obj, seconds):
    """The Nano reboots when the port opens, runs its self-test, then prints READY. Wait for that word."""
    port_obj.timeout = 0.2                                            # readline() gives up after 0.2 s so we can check the clock
    deadline = time.time() + seconds
    while time.time() < deadline:
        line = port_obj.readline().decode(errors="ignore").strip()    # read one line of text from the Nano
        if line == "READY":
            return True
    return False

# ================= CONNECTIONS =================
port = find_nano_port() if SERIAL_PORT == "auto" else SERIAL_PORT     # decide which port to use
ser = None
if port:
    try:
        # serial_for_url accepts both real port names and network URLs like rfc2217://...
        ser = serial.serial_for_url(port, baudrate=BAUD_RATE, timeout=0.2)
        print(f"Serial connected: {port}. Waiting for the Nano to boot...")
        if wait_for_ready(ser, READY_TIMEOUT):
            print("Nano is READY.")
        else:
            print("Did not see READY (normal for Wokwi). Continuing anyway.")
        ser.reset_input_buffer()                                      # throw away anything unread
    except Exception as err:
        print(f"Serial NOT connected ({err}). Close Serial Monitor if it is open. Continuing with UDP only.")
        ser = None
else:
    print("No Nano found. Plug it in via USB (check: ls /dev/cu.*). Continuing with UDP only.")

udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)                # UDP socket: fire-and-forget messages to Blender

# ================= MEDIAPIPE + CAMERA =================
mp_hands = mp.solutions.hands
mp_draw = mp.solutions.drawing_utils
cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_AVFOUNDATION)            # AVFoundation = macOS's native camera backend
smoothed = [0.0] * 5                                                  # last servo angle per finger, for smoothing

with mp_hands.Hands(max_num_hands=1,                                  # track only one hand
                    min_detection_confidence=0.6,
                    min_tracking_confidence=0.6) as hands:
    while cap.isOpened():
        ok, frame = cap.read()                                        # grab one camera frame
        if not ok:
            print("Camera not working. Check System Settings > Privacy & Security > Camera.")
            break
        frame = cv2.flip(frame, 1)                                    # mirror image so it feels natural
        h, w = frame.shape[:2]                                        # frame height/width in pixels
        result = hands.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))  # MediaPipe wants RGB, OpenCV gives BGR

        if result.multi_hand_landmarks:                               # only act if a hand was found
            hand = result.multi_hand_landmarks[0]
            mp_draw.draw_landmarks(frame, hand, mp_hands.HAND_CONNECTIONS)  # draw skeleton on the video

            # Convert normalised 0..1 landmarks to pixels so angles aren't distorted by the image aspect ratio
            pts = np.array([[lm.x * w, lm.y * h, lm.z * w] for lm in hand.landmark])

            for i, (name, (a, b, c, straight, curled)) in enumerate(FINGERS.items()):
                angle = joint_angle(pts[a], pts[b], pts[c])           # raw joint angle (big = straight, small = curled)
                t = (straight - angle) / (straight - curled)          # 0.0 = straight ... 1.0 = curled
                t = min(max(t, 0.0), 1.0)                             # clamp to 0..1
                target = t * 180.0                                    # map flexion to 0-180 (0 = open, 180 = fist)
                smoothed[i] += SMOOTHING_ALPHA * (target - smoothed[i])   # move part of the way toward the target
                cv2.putText(frame, f"{name}: raw {angle:3.0f} -> servo {smoothed[i]:3.0f}",
                            (10, 25 + 25 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            servo_angles = [int(round(s)) for s in smoothed]          # servos want whole numbers
            message = ",".join(map(str, servo_angles)) + "\n"         # e.g. "0,90,180,45,10\n" (thumb..pinky)

            if ser:                                                   # send to the Nano
                try:
                    ser.write(message.encode())
                except Exception as err:
                    print("Serial write failed:", err)
                    ser = None                                        # stop trying; keep running with UDP
            udp.sendto(message.encode(), UDP_ADDR)                    # send to Blender

        cv2.imshow("Hand Tracker (press q to quit)", frame)           # show the video window
        if cv2.waitKey(1) & 0xFF == ord("q"):                         # press q to stop
            break

# ================= CLEAN UP =================
cap.release()
cv2.destroyAllWindows()
if ser:
    ser.close()