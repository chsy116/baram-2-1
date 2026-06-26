#
import cv2
import numpy as np
import serial
import time
import re

#
#
#

SERIAL_PORT = "COM4"
BAUDRATE = 115200

CAMERA_INDEX = 1
CAM_WIDTH = 1280
CAM_HEIGHT = 720

SEND_INTERVAL = 0.03
MIN_AREA = 300
MAX_POSITION_JUMP = 350
FILTER_ALPHA = 0.55
FILTER_FAST_ALPHA = 0.90
FILTER_SNAP_DISTANCE = 80.0
TARGET_ZONE_RADIUS = 60
TARGET_ZONE_ALPHA = 0.25
DEFAULT_TARGET_X = 688
DEFAULT_TARGET_Y = 388
CONTROL_RESET_SECONDS = 0.35

# Camera-to-platform calibration from measured platform corner pixels.
# ID1: screen left, ID3: screen top, ID5: screen lower-right.
USE_PLATFORM_COORDS = True
PLATFORM_CONTROL_RADIUS = 300.0
PLATFORM_SCREEN_ID1 = np.float32([348.0, 340.0])
PLATFORM_SCREEN_ID3 = np.float32([805.0, 67.0])
PLATFORM_SCREEN_ID5 = np.float32([809.0, 588.0])
PLATFORM_ID1 = np.float32([-0.8660254 * PLATFORM_CONTROL_RADIUS, 0.5 * PLATFORM_CONTROL_RADIUS])
PLATFORM_ID3 = np.float32([0.0, -PLATFORM_CONTROL_RADIUS])
PLATFORM_ID5 = np.float32([0.8660254 * PLATFORM_CONTROL_RADIUS, 0.5 * PLATFORM_CONTROL_RADIUS])
PLATFORM_AFFINE = cv2.getAffineTransform(
    np.float32([PLATFORM_SCREEN_ID1, PLATFORM_SCREEN_ID3, PLATFORM_SCREEN_ID5]),
    np.float32([PLATFORM_ID1, PLATFORM_ID3, PLATFORM_ID5])
)
#
USE_CIRCULAR_FILTER = True
MIN_CIRCULARITY = 0.6

#
SHOW_DEBUG = True
USE_RED_BALL_HSV = True


#
#
#

origin_x = None
origin_y = None
last_cx = None
last_cy = None
filtered_cx = None
filtered_cy = None
control_reset_until = 0.0


#
#
#

def nothing(x):
    pass


def set_origin(event, x, y, flags, param):
    global origin_x, origin_y, filtered_cx, filtered_cy, control_reset_until

    if event == cv2.EVENT_LBUTTONDOWN:
        origin_x = x
        origin_y = y
        filtered_cx = None
        filtered_cy = None
        control_reset_until = time.time() + CONTROL_RESET_SECONDS
        print(f"[ORIGIN SET] x={origin_x}, y={origin_y}")


def open_stm():
    try:
        stm = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.01)
        time.sleep(2)
        print(f"[OK] STM connected: {SERIAL_PORT}, {BAUDRATE} bps")
        return stm
    except serial.SerialException:
        print("[WARN] STM connection failed")
        print("       Check the COM port")
        print("       Running OpenCV view only")
        return None


def open_camera():
    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)

    if not cap.isOpened():
        print("[ERROR] Camera open failed")
        print("        Change CAMERA_INDEX to 0, 1, or 2 and retry")
        exit()

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)

    return cap


#
#
#

stm = open_stm()


#
#
#

cap = open_camera()


#
#
#

cv2.namedWindow("Camera", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Camera", 1280, 720)
cv2.setMouseCallback("Camera", set_origin)

cv2.namedWindow("HSV Trackbar", cv2.WINDOW_NORMAL)
cv2.resizeWindow("HSV Trackbar", 400, 300)

#
cv2.createTrackbar("H_min", "HSV Trackbar", 0, 179, nothing)
cv2.createTrackbar("H_max", "HSV Trackbar", 10, 179, nothing)
cv2.createTrackbar("S_min", "HSV Trackbar", 100, 255, nothing)
cv2.createTrackbar("S_max", "HSV Trackbar", 255, 255, nothing)
cv2.createTrackbar("V_min", "HSV Trackbar", 80, 255, nothing)
cv2.createTrackbar("V_max", "HSV Trackbar", 255, 255, nothing)

if SHOW_DEBUG:
    cv2.namedWindow("Mask", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Mask", 640, 360)

kernel = np.ones((5, 5), np.uint8)
last_send_time = 0
last_pc_print_time = 0
last_stm_response_time = time.time()
stm_control = {
    "state": 0,
    "speed": 0,
    "approach": 0,
    "p_scale_percent": 100,
    "p_x10": 0,
    "p_y10": 0,
    "d_x10": 0,
    "d_y10": 0,
    "cmd_x10": 0,
    "cmd_y10": 0,
    "limit_x10": 0,
    "saturated": 0,
    "kick": 0,
}
stm_control_pattern = re.compile(
    r"CTRL e=-?\d+,-?\d+ st=(\d+) v=(-?\d+) ap=(-?\d+) ps=(\d+) "
    r"P=(-?\d+),(-?\d+) D=(-?\d+),(-?\d+) "
    r"C=(-?\d+),(-?\d+) lim=(\d+) sat=(\d+) kick=(\d+)"
)


#
#
#

while True:
    ret, frame = cap.read()

    if not ret:
        print("[ERROR] Frame read failed")
        break

#
    frame = cv2.flip(frame, 1)

    frame_h = frame.shape[0]

#
#
#
#

    if origin_x is None or origin_y is None:
        target_x = DEFAULT_TARGET_X
        target_y = DEFAULT_TARGET_Y
        origin_mode = "CENTER"
    else:
        target_x = origin_x
        target_y = origin_y
        origin_mode = "MOUSE"

#
    ball_detected = 0
    cx = 0
    cy = 0
    ex = 0
    ey = 0
    control_ex = 0
    control_ey = 0
    display = frame.copy()

#
#
#

    blur = cv2.GaussianBlur(frame, (9, 9), 0)
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

    h_min = cv2.getTrackbarPos("H_min", "HSV Trackbar")
    h_max = cv2.getTrackbarPos("H_max", "HSV Trackbar")
    s_min = cv2.getTrackbarPos("S_min", "HSV Trackbar")
    s_max = cv2.getTrackbarPos("S_max", "HSV Trackbar")
    v_min = cv2.getTrackbarPos("V_min", "HSV Trackbar")
    v_max = cv2.getTrackbarPos("V_max", "HSV Trackbar")

    lower_ball = np.array([h_min, s_min, v_min], dtype=np.uint8)
    upper_ball = np.array([h_max, s_max, v_max], dtype=np.uint8)

    mask = cv2.inRange(hsv, lower_ball, upper_ball)

    if USE_RED_BALL_HSV:
        lower_red_high = np.array([170, s_min, v_min], dtype=np.uint8)
        upper_red_high = np.array([179, s_max, v_max], dtype=np.uint8)
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower_red_high, upper_red_high))

#
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best_contour = None
    best_score = -1

    for cnt in contours:
        area = cv2.contourArea(cnt)

        if area < MIN_AREA:
            continue

        perimeter = cv2.arcLength(cnt, True)

        if perimeter == 0:
            continue

        circularity = 4 * np.pi * area / (perimeter * perimeter)

        _, _, w_rect, h_rect = cv2.boundingRect(cnt)
        aspect_ratio = w_rect / h_rect if h_rect != 0 else 0

        if USE_CIRCULAR_FILTER:
            if circularity < MIN_CIRCULARITY:
                continue

            if not (0.7 < aspect_ratio < 1.3):
                continue

        score = area * circularity

        if score > best_score:
            best_score = score
            best_contour = cnt

#
#
#

    if best_contour is not None and last_cx is not None and last_cy is not None:
        M_check = cv2.moments(best_contour)
        if M_check["m00"] != 0:
            check_cx = int(M_check["m10"] / M_check["m00"])
            check_cy = int(M_check["m01"] / M_check["m00"])
            jump = np.hypot(check_cx - last_cx, check_cy - last_cy)
            if jump > MAX_POSITION_JUMP:
                print(f"[WARN] ignored position jump: {jump:.1f}px")
                best_contour = None

    if best_contour is not None:
        M = cv2.moments(best_contour)

        if M["m00"] != 0:
            ball_detected = 1

            raw_cx = int(M["m10"] / M["m00"])
            raw_cy = int(M["m01"] / M["m00"])

            last_cx = raw_cx
            last_cy = raw_cy

            if filtered_cx is None or filtered_cy is None:
                filtered_cx = float(raw_cx)
                filtered_cy = float(raw_cy)
            else:
                lag = np.hypot(raw_cx - filtered_cx, raw_cy - filtered_cy)
                alpha = FILTER_FAST_ALPHA if lag > FILTER_SNAP_DISTANCE else FILTER_ALPHA
                filtered_cx = filtered_cx + alpha * (raw_cx - filtered_cx)
                filtered_cy = filtered_cy + alpha * (raw_cy - filtered_cy)

            cx = int(round(filtered_cx))
            cy = int(round(filtered_cy))
            ex = cx - target_x
            ey = cy - target_y

            if USE_PLATFORM_COORDS:
                ball_platform = cv2.transform(np.array([[[float(cx), float(cy)]]], dtype=np.float32), PLATFORM_AFFINE)[0][0]
                target_platform = cv2.transform(np.array([[[float(target_x), float(target_y)]]], dtype=np.float32), PLATFORM_AFFINE)[0][0]
                platform_ex = int(round(ball_platform[0] - target_platform[0]))
                platform_ey = int(round(ball_platform[1] - target_platform[1]))
            else:
                platform_ex = ex
                platform_ey = ey

            control_ex = platform_ex
            control_ey = platform_ey

            target_distance = np.hypot(ex, ey)
            area = cv2.contourArea(best_contour)
            perimeter = cv2.arcLength(best_contour, True)
            circularity = 4 * np.pi * area / (perimeter * perimeter) if perimeter > 0 else 0

            (x, y), radius = cv2.minEnclosingCircle(best_contour)

            cv2.drawContours(display, [best_contour], -1, (0, 255, 0), 2)
            cv2.circle(display, (int(x), int(y)), int(radius), (255, 0, 255), 2)
            cv2.circle(display, (cx, cy), 6, (0, 0, 255), -1)
            cv2.line(display, (target_x, target_y), (cx, cy), (0, 255, 255), 1)

            zone_status = "IN" if target_distance <= TARGET_ZONE_RADIUS else "OUT"
            cv2.putText(display,
                        f"BALL ({cx},{cy})  TARGET ({target_x},{target_y})  ZONE {zone_status}",
                        (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            cv2.putText(display,
                        f"ERROR ({ex},{ey})  CTRL ({control_ex},{control_ey})  DIST {target_distance:.1f}",
                        (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            state_names = ("FAR", "NEAR", "BRAKE", "CAP-BRAKE", "HOLD")
            state_index = min(stm_control["state"], len(state_names) - 1)
            control_color = (0, 0, 255) if stm_control["saturated"] else (255, 255, 0)
            cv2.putText(
                display,
                f"STM {state_names[state_index]}  V {stm_control['speed']} AP {stm_control['approach']}  "
                f"P% {stm_control['p_scale_percent']}  "
                f"P ({stm_control['p_x10'] / 10:.1f},{stm_control['p_y10'] / 10:.1f})  "
                f"D ({stm_control['d_x10'] / 10:.1f},{stm_control['d_y10'] / 10:.1f})  "
                f"CMD ({stm_control['cmd_x10'] / 10:.1f},{stm_control['cmd_y10'] / 10:.1f})  "
                f"LIM {stm_control['limit_x10'] / 10:.1f}  "
                f"SAT {stm_control['saturated']} KICK {stm_control['kick']}",
                (10, 90),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.62,
                control_color,
                2,
            )

    else:
        last_cx = None
        last_cy = None
        filtered_cx = None
        filtered_cy = None
        cv2.putText(display, "Red ball not found", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

#
#
#

    zone_color = (0, 200, 0) if ball_detected == 1 and np.hypot(ex, ey) <= TARGET_ZONE_RADIUS else (255, 0, 0)
    zone_overlay = display.copy()
    cv2.circle(zone_overlay, (target_x, target_y), TARGET_ZONE_RADIUS, zone_color, -1)
    display = cv2.addWeighted(zone_overlay, TARGET_ZONE_ALPHA, display, 1.0 - TARGET_ZONE_ALPHA, 0)
    cv2.circle(display, (target_x, target_y), TARGET_ZONE_RADIUS, zone_color, 2)
    cv2.circle(display, (target_x, target_y), 4, zone_color, -1)

    cv2.putText(display, f"TARGET R={TARGET_ZONE_RADIUS} [{origin_mode}]",
                (10, frame_h - 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

#
#
#

    current_time = time.time()

    if current_time - last_send_time >= SEND_INTERVAL:
        if current_time < control_reset_until:
            send_data = "0,0,0\n"
        elif ball_detected == 1:
            send_data = f"{control_ex},{control_ey},1\n"
        else:
            send_data = "0,0,0\n"

        if stm is not None:
            try:
                stm.write(send_data.encode())

#
                time.sleep(0.005)

                while stm.in_waiting > 0:
                    response = stm.readline().decode(errors="ignore").strip()
                    if response:
                        last_stm_response_time = current_time
                        match = stm_control_pattern.search(response)
                        if match:
                            values = [int(value) for value in match.groups()]
                            (
                                stm_control["state"],
                                stm_control["speed"],
                                stm_control["approach"],
                                stm_control["p_scale_percent"],
                                stm_control["p_x10"],
                                stm_control["p_y10"],
                                stm_control["d_x10"],
                                stm_control["d_y10"],
                                stm_control["cmd_x10"],
                                stm_control["cmd_y10"],
                                stm_control["limit_x10"],
                                stm_control["saturated"],
                                stm_control["kick"],
                            ) = values
                        print("STM:", response)
            except serial.SerialException as exc:
                print(f"[WARN] STM disconnected: {exc}")
                try:
                    stm.close()
                except serial.SerialException:
                    pass
                stm = None

        if current_time - last_pc_print_time >= 0.5:
            print("PC SEND:", send_data.strip())
            if stm is not None and current_time - last_stm_response_time > 1.5:
                print("[WARN] No STM log received yet. Check CubeIDE Run/flash, USART2 wiring, and 12V/DXL power.")
            last_pc_print_time = current_time

        last_send_time = current_time

#
#
#

    cv2.imshow("Camera", display)

    if SHOW_DEBUG:
        cv2.imshow("Mask", mask)

    key = cv2.waitKey(1) & 0xFF

#
    if key == ord('q') or key == 27:
        break

#
    elif key == ord('r'):
        origin_x = None
        origin_y = None
        control_reset_until = time.time() + CONTROL_RESET_SECONDS
        print("[ORIGIN RESET] center")

#
    elif key == ord('p'):
        print("Current HSV values")
        print(f"H_MIN = {h_min}")
        print(f"H_MAX = {h_max}")
        print(f"S_MIN = {s_min}")
        print(f"S_MAX = {s_max}")
        print(f"V_MIN = {v_min}")
        print(f"V_MAX = {v_max}")

#
    elif key == ord('s'):
        cv2.imwrite("capture_camera.png", display)
        cv2.imwrite("capture_mask.png", mask)
        print("[SAVE] capture_camera.png, capture_mask.png")


#
#
#

cap.release()
cv2.destroyAllWindows()

if stm is not None:
    stm.close()
















