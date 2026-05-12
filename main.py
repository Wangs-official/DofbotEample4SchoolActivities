import time
import ctypes
import inspect
import threading
import os
import random
import signal
import atexit

import numpy as np
import cv2
import tkinter as tk
from PIL import Image, ImageTk

import Arm_Lib
from Arm_Lib import Arm_Device
from Speech_Lib import Speech
from dofbot_utils.robot_controller import Robot_Controller
from dofbot_utils.dofbot_config import read_HSV, write_HSV
from follow.color_follow import color_follow


def cleanup_on_exit():
    try:
        if os.path.exists("1"):
            os.remove("1")
    except:
        pass


atexit.register(cleanup_on_exit)


def signal_handler(sig, frame):
    cleanup_on_exit()
    os._exit(0)


signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

print("僕のため それだけ")
print("出口探し 溢れただけの言葉")
print("......")
print("正在启动UI，请确认X11已转发")

Arm = Arm_Device()
time.sleep(0.1)

mySpeech = Speech()
robot = Robot_Controller()

robot.move_init_pose()
time.sleep(1)

look_at = robot.P_LOOK_AT
p_top = robot.P_TOP
p_Brown = robot.P_CENTER

p_Yellow = robot.P_YELLOW
p_Red = robot.P_RED
p_Green = robot.P_GREEN
p_Blue = robot.P_BLUE

p_layer_1 = robot.P_CENTER_HEAP_L1
p_layer_2 = robot.P_CENTER_HEAP_L2
p_layer_3 = robot.P_CENTER_HEAP_L3
p_layer_4 = robot.P_CENTER_HEAP_L4

p_move_layer_4 = robot.P_CENTER_4
p_move_layer_3 = robot.P_CENTER_3
p_move_layer_2 = robot.P_CENTER_2
p_move_layer_1 = robot.P_CENTER

cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
cap.set(3, 640)
cap.set(4, 480)
cap.set(5, 30)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))

follow_ctrl = color_follow()
HSV_learning = ((0, 240, 54), (8, 255, 255))

HSV_path = "./HSV_config.txt"

default_color_hsv = {
    "red": ((0, 96, 83), (10, 255, 255)),
    "green": ((38, 71, 90), (86, 156, 217)),
    "blue": ((78, 132, 48), (120, 186, 255)),
    "yellow": ((21, 98, 46), (34, 255, 255)),
}

color_hsv = {}
try:
    color_hsv = default_color_hsv.copy()
    read_HSV(HSV_path, color_hsv)
    print(f"[系统] 已从 {HSV_path} 加载HSV配置")
except Exception as e:
    print(f"[系统] 读取HSV配置失败: {e}")
    print("[系统] 使用默认HSV值")
    color_hsv = default_color_hsv.copy()

rand_colors = [[random.randint(0, 255) for _ in range(3)] for _ in range(255)]

g_mode = 'idle'
g_state_arm = 0
g_current_color = None
g_current_frame = None
g_calib_window_open = False


def _async_raise(tid, exctype):
    tid = ctypes.c_long(tid)
    if not inspect.isclass(exctype):
        exctype = type(exctype)
    res = ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, ctypes.py_object(exctype))
    if res == 0:
        raise ValueError("invalid thread id")
    elif res != 1:
        ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, None)


def stop_thread(thread):
    _async_raise(thread.ident, SystemExit)


def get_color(img):
    H = []
    color_name = {}
    img = cv2.resize(img, (640, 480))
    HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    cv2.rectangle(img, (280, 130), (360, 230), (0, 255, 0), 2)
    for i in range(280, 360):
        for j in range(130, 230):
            H.append(HSV[j, i][0])
    H_min, H_max = min(H), max(H)
    if (H_min >= 0 and H_max <= 20) or (H_min >= 156 and H_max <= 180):
        color_name['name'] = 'red'
    elif H_min >= 21 and H_max <= 28:
        color_name['name'] = 'yellow'
    elif H_min >= 35 and H_max <= 78:
        color_name['name'] = 'green'
    elif H_min >= 100 and H_max <= 124:
        color_name['name'] = 'blue'
    return img, color_name


def arm_move_6(p, s_time=500):
    for i in range(6):
        Arm.Arm_serial_servo_write(i + 1, p[i], s_time)
        time.sleep(0.01)
    time.sleep(s_time / 1000)


def arm_move(p, s_time=500):
    for i in range(5):
        id = i + 1
        if id == 5:
            time.sleep(0.1)
            Arm.Arm_serial_servo_write(id, p[i], int(s_time * 1.2))
        elif id == 1:
            Arm.Arm_serial_servo_write(id, p[i], int(3 * s_time / 4))
        else:
            Arm.Arm_serial_servo_write(id, p[i], int(s_time))
        time.sleep(0.01)
    time.sleep(s_time / 1000)


def arm_move_clamp(p, s_time=500):
    for i in range(5):
        id = i + 1
        if id == 5:
            time.sleep(0.1)
            Arm.Arm_serial_servo_write(id, p[i], int(s_time * 1.2))
        else:
            Arm.Arm_serial_servo_write(id, p[i], s_time)
        time.sleep(0.01)
    time.sleep(s_time / 1000)


def arm_clamp_block(enable):
    Arm.Arm_serial_servo_write(6, 135 if enable else 60, 400)
    time.sleep(0.5)


def arm_move_up():
    Arm.Arm_serial_servo_write(2, 90, 1500)
    Arm.Arm_serial_servo_write(3, 90, 1500)
    Arm.Arm_serial_servo_write(4, 90, 1500)
    time.sleep(0.1)


def put_down_block():
    arm_move(p_Brown, 1000)
    arm_clamp_block(0)
    time.sleep(0.5)
    arm_move_6(look_at, 1500)


def number_action(index):
    color_pos = [p_Yellow, p_Red, p_Green, p_Blue]
    arm_move(p_top, 1000)
    arm_move(color_pos[index - 1], 1500)
    arm_clamp_block(1)
    arm_move(p_top, 1500)


def ctrl_arm_move(index):
    global g_state_arm
    color_info = {1: ("黄色", 66), 2: ("红色", 69), 3: ("绿色", 67), 4: ("蓝色", 68)}
    arm_clamp_block(0)
    name, voice_code = color_info[index]
    log_chat(f"[机械臂] 识别到 {name}，开始抓取", tag="arm")
    mySpeech.void_write(voice_code)
    time.sleep(0.1)
    Arm.Arm_Buzzer_On(1)
    time.sleep(0.5)
    number_action(index)
    put_down_block()
    mySpeech.void_write(65)
    log_chat("[机械臂] 抓取完成", tag="arm")
    time.sleep(0.1)
    g_state_arm = 0


def start_move_arm(index):
    global g_state_arm
    if g_state_arm == 0:
        t = threading.Thread(target=ctrl_arm_move, args=[index], daemon=True)
        t.start()
        g_state_arm = 1


def heap_up():
    arm_clamp_block(0)
    arm_move(look_at, 1000)
    time.sleep(1)
    for src, dst in [(p_Yellow, p_layer_1), (p_Red, p_layer_2),
                     (p_Green, p_layer_3), (p_Blue, p_layer_4)]:
        arm_move(p_top, 1000)
        arm_move(src, 1000)
        arm_clamp_block(1)
        arm_move(p_top, 1000)
        arm_move(dst, 1000)
        arm_clamp_block(0)
        time.sleep(0.1)
        arm_move(look_at, 1100)
        time.sleep(2)


def dance():
    t1, ts = 500, 0.5
    Arm.Arm_serial_servo_write6(90, 90, 90, 90, 90, 90, 500)
    time.sleep(1)
    for a2, a3, a4 in [(60, 120, 60), (45, 135, 45), (60, 120, 60), (90, 90, 90),
                       (80, 80, 80), (60, 60, 60), (45, 45, 45), (90, 90, 90)]:
        Arm.Arm_serial_servo_write(2, 180 - a2, t1)
        time.sleep(0.001)
        Arm.Arm_serial_servo_write(3, a3, t1)
        time.sleep(0.001)
        Arm.Arm_serial_servo_write(4, a4, t1)
        time.sleep(ts)
    for a4, a6 in [(20, 150), (90, 90), (20, 150), (90, 90)]:
        Arm.Arm_serial_servo_write(4, a4, t1)
        time.sleep(0.001)
        Arm.Arm_serial_servo_write(6, a6, t1)
        time.sleep(ts)
    Arm.Arm_serial_servo_write(1, 0, t1)
    time.sleep(0.001)
    Arm.Arm_serial_servo_write(5, 0, t1)
    time.sleep(ts)
    Arm.Arm_serial_servo_write(3, 180, t1)
    time.sleep(0.001)
    Arm.Arm_serial_servo_write(4, 0, t1)
    time.sleep(ts)
    Arm.Arm_serial_servo_write(6, 180, t1)
    time.sleep(ts)
    Arm.Arm_serial_servo_write(6, 0, 1000)
    time.sleep(ts)
    Arm.Arm_serial_servo_write(6, 90, 1000)
    time.sleep(0.001)
    Arm.Arm_serial_servo_write(1, 90, t1)
    time.sleep(0.001)
    Arm.Arm_serial_servo_write(5, 90, t1)
    time.sleep(ts)
    Arm.Arm_serial_servo_write(3, 90, t1)
    time.sleep(0.001)
    Arm.Arm_serial_servo_write(4, 90, t1)
    time.sleep(ts)


def clamp_clock():
    arm_clamp_block(0)
    arm_move_clamp(look_at, 1000)
    time.sleep(1)
    for dst in [p_Yellow, p_Red, p_Green, p_Blue]:
        arm_move_clamp(p_top, 1000)
        arm_move_clamp(p_Brown, 1000)
        arm_clamp_block(1)
        arm_move_clamp(p_top, 1000)
        arm_move_clamp(dst, 1000)
        arm_clamp_block(0)
        arm_move_up()
        arm_move_clamp(look_at, 1100)
        time.sleep(2)


def move_block():
    arm_clamp_block(0)
    arm_move_clamp(look_at, 1000)
    time.sleep(1)
    for src, dst in [(p_move_layer_4, p_Yellow), (p_move_layer_3, p_Red),
                     (p_move_layer_2, p_Green), (p_move_layer_1, p_Blue)]:
        arm_move_clamp(p_top, 1000)
        arm_move_clamp(src, 1000)
        arm_clamp_block(1)
        arm_move_clamp(p_top, 1000)
        arm_move_clamp(dst, 1000)
        arm_clamp_block(0)
        time.sleep(0.1)
        arm_move_up()
        arm_move_clamp(look_at, 1100)
        time.sleep(2)


def do_basic_action(result):
    s = 500
    if result == 45:
        mySpeech.void_write(45)
        for _ in range(2):
            Arm.Arm_serial_servo_write(6, 180, s)
            time.sleep(s / 1000)
            Arm.Arm_serial_servo_write(6, 90, s)
            time.sleep(s / 1000)
    elif result == 46:
        mySpeech.void_write(45)
        s = 800
        for _ in range(2):
            Arm.Arm_serial_servo_write(4, 30, s)
            time.sleep(s / 1000)
            Arm.Arm_serial_servo_write(4, 0, s)
            time.sleep(s / 1000)
    elif result == 47:
        mySpeech.void_write(45)
        Arm.Arm_serial_servo_write6(90, 90, 0, 180, 90, 180, 1000)
        time.sleep(1.5)
        Arm.Arm_serial_servo_write6(90, 164, 18, 0, 90, 90, 1000)
        time.sleep(1)
    elif result == 48:
        mySpeech.void_write(45)
        Arm.Arm_serial_servo_write6(90, 90, 90, 90, 90, 90, 800)
        time.sleep(0.1)
        Arm.Arm_serial_servo_write6(90, 0, 180, 0, 90, 30, 1000)
        time.sleep(1)
    elif result == 49:
        mySpeech.void_write(45)
        Arm.Arm_serial_servo_write6(90, 90, 90, 90, 90, 90, 800)
        time.sleep(1)
    elif result == 50:
        mySpeech.void_write(45)
        s = 800
        for angle in [0, 180, 0, 180, 90]:
            Arm.Arm_serial_servo_write6(angle, 90, 90, 90, 90, 90, s)
            time.sleep(s / 1000)


RESULT_NAME = {
    45: "鼓掌", 46: "点头", 47: "招手",
    48: "弯腰", 49: "复位", 50: "左右摇摆",
    51: "叠罗汉", 52: "跳舞", 53: "夹方块", 54: "搬运",
    60: "问色",
}

BG = "#f0f0f0"
PANEL_BG = "#ffffff"
BORDER = "#c0c0c0"
BTN_INACTIVE = "#404040"
BTN_ACTIVE = "#228B22"
BTN_TRACKING = "#D4A017"
FG = "#000000"
FG_DIM = "#666666"
ACCENT_RED = "#FF0000"
ACCENT_GREEN = "#228B22"
ACCENT_BLUE = "#0000FF"
ACCENT_YELLOW = "#FFC000"

root = tk.Tk()
root.title("记得看看右侧的说明 OvO")
root.configure(bg=BG)
root.resizable(True, True)

root.attributes('-zoomed', True)

FONT_TITLE = ("Microsoft YaHei", 14, "bold")
FONT_BTN = ("Microsoft YaHei", 12, "bold")
FONT_LOG = ("Consolas", 11)
FONT_HELP = ("Microsoft YaHei", 12)

main_frame = tk.Frame(root, bg=BG)
main_frame.pack(padx=10, pady=10, fill="both", expand=True)

top_frame = tk.Frame(main_frame, bg=BG)
top_frame.pack(fill="both", expand=True)

left_frame = tk.Frame(top_frame, bg=BG)
left_frame.pack(side="left", padx=(0, 10), fill="both")

camera_container = tk.Frame(left_frame, bg=BORDER, bd=2, relief="solid")
camera_container.pack()

video_label = tk.Label(camera_container, bg="#c0c0c0")
video_label.pack()

log_section = tk.Frame(left_frame, bg=BG)
log_section.pack(fill="both", expand=True, pady=(8, 0))

tk.Label(log_section, text="日志", bg=BG, fg=FG_DIM,
         font=FONT_TITLE).pack(anchor="w")

log_container = tk.Frame(log_section, bg=BORDER, bd=2, relief="solid")
log_container.pack(fill="both", expand=True)

chat_scroll = tk.Scrollbar(log_container)
chat_scroll.pack(side="right", fill="y")

chat_box = tk.Text(
    log_container, width=45, height=15,
    bg=PANEL_BG, fg=FG, font=FONT_LOG,
    wrap="word", state="disabled",
    yscrollcommand=chat_scroll.set,
    relief="flat", bd=0,
    insertbackground=FG,
)
chat_box.pack(side="left", fill="both", expand=True)
chat_scroll.config(command=chat_box.yview)

chat_box.tag_config("sys", foreground=ACCENT_BLUE)
chat_box.tag_config("arm", foreground=ACCENT_GREEN)
chat_box.tag_config("warn", foreground=ACCENT_RED)
chat_box.tag_config("user", foreground=FG)

right_frame = tk.Frame(top_frame, bg=BG)
right_frame.pack(side="left", padx=10, fill="both", expand=True)

tk.Label(right_frame, text="说明", bg=BG, fg=FG_DIM,
         font=FONT_TITLE).pack(anchor="w")

help_container = tk.Frame(right_frame, bg=BORDER, bd=2, relief="solid")
help_container.pack(fill="both", expand=True)

help_box = tk.Text(
    help_container, width=40,
    bg=PANEL_BG, fg=FG, font=FONT_HELP,
    wrap="word", state="disabled",
    relief="flat", bd=4,
)
help_box.pack(side="left", fill="both", expand=True)

help_scroll = tk.Scrollbar(help_container, command=help_box.yview)
help_scroll.pack(side="right", fill="y")
help_box.config(yscrollcommand=help_scroll.set)


def load_help_text():
    help_content = """欢迎使用 DOFBOT 机械臂控制系统

【功能说明】

1. 待机模式
   - 机械臂保持初始姿态
   - 摄像头持续检测颜色

2. 颜色追踪
   - 选择追踪颜色后启动
   - 机械臂会追踪指定颜色的物体

3. 颜色抓取
   - 自动识别并抓取对应颜色方块
   - 放置到指定位置

4. 颜色学习
   - 学习新的颜色HSV范围
   - 将色块对准摄像头中心

5. 跟随学习
   - 基于学习的颜色进行跟随
   - 需先完成颜色学习

【语音指令】

基础动作：
• 鼓掌、点头、招手
• 弯腰、复位、左右摇摆

高级动作：
• 叠罗汉、跳舞
• 夹方块、搬运

其他：
• 问色 - 询问当前检测颜色

【注意事项】

1. 确保摄像头视野清晰
2. 光线充足以提高识别准确率
3. 操作前确认机械臂活动范围内无障碍物
"""
    help_box.config(state="normal")
    help_box.insert("1.0", help_content)
    help_box.config(state="disabled")


try:
    with open("help.txt", "r", encoding="utf-8") as f:
        help_text = f.read()
        help_box.config(state="normal")
        help_box.insert("1.0", help_text)
        help_box.config(state="disabled")
except:
    load_help_text()


def log_chat(msg, tag="sys"):
    def _append():
        chat_box.config(state="normal")
        chat_box.insert("end", msg + "\n", tag)
        chat_box.see("end")
        chat_box.config(state="disabled")
    root.after(0, _append)


bottom_frame = tk.Frame(main_frame, bg=BG)
bottom_frame.pack(fill="x", pady=(12, 0))

btn_row1 = tk.Frame(bottom_frame, bg=BG)
btn_row1.pack(fill="x")


def make_control_btn(parent, text, cmd, color=BTN_INACTIVE, active=False):
    btn = tk.Button(
        parent, text=text, command=cmd,
        bg=color if active else BTN_INACTIVE,
        fg="white",
        activebackground=color,
        activeforeground="white",
        font=FONT_BTN, width=10, height=2,
        relief="flat", bd=0, cursor="hand2"
    )
    return btn


control_buttons = {}


def set_mode(new_mode):
    global g_mode, HSV_learning
    g_mode = new_mode

    for mode, btn in control_buttons.items():
        if mode == new_mode:
            btn.config(bg=BTN_ACTIVE if mode in ['grab', 'learn_follow'] else BTN_TRACKING)
        else:
            btn.config(bg=BTN_INACTIVE)

    if new_mode == 'idle':
        robot.move_init_pose()
        log_chat("[系统] 已切换到待机模式")
    elif new_mode == 'grab':
        arm_move_6(look_at, 1000)
        log_chat("[系统] 已开启颜色抓取模式")
    elif new_mode == 'follow':
        log_chat("[系统] 已开启颜色追踪模式")
    elif new_mode == 'learn_color':
        log_chat("[系统] 开始学习颜色，请将色块对准摄像头")
    elif new_mode == 'learn_follow':
        if len(HSV_learning) != 0:
            log_chat("[系统] 开始学习跟随")
        else:
            log_chat("[系统] 请先学习颜色再启动跟随", tag="warn")
            g_mode = 'follow'
            return
    update_mode_label()


def open_calibration_window():
    calib_window = tk.Toplevel(root)
    calib_window.title("HSV 颜色校准")
    calib_window.configure(bg=BG)
    calib_window.geometry("1100x700")
    calib_window.resizable(True, True)

    calib_hsv_name = tk.StringVar(value="red")
    calib_mode = tk.StringVar(value="General")

    calib_left = tk.Frame(calib_window, bg=BG)
    calib_left.pack(side="left", padx=10, pady=10, fill="both", expand=True)

    calib_right = tk.Frame(calib_window, bg=BG)
    calib_right.pack(side="left", padx=10, pady=10, fill="y")

    calib_video_label = tk.Label(calib_left, bg="#c0c0c0")
    calib_video_label.pack()

    current_color_label = tk.Label(calib_left, text="当前校准: red", bg=BG, fg=ACCENT_RED,
                                    font=FONT_TITLE)
    current_color_label.pack(pady=(10, 0))

    tk.Label(calib_right, text="颜色选择", bg=BG, fg=FG_DIM, font=FONT_TITLE).pack(anchor="w", pady=(0, 10))

    color_btn_frame = tk.Frame(calib_right, bg=BG)
    color_btn_frame.pack(fill="x", pady=(0, 20))

    def select_calib_color(color_name, color_hex):
        calib_hsv_name.set(color_name)
        current_color_label.config(text=f"当前校准: {color_name}", fg=color_hex)
        hsv_values = color_hsv.get(color_name, ((0, 43, 46), (10, 255, 255)))
        h_min.set(hsv_values[0][0])
        s_min.set(hsv_values[0][1])
        v_min.set(hsv_values[0][2])
        h_max.set(hsv_values[1][0])
        s_max.set(hsv_values[1][1])
        v_max.set(hsv_values[1][2])

    color_buttons_calib = [
        ("red", ACCENT_RED, "红色"),
        ("green", ACCENT_GREEN, "绿色"),
        ("blue", ACCENT_BLUE, "蓝色"),
        ("yellow", ACCENT_YELLOW, "黄色"),
    ]

    for color_name, color_hex, label in color_buttons_calib:
        btn = tk.Button(color_btn_frame, text=label, bg=color_hex, fg="white",
                       font=FONT_BTN, width=8, height=1,
                       command=lambda c=color_name, h=color_hex: select_calib_color(c, h))
        btn.pack(side="left", padx=2)

    tk.Label(calib_right, text="HSV 调节", bg=BG, fg=FG_DIM, font=FONT_TITLE).pack(anchor="w", pady=(10, 10))

    slider_frame = tk.Frame(calib_right, bg=BG)
    slider_frame.pack(fill="x")

    h_min = tk.IntVar(value=color_hsv["red"][0][0])
    s_min = tk.IntVar(value=color_hsv["red"][0][1])
    v_min = tk.IntVar(value=color_hsv["red"][0][2])
    h_max = tk.IntVar(value=color_hsv["red"][1][0])
    s_max = tk.IntVar(value=color_hsv["red"][1][1])
    v_max = tk.IntVar(value=color_hsv["red"][1][2])

    sliders = [
        ("H_min", h_min, 0, 180),
        ("S_min", s_min, 0, 255),
        ("V_min", v_min, 0, 255),
        ("H_max", h_max, 0, 180),
        ("S_max", s_max, 0, 255),
        ("V_max", v_max, 0, 255),
    ]

    for label, var, min_val, max_val in sliders:
        frame = tk.Frame(slider_frame, bg=BG)
        frame.pack(fill="x", pady=2)
        tk.Label(frame, text=f"{label}:", bg=BG, fg=FG, font=FONT_HELP, width=8).pack(side="left")
        scale = tk.Scale(frame, from_=min_val, to=max_val, orient=tk.HORIZONTAL,
                        variable=var, bg=BG, fg=FG, font=FONT_HELP, length=200)
        scale.pack(side="left", fill="x", expand=True)

    tk.Label(calib_right, text="显示模式", bg=BG, fg=FG_DIM, font=FONT_TITLE).pack(anchor="w", pady=(20, 10))

    mode_frame = tk.Frame(calib_right, bg=BG)
    mode_frame.pack(fill="x")

    tk.Radiobutton(mode_frame, text="原图", variable=calib_mode, value="General",
                  bg=BG, fg=FG, selectcolor=PANEL_BG, font=FONT_HELP).pack(anchor="w")
    tk.Radiobutton(mode_frame, text="二值图", variable=calib_mode, value="Binary",
                  bg=BG, fg=FG, selectcolor=PANEL_BG, font=FONT_HELP).pack(anchor="w")

    btn_frame = tk.Frame(calib_right, bg=BG)
    btn_frame.pack(fill="x", pady=(30, 0))

    def save_hsv_config():
        color_name = calib_hsv_name.get()
        color_hsv[color_name] = (
            (h_min.get(), s_min.get(), v_min.get()),
            (h_max.get(), s_max.get(), v_max.get())
        )
        try:
            write_HSV(HSV_path, color_hsv)
            log_chat(f"[系统] {color_name} HSV配置已保存到 {HSV_path}")
        except Exception as e:
            log_chat(f"[错误] 保存HSV失败: {e}", tag="warn")

    def close_calibration():
        calib_window.destroy()

    tk.Button(btn_frame, text="保存配置", command=save_hsv_config,
             bg=ACCENT_GREEN, fg="white", font=FONT_BTN, width=15, height=2).pack(pady=5)
    tk.Button(btn_frame, text="关闭窗口", command=close_calibration,
             bg=BTN_INACTIVE, fg="white", font=FONT_BTN, width=15, height=2).pack(pady=5)

    def calibration_camera_thread():
        global g_calib_window_open
        g_calib_window_open = True

        while calib_window.winfo_exists():
            if g_current_frame is None:
                time.sleep(0.01)
                continue

            frame = g_current_frame.copy()
            color_name = calib_hsv_name.get()

            color_hsv[color_name] = (
                (h_min.get(), s_min.get(), v_min.get()),
                (h_max.get(), s_max.get(), v_max.get())
            )

            cv2.putText(frame, color_name, (280, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.5,
                       (0, 255, 0), 3)
            cv2.rectangle(frame, (220, 180), (420, 300), (0, 255, 0), 2)

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            lower = np.array(color_hsv[color_name][0])
            upper = np.array(color_hsv[color_name][1])
            mask = cv2.inRange(hsv, lower, upper)

            if calib_mode.get() == "Binary":
                display_frame = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            else:
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                display_frame = frame.copy()
                cv2.drawContours(display_frame, contours, -1, (0, 255, 0), 2)

            img_rgb = cv2.cvtColor(display_frame, cv2.COLOR_BGR2RGB)
            pil_img = Image.fromarray(img_rgb)
            imgtk = ImageTk.PhotoImage(image=pil_img)

            def update(img=imgtk):
                if calib_video_label.winfo_exists():
                    calib_video_label.imgtk = img
                    calib_video_label.config(image=img)

            calib_window.after(0, update)
            time.sleep(0.033)

        g_calib_window_open = False

    threading.Thread(target=calibration_camera_thread, daemon=True).start()


btn_configs = [
    ("idle", "待机", lambda: set_mode('idle')),
    ("follow", "颜色追踪", lambda: set_mode('follow')),
    ("grab", "颜色抓取", lambda: set_mode('grab')),
    ("learn_color", "颜色学习", lambda: set_mode('learn_color')),
    ("learn_follow", "跟随学习", lambda: set_mode('learn_follow')),
]

for mode_key, text, cmd in btn_configs:
    btn = make_control_btn(btn_row1, text, cmd)
    btn.pack(side="left", padx=4)
    control_buttons[mode_key] = btn

calib_btn = tk.Button(btn_row1, text="颜色校准", command=open_calibration_window,
                     bg=BTN_INACTIVE, fg="white", font=FONT_BTN, width=10, height=2)
calib_btn.pack(side="left", padx=4)

color_var = tk.StringVar(value="red")

tk.Label(btn_row1, text="  追踪颜色", bg=BG, fg=FG_DIM,
         font=FONT_HELP).pack(side="left", padx=(20, 5))

color_buttons = {}
color_styles = [
    ("red", ACCENT_RED, "红色"),
    ("green", ACCENT_GREEN, "绿色"),
    ("blue", ACCENT_BLUE, "蓝色"),
    ("yellow", ACCENT_YELLOW, "黄色"),
]

for color_name, color_hex, color_label in color_styles:
    btn = tk.Button(
        btn_row1, text=color_label,
        bg=color_hex, fg="white",
        activebackground=color_hex,
        font=FONT_HELP, width=4, height=1,
        relief="raised", bd=2, cursor="hand2",
        command=lambda c=color_name: select_color(c)
    )
    btn.pack(side="left", padx=2)
    color_buttons[color_name] = btn

tk.Label(btn_row1, text="  当前模式", bg=BG, fg=FG_DIM,
         font=FONT_HELP).pack(side="left", padx=(20, 5))

mode_label = tk.Label(btn_row1, text="● 待机中",
                       bg=BG, fg=BTN_TRACKING, font=FONT_BTN)
mode_label.pack(side="left")


def select_color(color):
    color_var.set(color)
    for c, btn in color_buttons.items():
        if c == color:
            btn.config(relief="sunken", bd=3)
        else:
            btn.config(relief="raised", bd=2)
    log_chat(f"[系统] 已选择追踪颜色：{color}")


def update_mode_label():
    text_map = {
        'idle': ("● 待机中", FG_DIM),
        'grab': ("● 颜色抓取中", BTN_ACTIVE),
        'follow': ("● 颜色追踪中", BTN_TRACKING),
        'learn_color': ("● 颜色学习中", ACCENT_BLUE),
        'learn_follow': ("● 跟随学习中", BTN_ACTIVE),
    }
    txt, col = text_map.get(g_mode, ("● 待机中", FG_DIM))
    mode_label.config(text=txt, fg=col)


select_color("red")


def camera_thread():
    global g_current_color, g_mode, HSV_learning, g_current_frame

    Arm.Arm_Buzzer_On(1)
    s_time = 300
    for _ in range(2):
        Arm.Arm_serial_servo_write(4, 10, s_time)
        time.sleep(s_time / 1000)
        Arm.Arm_serial_servo_write(4, 0, s_time)
        time.sleep(s_time / 1000)

    while True:
        if g_mode == 'Exit':
            break

        ret, frame = cap.read()
        if not ret:
            time.sleep(0.01)
            continue

        g_current_frame = frame.copy()

        if g_mode == 'grab':
            frame, color_name = get_color(frame)
            g_current_color = color_name.get('name', None)
            if g_current_color:
                grab_map = {'yellow': 1, 'red': 2, 'green': 3, 'blue': 4}
                if g_current_color in grab_map:
                    start_move_arm(grab_map[g_current_color])
            cv2.rectangle(frame, (0, 0), (310, 36), (0, 0, 0), -1)
            cv2.putText(frame, f"颜色抓取模式 | {g_current_color or '未识别到颜色'}",
                        (8, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (100, 255, 100), 2)

        elif g_mode == 'follow':
            current_color = color_var.get()
            frame = follow_ctrl.follow_function(frame, color_hsv[current_color])
            cv2.putText(frame, current_color,
                        (int(frame.shape[1] / 2), 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 2,
                        rand_colors[random.randint(0, 254)], 2)

        elif g_mode == 'learn_color':
            frame, HSV_learning = follow_ctrl.get_hsv(frame)

        elif g_mode == 'learn_follow':
            if len(HSV_learning) != 0:
                frame = follow_ctrl.learning_follow(frame, HSV_learning)
                cv2.putText(frame, '学习颜色', (240, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 1,
                            rand_colors[random.randint(0, 254)], 1)

        else:
            frame, color_name = get_color(frame)
            g_current_color = color_name.get('name', None)
            cv2.rectangle(frame, (0, 0), (210, 36), (0, 0, 0), -1)
            cv2.putText(frame, f"待机 | {g_current_color or '未识别到颜色'}",
                        (8, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (150, 150, 150), 2)

        img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        pil_img = Image.fromarray(img_rgb)
        imgtk = ImageTk.PhotoImage(image=pil_img)

        def _update(tk_img=imgtk):
            video_label.imgtk = tk_img
            video_label.config(image=tk_img)
        root.after(0, _update)

        time.sleep(0.033)

    cap.release()


def speech_thread():
    arm_move_6(look_at, 1000)
    time.sleep(1)
    log_chat("[系统] 系统就绪，等待语音指令...")

    while True:
        result = mySpeech.speech_read()

        if result is None or result == 0 or result == 999:
            time.sleep(0.1)
            continue

        if result not in RESULT_NAME:
            log_chat(f"[系统] 未识别成功（code={result}），请再说一次", tag="warn")
            time.sleep(0.1)
            continue

        if g_mode in ('grab', 'follow'):
            log_chat(f"[系统] 当前处于抓取/跟踪功能，语音功能不可用，请点击待机再试", tag="warn")
            time.sleep(0.1)
            continue

        log_chat(f"[用户] {RESULT_NAME[result]}", tag="user")

        if result in (45, 46, 47, 48, 49, 50):
            threading.Thread(target=do_basic_action, args=[result], daemon=True).start()

        elif result == 51:
            log_chat("[机械臂] 开始叠罗汉", tag="arm")
            mySpeech.void_write(51)
            threading.Thread(target=heap_up, daemon=True).start()

        elif result == 52:
            log_chat("[机械臂] 开始跳舞", tag="arm")
            mySpeech.void_write(52)
            threading.Thread(target=dance, daemon=True).start()

        elif result == 53:
            log_chat("[机械臂] 开始夹方块", tag="arm")
            mySpeech.void_write(53)
            threading.Thread(target=clamp_clock, daemon=True).start()

        elif result == 54:
            log_chat("[机械臂] 开始搬运", tag="arm")
            Arm.Arm_Buzzer_On(3)
            time.sleep(0.5)
            mySpeech.void_write(54)
            threading.Thread(target=move_block, daemon=True).start()

        elif result == 60:
            voice_map = {'yellow': 64, 'red': 61, 'green': 63, 'blue': 62}
            color_cn_map = {'yellow': '黄色', 'red': '红色',
                            'green': '绿色', 'blue': '蓝色'}
            if g_current_color in voice_map:
                mySpeech.void_write(voice_map[g_current_color])
                log_chat(f"[系统] 当前颜色：{color_cn_map[g_current_color]}")
            else:
                log_chat("[系统] 当前未检测到颜色", tag="warn")

        time.sleep(0.1)


threading.Thread(target=camera_thread, daemon=True).start()
threading.Thread(target=speech_thread, daemon=True).start()


def on_close():
    global g_mode, Arm
    g_mode = 'Exit'
    time.sleep(0.2)
    cap.release()
    del Arm
    cleanup_on_exit()
    root.destroy()


root.protocol("WM_DELETE_WINDOW", on_close)

log_chat("[系统] 界面启动完成")

root.mainloop()