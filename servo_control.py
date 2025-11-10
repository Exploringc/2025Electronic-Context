from media.sensor import *
from media.display import *
from media.media import *
from machine import FPIOA
from machine import Pin
from machine import PWM
from machine import TOUCH
from machine import UART
from machine import Timer
import time
import math
import lvgl as lv
import cv_lite  # 导入cv_lite扩展模块
import ulab.numpy as np  # 导入numpy库

class PID:
    def __init__(self,Kp=1,Ki=0,Kd=0,outmax=100,outmin=-100,smooth_factor=0.0):
        self.Kp = Kp
        self.Kd = Kd
        self.Ki = Ki
        self.last_error = 0
        self.last_out = 0
        self.integral = 0
        self.outmax = outmax
        self.outmin = outmin
        self.smooth_factor = smooth_factor

    def Calc(self, input_value, setpoint):
        error = setpoint - input_value
        derivative = error - self.last_error
        self.integral += error
        self.last_error = error
        if self.integral >= self.outmax/9:
            self.integral = self.outmax/9
        elif self.integral <= self.outmin/9:
            self.integral = self.outmin/9
        output = max(min(self.Kp * error + self.Ki * self.integral + self.Kd * derivative, self.outmax), self.outmin)
        if self.smooth_factor > 0:
            output = output * (1 - self.smooth_factor) + self.last_out * self.smooth_factor
        self.last_out = output
        return int(output)

class PIDIncremental:
    def __init__(self, Kp, Ki, Kd, outmax, outmin, use_lowpass_filter, lowpass_filter_factor):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self._Kp = 0
        self._Ki = 0
        self._Kd = 0
        self.error = 0
        self.last_error = 0
        self.last_last_error = 0
        self.last_out = 0
        self.out = 0
        self.outmax = outmax
        self.outmin = outmin
        self.use_lowpass_filter = use_lowpass_filter
        self.lowpass_filter_factor = lowpass_filter_factor

    def adaptive_parameter(self, input_value, setpoint):
        ratio = (math.tanh(abs(input_value - setpoint) / 100))
        self._Kp = self.Kp * ratio
        self._Ki = self.Ki * ratio
        self._Kd = self.Kd * ratio

    def calculate(self, input_value, setpoint):
        self.adaptive_parameter(input_value, setpoint)
        self.last_last_error = self.last_error
        self.last_error = self.error
        self.error = setpoint - input_value
        derivative = self.error - 2 * self.last_error + self.last_last_error
        output_increment = self._Kp * (self.error - self.last_error) + self._Ki * self.error + self._Kd * derivative

        self.out += output_increment

        # Output limit
        if self.out > self.outmax:
            self.out = self.outmax
        elif self.out < self.outmin:
            self.out = self.outmin

        # Low pass filter
        if self.use_lowpass_filter:
            self.out = self.last_out * self.lowpass_filter_factor + self.out * (1 - self.lowpass_filter_factor)

        self.last_out = self.out

        return self.out


class AverageFilter:
    def __init__(self, window_size=5):
        self.window_size = window_size
        self.values = []

    def update(self, value):
        self.values.append(value)
        if len(self.values) > self.window_size:
            self.values.pop(0)
        return self.get_average()

    def get_average(self):
        if not self.values:
            return 0
        return sum(self.values) // len(self.values)

fpioa = FPIOA()
fpioa.set_function(49,FPIOA.GPIO49)

fpioa.set_function(42,FPIOA.PWM0)
fpioa.set_function(52,FPIOA.PWM4)

fpioa.set_function(11,FPIOA.UART2_TXD)
fpioa.set_function(12,FPIOA.UART2_RXD)

fpioa.set_function(50,FPIOA.UART3_TXD)
fpioa.set_function(51,FPIOA.UART3_RXD)

laser = Pin(49, Pin.OUT, pull=Pin.PULL_NONE, drive=7)

uart1 = UART(UART.UART2,baudrate=460800,bits=UART.EIGHTBITS,parity=UART.PARITY_NONE,stop=UART.STOPBITS_ONE)
uart2 = UART(UART.UART3,baudrate=460800,bits=UART.EIGHTBITS,parity=UART.PARITY_NONE,stop=UART.STOPBITS_ONE)

x_servo = PWM(0,freq=200)
y_servo = PWM(4,freq=200)

DISPLAY_WIDTH = 400
DISPLAY_HEIGHT = 340
picture_w = 400
picture_h = 340
img_center_x = picture_w // 2
img_center_y = picture_h // 2

#tp = TOUCH(0)
tim = Timer(-1)

def tim_cb(t):
    global task
    point = tp.read(1)

    if len(point):
        pt = point[0]
        # Handle touch events (down or move)
        # 处理触摸事件（按下或移动）
        if pt.event == 0 or pt.event == TOUCH.EVENT_UP or pt.event == TOUCH.EVENT_DOWN or pt.event == TOUCH.EVENT_MOVE:
            if 0 <= pt.x <= 80 and 0 <= pt.y <= 80:
                print("task1_2")
                task = 2
            elif 0 <= pt.x <= 80 and 120 <= pt.y <= 200:
                print("task1_3")
                task = 3
            elif 0 <= pt.x <= 80 and 240 <= pt.y <= 320:
                print("task2_1")
                task = 4
            elif 0 <= pt.x <= 80 and 400 <= pt.y <= 480:
                pass

sensor = Sensor(id=2)
sensor.reset()
sensor.set_vflip(True)
sensor.set_hmirror(True)

sensor.set_framesize(width=picture_w,height=picture_h,chn=CAM_CHN_ID_0)
sensor.set_pixformat(Sensor.RGB565,chn=CAM_CHN_ID_0)
Display.init(Display.VIRT,width=DISPLAY_WIDTH,height=DISPLAY_HEIGHT)
MediaManager.init()
sensor.run()

color_threshold = [(0,58)]
no_rect_count = 0
circle_r = 0
last_cx = 0
last_cy = 0
last_target_x = 0
last_target_y = 0
last_angle = 0
search_count = 0
servo_reset_flag1 = 0
servo_reset_flag2 = 0
search_count1 = 0
search_count2 = 0
circle_point_list1 = []
circle_point_list2 = []
circle_point_list3 = []
circle_point_list4 = []
x_servo_pid = PIDIncremental(Kp=0.5,Ki=0.2,Kd=0,outmax=360,outmin=-360,use_lowpass_filter=0, lowpass_filter_factor=0)
y_servo_pid = PIDIncremental(Kp=0,Ki=0.7,Kd=0,outmax=90,outmin=-90,use_lowpass_filter=0, lowpass_filter_factor=0)

img2 = None
set_init_position_flag = 0

rect_cx_filter = AverageFilter(window_size=3)
rect_cy_filter = AverageFilter(window_size=3)
blob_cx_filter = AverageFilter(window_size=3)
blob_cy_filter = AverageFilter(window_size=3)

# --------------------------- 配置参数 ---------------------------
# 矩形检测核心参数（基于cv_lite）
canny_thresh1      = 30        # Canny边缘检测低阈值
canny_thresh2      = 100       # Canny边缘检测高阈值
approx_epsilon     = 0.04      # 多边形拟合精度（越小越精确）
area_min_ratio     = 0.005     # 最小面积比例（相对于图像总面积）
max_angle_cos      = 0.3       # 角度余弦阈值（越小越接近矩形）
gaussian_blur_size = 3         # 高斯模糊核尺寸（奇数）

# 原有筛选参数
MIN_AREA = 1000               # 最小面积阈值
MAX_AREA = 100000             # 最大面积阈值
MIN_ASPECT_RATIO = 0.3        # 最小宽高比
MAX_ASPECT_RATIO = 3.0        # 最大宽高比

# 虚拟坐标与圆形参数
BASE_RADIUS = 70              # 基础半径（虚拟坐标单位）
POINTS_PER_CIRCLE = 24        # 圆形采样点数量
PURPLE_THRESHOLD = (20, 60, 15, 70, -70, -20)  # 紫色色块阈值

# 基础矩形参数（固定方向，不再自动切换）
RECT_WIDTH = 297    # 固定矩形宽度
RECT_HEIGHT = 210    # 固定矩形高度
# 移除自动切换方向的逻辑，始终使用固定宽高的虚拟矩形

# --------------------------- 工具函数 ---------------------------
def calculate_distance(p1, p2):
    return math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])** 2)

def calculate_center(points):
    if not points:
        return (0, 0)
    sum_x = sum(p[0] for p in points)
    sum_y = sum(p[1] for p in points)
    return (sum_x / len(points), sum_y / len(points))

def is_valid_rect(corners):
    edges = [calculate_distance(corners[i], corners[(i+1)%4]) for i in range(4)]

    # 对边比例校验
    ratio1 = edges[0] / max(edges[2], 0.1)
    ratio2 = edges[1] / max(edges[3], 0.1)
    valid_ratio = 0.5 < ratio1 < 1.5 and 0.5 < ratio2 < 1.5

    # 面积校验
    area = 0
    for i in range(4):
        x1, y1 = corners[i]
        x2, y2 = corners[(i+1) % 4]
        area += (x1 * y2 - x2 * y1)
    area = abs(area) / 2
    valid_area = MIN_AREA < area < MAX_AREA

    # 宽高比校验
    width = max(p[0] for p in corners) - min(p[0] for p in corners)
    height = max(p[1] for p in corners) - min(p[1] for p in corners)
    aspect_ratio = width / max(height, 0.1)
    valid_aspect = MIN_ASPECT_RATIO < aspect_ratio < MAX_ASPECT_RATIO

    return valid_ratio and valid_area and valid_aspect

def detect_purple_blobs(img):
    return img.find_blobs(
        [PURPLE_THRESHOLD],
        pixels_threshold=100,
        area_threshold=100,
        merge=True
    )

def send_circle_points(points):
    if not points:
        return
    count = len(points)
    msg = f"$$C,{count},"
    for x, y in points:
        msg += f"{x},{y},"
    msg = msg.rstrip(',') + "##"
    uart.write(msg)
    # print(f"发送圆形点: {msg}")

def get_perspective_matrix(src_pts, dst_pts):
    """计算透视变换矩阵"""
    A = []
    B = []
    for i in range(4):
        x, y = src_pts[i]
        u, v = dst_pts[i]
        A.append([x, y, 1, 0, 0, 0, -u*x, -u*y])
        A.append([0, 0, 0, x, y, 1, -v*x, -v*y])
        B.append(u)
        B.append(v)

    # 高斯消元求解矩阵
    n = 8
    for i in range(n):
        max_row = i
        for j in range(i, len(A)):
            if abs(A[j][i]) > abs(A[max_row][i]):
                max_row = j
        A[i], A[max_row] = A[max_row], A[i]
        B[i], B[max_row] = B[max_row], B[i]

        pivot = A[i][i]
        if abs(pivot) < 1e-8:
            return None
        for j in range(i, n):
            A[i][j] /= pivot
        B[i] /= pivot

        for j in range(len(A)):
            if j != i and A[j][i] != 0:
                factor = A[j][i]
                for k in range(i, n):
                    A[j][k] -= factor * A[i][k]
                B[j] -= factor * B[i]

    return [
        [B[0], B[1], B[2]],
        [B[3], B[4], B[5]],
        [B[6], B[7], 1.0]
    ]

def transform_points(points, matrix):
    """应用透视变换将虚拟坐标映射到原始图像坐标"""
    transformed = []
    for (x, y) in points:
        x_hom = x * matrix[0][0] + y * matrix[0][1] + matrix[0][2]
        y_hom = x * matrix[1][0] + y * matrix[1][1] + matrix[1][2]
        w_hom = x * matrix[2][0] + y * matrix[2][1] + matrix[2][2]
        if abs(w_hom) > 1e-8:
            transformed.append((x_hom / w_hom, y_hom / w_hom))
    return transformed

def sort_corners(corners):
    """将矩形角点按左上、右上、右下、左下顺序排序"""
    center = calculate_center(corners)
    sorted_corners = sorted(corners, key=lambda p: math.atan2(p[1]-center[1], p[0]-center[0]))

    # 调整顺序为左上、右上、右下、左下
    if len(sorted_corners) == 4:
        left_top = min(sorted_corners, key=lambda p: p[0]+p[1])
        index = sorted_corners.index(left_top)
        sorted_corners = sorted_corners[index:] + sorted_corners[:index]
    return sorted_corners

def get_rectangle_orientation(corners):
    """计算矩形的主方向角（水平边与x轴的夹角）"""
    if len(corners) != 4:
        return 0

    # 计算上边和右边的向量
    top_edge = (corners[1][0] - corners[0][0], corners[1][1] - corners[0][1])
    right_edge = (corners[2][0] - corners[1][0], corners[2][1] - corners[1][1])

    # 选择较长的边作为主方向
    if calculate_distance(corners[0], corners[1]) > calculate_distance(corners[1], corners[2]):
        main_edge = top_edge
    else:
        main_edge = right_edge

    # 计算主方向角（弧度）
    angle = math.atan2(main_edge[1], main_edge[0])
    return angle

def rect_detect(img):
    global last_cx,last_cy
    gray_img = img.to_grayscale()
    img_np = gray_img.to_numpy_ref()  # 转为numpy数组供cv_lite使用

    # 2.2 调用cv_lite矩形检测函数（带角点）
    rects = cv_lite.grayscale_find_rectangles_with_corners(
        image_shape,       # 图像尺寸 [高, 宽]
        img_np,            # 灰度图数据
        canny_thresh1,     # Canny低阈值
        canny_thresh2,     # Canny高阈值
        approx_epsilon,    # 多边形拟合精度
        area_min_ratio,    # 最小面积比例
        max_angle_cos,     # 角度余弦阈值
        gaussian_blur_size # 高斯模糊尺寸
    )

    # 3. 筛选最小矩形（保留原有逻辑）
    min_area = float('inf')
    smallest_rect = None
    smallest_rect_corners = None  # 存储最小矩形的角点

    for rect in rects:
        # rect格式: [x, y, w, h, c1.x, c1.y, c2.x, c2.y, c3.x, c3.y, c4.x, c4.y]
        x, y, w, h = rect[0], rect[1], rect[2], rect[3]
        # 提取四个角点
        corners = [
            (rect[4], rect[5]),   # 角点1
            (rect[6], rect[7]),   # 角点2
            (rect[8], rect[9]),   # 角点3
            (rect[10], rect[11])  # 角点4
        ]

        # 验证矩形有效性
        if is_valid_rect(corners):
            # 计算面积
            area = w * h  # 直接使用矩形宽高计算面积（更高效）
            # 更新最小矩形
            if area < min_area:
                min_area = area
                smallest_rect = (x, y, w, h)
                smallest_rect_corners = corners

    # 4. 处理最小矩形（修改后：固定虚拟矩形方向）
    if smallest_rect and smallest_rect_corners:
        x, y, w, h = smallest_rect
        corners = smallest_rect_corners

        # 对矩形角点进行排序
        sorted_corners = sort_corners(corners)

        # 绘制矩形边框和角点
        for i in range(4):
            x1, y1 = sorted_corners[i]
            x2, y2 = sorted_corners[(i+1) % 4]
            img.draw_line(x1, y1, x2, y2, color=(255, 0, 0), thickness=2)
        for p in sorted_corners:
            img.draw_circle(p[0], p[1], 5, color=(0, 255, 0), thickness=2)

        # 计算并绘制矩形中心点
        rect_center = calculate_center(sorted_corners)
        rect_center_int = (int(round(rect_center[0])), int(round(rect_center[1])))
#        img.draw_circle(rect_center_int[0], rect_center_int[1], 4, color=(0, 255, 255), thickness=2)

        # 计算矩形主方向角（仅用于参考，不再影响虚拟矩形方向）
        angle = get_rectangle_orientation(sorted_corners)

        # 【核心修改】移除自动切换方向逻辑，固定使用预设的虚拟矩形尺寸和方向
        # 固定虚拟矩形（不再根据实际宽高比切换）
        virtual_rect = [
            (0, 0),
            (RECT_WIDTH, 0),
            (RECT_WIDTH, RECT_HEIGHT),
            (0, RECT_HEIGHT)
        ]

        # 【核心修改】固定圆形半径参数（不再根据实际宽高比调整）
        radius_x = BASE_RADIUS
        radius_y = BASE_RADIUS

        # 【核心修改】固定虚拟中心（基于固定的宽高）
        virtual_center = (RECT_WIDTH / 2, RECT_HEIGHT / 2)

        # 在虚拟矩形中生成椭圆点集（映射后为正圆）
        virtual_circle_points = []
        for i in range(POINTS_PER_CIRCLE):
            angle_rad = 2 * math.pi * i / POINTS_PER_CIRCLE
            x_virt = virtual_center[0] + radius_x * math.cos(angle_rad)
            y_virt = virtual_center[1] - radius_y * math.sin(angle_rad)
            virtual_circle_points.append((x_virt, y_virt))

        # 计算透视变换矩阵并映射坐标
        matrix = get_perspective_matrix(virtual_rect, sorted_corners)
        if matrix:
            mapped_points = transform_points(virtual_circle_points, matrix)
            int_points = [(int(round(x)), int(round(y))) for x, y in mapped_points]

            # 绘制圆形
            for (px, py) in int_points:
                img.draw_circle(px, py, 2, color=(255, 0, 255), thickness=2)

            # 绘制圆心
            mapped_center = transform_points([virtual_center], matrix)
            if mapped_center:
                cx, cy = map(int, map(round, mapped_center[0]))
                last_cx = cx
                last_cy = cy
                img.draw_circle(cx, cy, 3, color=(0, 0, 255), thickness=1)
                target_flag = 1
                return (cx,cy,target_flag)
    else:
        target_flag = 0
        return (last_cx,last_cy,target_flag)

def angle_to_coord(angle):
    """
    将 0~360 度映射回 0~16383 的坐标值
    """
    coord = int(angle * 16384 / 360.0)
    if coord > 16383:
        coord = 16383
    return coord

def getCheckSum(buffer, size):
    """
    功能：计算一组数据的校验和
    输入：buffer - 待校验数据 (列表或字节序列)
          size   - 待校验数据个数
    输出：校验值 (0-255)
    """
    total = 0
    for i in range(size):
        total += buffer[i]  # 累加所有字节值
    return total & 0xFF     # 返回低8位作为校验和

def positionMode3Run(serial_port, slaveAddr, speed, acc, angle):
    """
    功能：串口发送位置模式3运行指令
    输入：serial_port - pyserial 串口对象
          slaveAddr   - 从机地址 (0-255)
          speed       - 运行速度 (0-65535)
          acc         - 加速度 (0-255)
          absAxis     - 绝对坐标 (32位有符号整数)
    """
    txBuffer = [0] * 11  # 创建11字节的发送缓冲区
    absAxis = angle_to_coord(angle)
    # 构建数据帧
    txBuffer[0] = 0xFA               # 帧头
    txBuffer[1] = slaveAddr          # 从机地址
    txBuffer[2] = 0xF5               # 功能码
    txBuffer[3] = (speed >> 8) & 0xFF # 速度高8位
    txBuffer[4] = speed & 0xFF        # 速度低8位
    txBuffer[5] = acc                # 加速度
    txBuffer[6] = (absAxis >> 24) & 0xFF  # 绝对坐标 bit31-bit24
    txBuffer[7] = (absAxis >> 16) & 0xFF  # 绝对坐标 bit23-bit16
    txBuffer[8] = (absAxis >> 8) & 0xFF   # 绝对坐标 bit15-bit8
    txBuffer[9] = absAxis & 0xFF          # 绝对坐标 bit7-bit0

    # 计算并添加校验和 (前10字节)
    txBuffer[10] = getCheckSum(txBuffer, 10)

    # 通过串口发送数据
    serial_port.write(bytes(txBuffer))

def line_intersection(line1, line2):
    """
    计算两条直线的交点坐标
    """
    (x1, y1, x2, y2) = line1
    (x3, y3,x4, y4) = line2

    # 计算分母（判断是否平行）
    den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if den == 0:
        # 分母为0，两直线平行或重合，无唯一交点
        return (abs(x2-x1)//2,abs(y2-y1)//2)

    # 计算分子
    t_num = (x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)
    s_num = (x1 - x3) * (y1 - y2) - (y1 - y3) * (x1 - x2)

    t = t_num / den
    s = s_num / den

    # 计算交点坐标
    x = x1 + t * (x2 - x1)
    y = y1 + t * (y2 - y1)

    return (x, y)

def y_set_angle(angle=0):
    positionMode3Run(uart1, 2, 600, 0, angle)

def x_set_angle(angle=0):
    positionMode3Run(uart1, 1, 600, 0, angle)

def brushless_angle_to_coord(angle):
    """
    将 0~360 度映射回 0~32500 的坐标值
    """
    coord = int(angle * 32500 / 360.0)
    if coord > 32500:
        coord = 32500
    return coord

def brushless_set_angle(x_angle=0,y_angle=0):
    x_duty = brushless_angle_to_coord(x_angle)
    y_duty = brushless_angle_to_coord(y_angle)

    data_str = "{},{},{}".format("SET-DUTY", x_duty, y_duty)

    uart1.write((data_str + '\n').encode())

def position_point(input_x,input_y,target_x,target_y,basic_angle_x=0,basic_angle_y=0):
    global last_target_x,last_target_y,last_angle,x_servo_pid,y_servo_pid
    if last_target_x != target_x or last_target_y != target_y :
        x_devation = target_x - input_x
        y_devation = target_y - input_y
        x_pid_out = x_servo_pid.calculate(x_devation,0)
        y_pid_out = y_servo_pid.calculate(y_devation,0)

        print(f"target:({target_x},{target_y}) input:({input_x},{input_y}) diff_x:{x_devation} diff_y:{y_devation} x_pid_out:{x_pid_out} y_pid_out:{y_pid_out}")
    #    x_set_angle(basic_angle_x + x_pid_out)
    #    y_set_angle(basic_angle_y - y_pid_out)
        brushless_set_angle(basic_angle_x + x_pid_out)
        last_angle = basic_angle_x + x_pid_out
        last_target_x = target_x
        last_target_y = target_y
        return basic_angle_x + x_pid_out
    else:
        return last_angle

def servo_reset():
#    x_set_angle(135)
#    y_set_angle(0)
    brushless_set_angle(0,0)

def task1_2(img):
    rect_cx,rect_cy,_= rect_detect(img)
    position_point(img_center_x,img_center_y,rect_cx,rect_cy)

def task1_3(img,search_step_angle=0.5):
    global servo_reset_flag1,servo_reset_flag2,search_count1,search_count2
    global no_rect_count,search_count,x_servo_pid,y_servo_pid

    rect_cx,rect_cy,target = rect_detect(img)
    if target == 1:
#        rect_cx = rect_cx_filter.update(rect_cx)
#        rect_cy = rect_cy_filter.update(rect_cy)
        img.draw_circle(rect_cx,rect_cy,2,color=(255,0,0),thickness=4)

        if servo_reset_flag2 == 0:
#            if search_count < 40:
#                search_count += 1
#                x_servo_pid = PIDIncremental(Kp=0,Ki=0.01,Kd=0,outmax=360,outmin=-360,use_lowpass_filter=0, lowpass_filter_factor=0)
#                y_servo_pid = PIDIncremental(Kp=0.4,Ki=0.4,Kd=0,outmax=90,outmin=-90,use_lowpass_filter=0, lowpass_filter_factor=0)
#            else:
#                x_servo_pid = PIDIncremental(Kp=0.03,Ki=0.2,Kd=0,outmax=360,outmin=-360,use_lowpass_filter=0, lowpass_filter_factor=0)
#                y_servo_pid = PIDIncremental(Kp=0,Ki=0.7,Kd=0,outmax=90,outmin=-90,use_lowpass_filter=0, lowpass_filter_factor=0)

#                print(x_servo_pid.Ki)
##            if rect_cx - img_center_x > 10:
            position_point(img_center_x,img_center_y,rect_cx,rect_cy,basic_angle_x=search_step_angle * search_count1)
        else:
#            if search_count < 5:
#                search_count += 1
#                x_servo_pid = PIDIncremental(Kp=0.01,Ki=0.06,Kd=0,outmax=360,outmin=-360,use_lowpass_filter=0, lowpass_filter_factor=0)
#                y_servo_pid = PIDIncremental(Kp=0,Ki=0.7,Kd=0,outmax=90,outmin=-90,use_lowpass_filter=0, lowpass_filter_factor=0)
#            else:
#                x_servo_pid = PIDIncremental(Kp=0.3,Ki=0.2,Kd=0,outmax=360,outmin=-360,use_lowpass_filter=0, lowpass_filter_factor=0)
#                y_servo_pid = PIDIncremental(Kp=0,Ki=0.7,Kd=0,outmax=90,outmin=-90,use_lowpass_filter=0, lowpass_filter_factor=0)
#            print(x_servo_pid.Ki)
            position_point(img_center_x,img_center_y,rect_cx,rect_cy,basic_angle_x=search_step_angle * search_count2)

    else:
        no_rect_count += 1
        if no_rect_count >= 10:
            if servo_reset_flag1 == 0:
                servo_reset()
                servo_reset_flag1 = 1
            if search_count1 <= int(170/search_step_angle):
                search_count1 += 1
                brushless_set_angle(search_step_angle * search_count1)
            else:
                if servo_reset_flag2 == 0:
                    servo_reset()
                    servo_reset_flag2 = 1
                if search_count2 <= int(170/search_step_angle):
                    search_count2 += 1
                    brushless_set_angle(-search_step_angle * search_count2)

def task2_12(img):
    if set_init_position_flag == 0:
        brushless_set_angle(90)
        set_init_position_flag = 1
        current_angle = 0
        delta_yaw = None

    rect_cx,rect_cy,_ = rect_detect(img)
    if rect_cx != 0 and rect_cy != 0:
        current_angle = position_point(img_center_x,img_center_y,rect_cx,rect_cy,basic_x_angle=90)
    else:
        delta_yaw = uart2.read()
        if delta_yaw is not None:
            brushless_set_angle(current_angle + delta_yaw)

def task2_3(img):
    pass

def display_deinit():
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(50)
    # deinit display
    Display.deinit()
    # release media buffer
    MediaManager.deinit()

tasks = {
    0:servo_reset,
    2:task1_2,
    3:task1_3,
    4:task2_12,
    6:task2_3
}
task = 3

def creat_ui():
    global img2
    img2 = image.Image(DISPLAY_WIDTH, DISPLAY_HEIGHT, image.ARGB8888)
    img2.clear()
    img2.draw_rectangle(0,0,80,80)
    img2.draw_string_advanced(10,30,20,"task1_2")

    img2.draw_rectangle(0,120,80,80)
    img2.draw_string_advanced(10,150,20,"task1_3")

    img2.draw_rectangle(0,240,80,80)
    img2.draw_string_advanced(10,270,20,"task2_1")

    img2.draw_rectangle(0,400,80,80)
    img2.draw_string_advanced(10,430,20,"task2_3")

    tim.init(mode=Timer.PERIODIC,period=750,callback=tim_cb)

image_shape = [sensor.height(), sensor.width()]
#creat_ui()
servo_reset()
count = 0
count_flag = 0
while True:
    img = sensor.snapshot(chn=CAM_CHN_ID_0)
    if count_flag == 0:
        if count < 4:
            count += 1
            continue
        else:
            count = 0
            count_flag = 1

    task_function = tasks.get(task)
    task_function(img)

    img.draw_cross(img_center_x,img_center_y,color=(0,0,0),size=4,thickness=4)

#    Display.show_image(img2, layer = Display.LAYER_OSD2, alpha = 128)
    Display.show_image(img,x=int((DISPLAY_WIDTH - picture_w)/2),y=int((DISPLAY_HEIGHT - picture_h)/2))
