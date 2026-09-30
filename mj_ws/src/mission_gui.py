#!/usr/bin/env python3
"""
Pinky 미션 GUI

SLAM으로 만든 맵(pgm + yaml)을 불러와 목표 지점을 지정한다.
Nav2에 주행을 맡기지 않고, 차선 추종 노드가 참고할 목표만 넘긴다.

  발행:
    /mission/goal_pose   (PoseStamped)            목표 지점
    /mission/enable      (Bool)                   주행 시작/정지
    /initialpose         (PoseWithCovarianceStamped)  AMCL 초기 위치

  사용:
    python3 src/mission_gui.py maps/mission4_3.yaml
    python3 src/mission_gui.py ~/ros2_ws/src/pinky_pro/pinky_navigation/map/my_map.yaml

  조작:
    맵 위에서 드래그  → 위치와 방향 지정 (짧게 클릭하면 방향은 현재 유지)
    모드 전환으로 AMCL 초기 위치도 같은 방식으로 지정

주의:
    Nav2 전체(bringup_launch)를 띄우면 cmd_vel이 충돌한다.
    localization_launch만 실행할 것.
"""

import math
import os
import sys
import threading

import numpy as np
import tkinter as tk
from tkinter import ttk

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from PIL import Image, ImageTk
from rclpy.node import Node
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener


# ----------------------------- 설정 -----------------------------

MAP_FRAME = 'map'
ROBOT_FRAME = 'base_footprint'

CANVAS_MAX = 720          # 캔버스 한 변의 최대 픽셀
POSE_HZ = 10              # 로봇 위치 갱신 주기


# --------------------------- 맵 읽기 ---------------------------

class MapData:
    """map_server와 동일한 규칙으로 pgm/yaml을 해석한다."""

    def __init__(self, yaml_path):
        yaml_path = os.path.abspath(os.path.expanduser(yaml_path))
        folder = os.path.dirname(yaml_path)

        with open(yaml_path) as f:
            meta = yaml.safe_load(f)

        self.resolution = float(meta['resolution'])
        origin = meta['origin']
        self.origin_x = float(origin[0])
        self.origin_y = float(origin[1])
        self.origin_yaw = float(origin[2]) if len(origin) > 2 else 0.0

        image_path = meta['image']
        if not os.path.isabs(image_path):
            image_path = os.path.join(folder, image_path)

        self.image = Image.open(image_path).convert('L')
        self.width, self.height = self.image.size

        if abs(self.origin_yaw) > 1e-6:
            print('[경고] 맵 origin에 회전이 있습니다. 이 GUI는 회전을 지원하지 않습니다.')

    # --- 좌표 변환 ---

    def pixel_to_world(self, col, row):
        x = self.origin_x + col * self.resolution
        y = self.origin_y + (self.height - 1 - row) * self.resolution
        return x, y

    def world_to_pixel(self, x, y):
        col = (x - self.origin_x) / self.resolution
        row = (self.height - 1) - (y - self.origin_y) / self.resolution
        return col, row

    def describe(self):
        return (f'{self.width}x{self.height} px, {self.resolution} m/px '
                f'→ 실제 {self.width * self.resolution:.2f} x '
                f'{self.height * self.resolution:.2f} m')


# --------------------------- ROS 노드 ---------------------------

class MissionNode(Node):

    def __init__(self):
        super().__init__('mission_gui')

        self.goal_pub = self.create_publisher(
            PoseStamped, '/mission/goal_pose', 10)
        self.enable_pub = self.create_publisher(
            Bool, '/mission/enable', 10)
        self.initial_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def robot_pose(self):
        """map 기준 로봇 위치. 없으면 None."""
        try:
            tf = self.tf_buffer.lookup_transform(
                MAP_FRAME, ROBOT_FRAME, rclpy.time.Time())
        except Exception:
            return None

        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return t.x, t.y, yaw

    def send_goal(self, x, y, yaw):
        msg = PoseStamped()
        msg.header.frame_id = MAP_FRAME
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        self.goal_pub.publish(msg)
        self.get_logger().info(f'목표 전송: ({x:.2f}, {y:.2f}) yaw {math.degrees(yaw):.0f}°')

    def send_enable(self, value):
        msg = Bool()
        msg.data = bool(value)
        self.enable_pub.publish(msg)
        self.get_logger().info('주행 시작' if value else '주행 정지')

    def send_initial_pose(self, x, y, yaw):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = MAP_FRAME
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        cov = [0.0] * 36
        cov[0] = 0.25      # x
        cov[7] = 0.25      # y
        cov[35] = 0.068    # yaw
        msg.pose.covariance = cov

        self.initial_pub.publish(msg)
        self.get_logger().info(f'초기 위치 설정: ({x:.2f}, {y:.2f})')


# ----------------------------- GUI -----------------------------

class MissionGUI:

    def __init__(self, root, node, map_data):
        self.root = root
        self.node = node
        self.map = map_data

        self.scale = max(1, min(CANVAS_MAX // map_data.width,
                                CANVAS_MAX // map_data.height))
        self.view_w = map_data.width * self.scale
        self.view_h = map_data.height * self.scale

        self.click_mode = tk.StringVar(value='goal')
        self.goal = None           # (x, y, yaw)
        self.drag_start = None     # (col, row)

        self._build()
        self._draw_map()
        self._tick()

    # ---------- 화면 구성 ----------

    def _build(self):
        self.root.title('Pinky 미션 GUI')

        top = ttk.Frame(self.root, padding=8)
        top.pack(fill='x')

        ttk.Radiobutton(top, text='목표 지점', value='goal',
                        variable=self.click_mode).pack(side='left')
        ttk.Radiobutton(top, text='초기 위치(AMCL)', value='initial',
                        variable=self.click_mode).pack(side='left', padx=(8, 20))

        ttk.Button(top, text='주행 시작',
                   command=lambda: self.node.send_enable(True)).pack(side='left')
        ttk.Button(top, text='정지',
                   command=lambda: self.node.send_enable(False)).pack(side='left', padx=6)
        ttk.Button(top, text='목표 지우기',
                   command=self._clear_goal).pack(side='left')

        self.canvas = tk.Canvas(self.root, width=self.view_w, height=self.view_h,
                                bg='#202020', highlightthickness=0)
        self.canvas.pack(padx=8)
        self.canvas.bind('<Button-1>', self._on_press)
        self.canvas.bind('<B1-Motion>', self._on_drag)
        self.canvas.bind('<ButtonRelease-1>', self._on_release)
        self.canvas.bind('<Motion>', self._on_move)

        bottom = ttk.Frame(self.root, padding=8)
        bottom.pack(fill='x')

        self.status = tk.StringVar(value='맵 로딩 완료')
        ttk.Label(bottom, textvariable=self.status,
                  font=('TkDefaultFont', 10)).pack(anchor='w')

        self.info = tk.StringVar(value=self.map.describe())
        ttk.Label(bottom, textvariable=self.info,
                  foreground='#666').pack(anchor='w')

    def _draw_map(self):
        image = self.map.image.resize(
            (self.view_w, self.view_h), Image.NEAREST)
        self.photo = ImageTk.PhotoImage(image)
        self.canvas.create_image(0, 0, anchor='nw', image=self.photo,
                                 tags='mapimg')

    # ---------- 좌표 변환 ----------

    def _canvas_to_pixel(self, cx, cy):
        return cx / self.scale, cy / self.scale

    def _world_to_canvas(self, x, y):
        col, row = self.map.world_to_pixel(x, y)
        return col * self.scale, row * self.scale

    # ---------- 마우스 ----------

    def _on_press(self, event):
        self.drag_start = (event.x, event.y)

    def _on_drag(self, event):
        if self.drag_start is None:
            return
        self.canvas.delete('preview')
        x0, y0 = self.drag_start
        self.canvas.create_line(x0, y0, event.x, event.y,
                                fill='#ff9800', width=3,
                                arrow='last', tags='preview')

    def _on_release(self, event):
        if self.drag_start is None:
            return

        self.canvas.delete('preview')
        x0, y0 = self.drag_start
        self.drag_start = None

        col, row = self._canvas_to_pixel(x0, y0)
        wx, wy = self.map.pixel_to_world(col, row)

        dx = event.x - x0
        dy = event.y - y0

        if math.hypot(dx, dy) < 8:
            yaw = self.goal[2] if self.goal else 0.0
        else:
            # 캔버스 y축은 아래로 증가하므로 부호를 뒤집는다
            yaw = math.atan2(-dy, dx)

        if self.click_mode.get() == 'goal':
            self.goal = (wx, wy, yaw)
            self.node.send_goal(wx, wy, yaw)
            self.status.set(f'목표 전송: ({wx:.2f}, {wy:.2f})  '
                            f'{math.degrees(yaw):.0f}°')
        else:
            self.node.send_initial_pose(wx, wy, yaw)
            self.status.set(f'초기 위치 설정: ({wx:.2f}, {wy:.2f})')

    def _on_move(self, event):
        col, row = self._canvas_to_pixel(event.x, event.y)
        wx, wy = self.map.pixel_to_world(col, row)
        self.info.set(f'{self.map.describe()}   |   커서 ({wx:.2f}, {wy:.2f})')

    def _clear_goal(self):
        self.goal = None
        self.status.set('목표 지움 (차선 추종만 계속)')

    # ---------- 주기 갱신 ----------

    def _tick(self):
        self.canvas.delete('overlay')

        # 목표 표시
        if self.goal:
            gx, gy, gyaw = self.goal
            cx, cy = self._world_to_canvas(gx, gy)
            r = 9
            self.canvas.create_oval(cx - r, cy - r, cx + r, cy + r,
                                    outline='#e53935', width=3, tags='overlay')
            self.canvas.create_line(
                cx, cy,
                cx + 26 * math.cos(gyaw), cy - 26 * math.sin(gyaw),
                fill='#e53935', width=3, arrow='last', tags='overlay')

        # 로봇 표시
        pose = self.node.robot_pose()
        if pose:
            rx, ry, ryaw = pose
            cx, cy = self._world_to_canvas(rx, ry)
            r = 8
            self.canvas.create_oval(cx - r, cy - r, cx + r, cy + r,
                                    fill='#1e88e5', outline='white',
                                    width=2, tags='overlay')
            self.canvas.create_line(
                cx, cy,
                cx + 24 * math.cos(ryaw), cy - 24 * math.sin(ryaw),
                fill='#1e88e5', width=3, arrow='last', tags='overlay')

            if self.goal:
                dist = math.hypot(self.goal[0] - rx, self.goal[1] - ry)
                self.status.set(
                    f'로봇 ({rx:.2f}, {ry:.2f}) {math.degrees(ryaw):+.0f}°   |   '
                    f'목표까지 {dist:.2f} m')
        else:
            if not self.goal:
                self.status.set('로봇 위치 없음 — AMCL이 실행 중인지 확인하세요')

        self.root.after(int(1000 / POSE_HZ), self._tick)


# ----------------------------- 메인 -----------------------------

def main():
    if len(sys.argv) < 2:
        print('사용법: python3 mission_gui.py <맵 yaml 경로>')
        return

    map_data = MapData(sys.argv[1])
    print('맵 로딩:', map_data.describe())

    rclpy.init()
    node = MissionNode()

    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    root = tk.Tk()
    gui = MissionGUI(root, node, map_data)

    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.send_enable(False)     # 창을 닫으면 반드시 정지
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
