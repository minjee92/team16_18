#!/usr/bin/env python3
"""관제PC(fleet_traffic)가 보내는 주행 상태를 로봇 LCD 에 크게 띄운다.

  구독: /<ns>/fms_status (std_msgs/String, JSON {"robot", "state", "detail", "dist": 최종 목표까지 남은 경로 m 또는 null})
        /<ns>/lane_status (Lane Following 미션 노드, 같은 형식) — 3초 안에 받은 것이 있으면 이쪽을 우선 표시
  상태: DRIVING / OWN DRIVING PRIORITY / WAITING / RESUME DRIVING / ARRIVED / FAILED / STOPPED / IDLE

LCD 는 pinky_emotion 의 LCD 드라이버(240x320 SPI)를 쓴다. 한 프로세스만 LCD 를 쓸 수 있으므로 emotion 서버와 같이 띄우지 않는다.
LCD 를 열 수 없으면(드라이버 없음·권한 등) 경고만 남기고 상태 로그만 찍는다.
PC 에서 화면 미리보기:  python3 fms_lcd_status.py --preview <출력 폴더>
"""
import json
import os
import sys
import time

from PIL import Image, ImageDraw, ImageFont

W, H = 320, 240           # 가로 화면 기준으로 그리고, LCD 드라이버가 세로(240x320)로 돌려 보낸다
BAR = 40                  # 위쪽 로봇 이름 띠
FONT_PATH = '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'

# 상태 -> (배경색, 글자색)
STYLE = {
    'DRIVING': ('#1d4ed8', '#ffffff'),
    'OWN DRIVING PRIORITY': ('#15803d', '#ffffff'),
    'WAITING': ('#f59e0b', '#111827'),
    'RESUME DRIVING': ('#0891b2', '#ffffff'),
    'RETURNING': ('#0d9488', '#ffffff'),
    'ARRIVED': ('#7c3aed', '#ffffff'),
    'FAILED': ('#b91c1c', '#ffffff'),
    'STOPPED': ('#475569', '#ffffff'),
    'IDLE': ('#1e293b', '#cbd5e1'),
    'FMS STANDBY': ('#1e293b', '#64748b'),
}


def _font(size):
    try:
        return ImageFont.truetype(FONT_PATH, size)
    except OSError:
        return ImageFont.load_default(size=size)


def _wrap(draw, text, font, max_w):
    lines, cur = [], ''
    for word in text.split():
        trial = f'{cur} {word}'.strip()
        if draw.textlength(trial, font=font) <= max_w or not cur:
            cur = trial
        else:
            lines.append(cur)
            cur = word
    if cur:
        lines.append(cur)
    return lines


def _fit(draw, text, max_w, max_h, sizes):
    """가장 큰 글자 크기로 줄바꿈해서 영역에 맞춘다"""
    for size in sizes:
        font = _font(size)
        lines = _wrap(draw, text, font, max_w)
        lh = int(size * 1.15)
        if all(draw.textlength(ln, font=font) <= max_w for ln in lines) and lh * len(lines) <= max_h:
            return font, lines, lh
    font = _font(sizes[-1])
    return font, _wrap(draw, text, font, max_w), int(sizes[-1] * 1.15)


def render(robot, state, detail='', dist=None):
    bg, fg = STYLE.get(state, ('#334155', '#ffffff'))
    img = Image.new('RGB', (W, H), bg)
    d = ImageDraw.Draw(img)
    # 위쪽 띠: 로봇 이름
    d.rectangle([0, 0, W, BAR], fill='#0f172a')
    d.text((12, BAR // 2), robot.upper().replace('_', '-'), font=_font(24), fill='#e2e8f0', anchor='lm')
    if dist is None:
        d.text((W - 12, BAR // 2), 'FMS', font=_font(16), fill='#64748b', anchor='rm')
    else:       # 목적지까지 남은 거리
        d.text((W - 12, BAR // 2), f'{dist:.1f} m', font=_font(24), fill='#fde047', anchor='rm')
        d.text((W - 12 - d.textlength(f'{dist:.1f} m', font=_font(24)) - 8, BAR // 2 + 1), 'GOAL', font=_font(14), fill='#94a3b8', anchor='rm')
    # 가운데: 상태 (크게), 아래: 상세 (작게)
    detail_h = 34 if detail else 0
    area_h = H - BAR - detail_h - 16
    font, lines, lh = _fit(d, state, W - 24, area_h, (58, 52, 46, 40, 36, 32, 28, 24))
    y = BAR + 8 + (area_h - lh * len(lines)) // 2 + lh // 2
    for ln in lines:
        d.text((W // 2, y), ln, font=font, fill=fg, anchor='mm')
        y += lh
    if detail:
        dfont, dlines, _ = _fit(d, detail, W - 24, detail_h, (22, 20, 18, 16))
        d.text((W // 2, H - 8 - detail_h // 2), dlines[0], font=dfont, fill=fg, anchor='mm')
    return img


def preview(out_dir):
    os.makedirs(out_dir, exist_ok=True)
    samples = [('DRIVING', '', 2.4), ('OWN DRIVING PRIORITY', 'amr_02 yielding', 1.8), ('WAITING', 'yield to amr_01', 3.1),
               ('RESUME DRIVING', '', 0.7), ('ARRIVED', '', None), ('FAILED', '', None), ('STOPPED', '', None), ('IDLE', '', None),
               ('FMS STANDBY', '', None)]
    for state, detail, dist in samples:
        path = os.path.join(out_dir, state.lower().replace(' ', '_') + '.png')
        render('amr_01', state, detail, dist).save(path)
        print(path)


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from std_msgs.msg import String

    class FmsLcdStatus(Node):
        def __init__(self):
            super().__init__('fms_lcd_status')
            ns = self.get_namespace().strip('/')
            self.robot = self.declare_parameter('robot_name', ns or 'robot').value
            self.lcd = None
            try:
                from pinky_emotion.pinky_lcd import LCD
                self.lcd = LCD()
            except Exception as e:      # 드라이버가 없거나(PC) SPI·GPIO 를 열 수 없음
                self.get_logger().warn(f'LCD 를 열 수 없어 화면 표시 없이 상태만 기록합니다: {e}')
            self.shown = None
            self.lane_t = 0.0           # Lane Following 미션 상태를 마지막으로 받은 시각
            self.create_subscription(String, 'fms_status', self.on_status,
                                     QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.create_subscription(String, 'lane_status', self.on_lane_status, 10)
            self.show('FMS STANDBY', '')

        def on_lane_status(self, msg):
            self.lane_t = time.monotonic()
            self.on_status(msg, lane=True)

        def on_status(self, msg, lane=False):
            if not lane and time.monotonic() - self.lane_t < 3.0:
                return                  # Lane Following 미션 중에는 그 노드의 상태를 띄운다
            try:
                m = json.loads(msg.data)
            except ValueError:
                return
            dist = m.get('dist')
            self.show(str(m.get('state', '')), str(m.get('detail', '')), float(dist) if isinstance(dist, (int, float)) else None)

        def show(self, state, detail, dist=None):
            if (state, detail, dist) == self.shown:      # 같은 화면은 다시 보내지 않는다 (SPI 전송이 무겁다)
                return
            if self.shown is None or (state, detail) != self.shown[:2]:     # 거리만 바뀐 것은 로그에 남기지 않는다
                self.get_logger().info(f'LCD: {state}' + (f' ({detail})' if detail else ''))
            self.shown = (state, detail, dist)
            if self.lcd is None:
                return
            try:
                self.lcd.img_show(render(self.robot, state, detail, dist))
            except Exception as e:
                self.get_logger().warn(f'LCD 출력 실패: {e}', throttle_duration_sec=5.0)

        def close(self):
            if self.lcd is not None:
                try:
                    self.lcd.clear()
                    self.lcd.close()
                except Exception:
                    pass

    rclpy.init()
    node = FmsLcdStatus()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    if len(sys.argv) >= 3 and sys.argv[1] == '--preview':
        preview(sys.argv[2])
    else:
        main()
