import threading, time
from typing import Optional
import rclpy
from rclpy.executors import MultiThreadedExecutor
from pinky_lane.config import DRIVE_OUTPUT, MISSION, MOTION
from pinky_lane.ros.node import LaneMissionController
from pinky_lane.ui.keyboard import KeyboardStop

def main(args=None) -> None:
    rclpy.init(args=args)

    node: Optional[LaneMissionController] = None
    executor: Optional[MultiThreadedExecutor] = None
    spin_thread: Optional[threading.Thread] = None

    period = 1.0 / MOTION.control_hz

    try:
        node = LaneMissionController()
        node.start_hardware()

        # ROS subscription/TF 처리를 추론 loop와 분리한다.
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()

        print()
        print("=" * 64)
        print(f"mode            : {'DRIVE' if DRIVE_OUTPUT else 'DRY RUN'}")
        print(f"control rate    : {MOTION.control_hz:.1f} Hz")
        print(f"base speed      : {MOTION.base_speed:.2f} m/s")
        print(f"left turn       : {'allowed' if MISSION.allow_left_turn else 'disabled'}")
        print("start/stop      : mission GUI")
        print("quit            : Enter or ESC")
        print("=" * 64)
        print()

        with KeyboardStop() as keyboard:
            deadline = time.monotonic()

            while rclpy.ok():
                if keyboard.requested():
                    print("\nstop requested")
                    break

                node.step()

                deadline += period
                remaining = deadline - time.monotonic()
                if remaining > 0:
                    time.sleep(remaining)
                else:
                    # 추론이 목표 주기보다 오래 걸렸으면 누적 지연을 버린다.
                    deadline = time.monotonic()

    except KeyboardInterrupt:
        print("\ninterrupted")

    except Exception as exc:
        print(f"error: {exc}")
        import traceback

        traceback.print_exc()

    finally:
        if node is not None:
            node.shutdown()

        if executor is not None:
            try:
                executor.shutdown(timeout_sec=1.0)
            except Exception:
                pass

        if spin_thread is not None:
            spin_thread.join(timeout=1.0)

        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

        print("shutdown complete")
