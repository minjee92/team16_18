"""미션 층: "무엇을 처리해야 하는가".

GUI 의 MissionRequest 를 검증해서 실행 가능한 Task 로 바꿔 조정 층에 넘기고,
조정 층이 올려주는 TaskState 를 미션 상태(/fleet/mission_state)로 다시 내보낸다.
어떤 로봇이 수행할지, 어떻게 이동할지는 이 노드가 모른다.
"""
import rclpy
from rclpy.node import Node

from pinky_fms_interfaces.msg import MissionRequest, Task, TaskState

KNOWN_TYPES = ('NAV_GOTO', 'CANCEL')


class FleetMission(Node):
    def __init__(self):
        super().__init__('fleet_mission')
        self.missions = {}   # mission_id -> 마지막 TaskState

        self.create_subscription(MissionRequest, '/fleet/mission_request', self.on_request, 10)
        self.create_subscription(TaskState, '/fleet/task_state', self.on_task_state, 10)
        self.task_pub = self.create_publisher(Task, '/fleet/task', 10)
        self.state_pub = self.create_publisher(TaskState, '/fleet/mission_state', 10)
        self.get_logger().info('fleet_mission ready')

    def _reject(self, req, reason):
        st = TaskState(task_id='', mission_id=req.mission_id, robot_id=req.robot_id,
                       state='FAILED', message=reason)
        self.get_logger().warn(f'mission {req.mission_id} rejected: {reason}')
        self.state_pub.publish(st)

    def on_request(self, req):
        if not req.mission_id:
            return self._reject(req, 'mission_id 가 비어 있음')
        if req.type not in KNOWN_TYPES:
            return self._reject(req, f'알 수 없는 type: {req.type}')
        if req.type == 'NAV_GOTO' and req.goal.header.frame_id != 'map':
            return self._reject(req, "goal.header.frame_id 는 'map' 이어야 함")

        task = Task(task_id=f'{req.mission_id}-1', mission_id=req.mission_id, type=req.type,
                    robot_id=req.robot_id, goal=req.goal)
        self.get_logger().info(f'mission {req.mission_id} -> task {task.task_id} ({req.type})')
        self.task_pub.publish(task)

    def on_task_state(self, st):
        self.missions[st.mission_id] = st
        self.state_pub.publish(st)


def main():
    rclpy.init()
    node = FleetMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
