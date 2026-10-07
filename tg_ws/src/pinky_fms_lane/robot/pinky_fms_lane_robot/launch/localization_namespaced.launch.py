"""pinky_navigation/localization_launch.xml(map_server + amcl + lifecycle 관리자)만 namespace 안에서 실행하는 내부 launch.
robot_localization.launch.xml 이 부른다. Nav2 주행 노드(controller_server 등)를 띄우지 않으므로 cmd_vel 을 내는 노드가 없다.

팀원 pinky_fms_bringup/launch/nav2_namespaced.launch.py 와 같은 방법으로 namespace 문제를 맞춘다 (원본 파일은 고치지 않는다).
  1. 안쪽 launch 에는 namespace='' 를 넘기고, 바깥 PushRosNamespace 가 한 번만 적용한다
  2. 파라미터 노드 키를 전체 이름(/amr_01/amcl)으로 바꾼 임시 파일을 쓴다 (짧은 키 amcl: 는 namespace 노드에 적용되지 않음)
  3. 시뮬레이션처럼 TF 프레임에 접두어(amr_01/)가 붙으면 프레임 파라미터에도 접두어를 붙인다
파라미터: params_file(기본 팀원 nav2_params.yaml)에서 amcl 부분만 쓰고 → overrides_file 로 일부를 덮어쓰고 → scan_topic 을 적용한다.
scan_topic 이 scan_robotfree 이면 팀원 robot_scan_filter(다른 로봇을 지운 스캔)를 같이 띄운다.
"""
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

LOCALIZATION_NODES = ('amcl', 'map_server')     # params_file 에서 가져올 노드 (없는 것은 건너뜀)
ROBOTFREE_SCAN = 'scan_robotfree'


# ---- 아래 함수 4개와 FRAME_KEYS 는 팀원 코드를 그대로 복사했다 (팀원 파일에 의존하지 않으려고) ----
# 원본: tg_ws/src/pinky_fms/robot/pinky_fms_bringup/launch/nav2_namespaced.launch.py (커밋 acc6fb1)
def qualify_topics(node, ns):
    """파라미터 값 안의 'topic' 항목(costmap observation source)을 /<ns>/<이름> 으로 바꾼다."""
    if isinstance(node, dict):
        for key, val in node.items():
            if key == 'topic' and isinstance(val, str) and ns:
                node[key] = f"/{ns}/{val.lstrip('/')}"
            else:
                qualify_topics(val, ns)


def fqn_params(data, ns):
    """{amcl: {ros__parameters: ...}, local_costmap: {local_costmap: {ros__parameters: ...}}} 를
    {/ns/amcl: {...}, /ns/local_costmap/local_costmap: {...}} 로 바꾼다."""
    out = {}

    def walk(node, path):
        for key, val in node.items():
            if key == 'ros__parameters':
                out['/' + '/'.join(path)] = {'ros__parameters': val}
            elif isinstance(val, dict):
                walk(val, path + [str(key).strip('/')])

    walk(data, [ns] if ns else [])
    qualify_topics(out, ns)
    return out


FRAME_KEYS = ('base_frame_id', 'odom_frame_id', 'robot_base_frame', 'local_frame')     # 로봇 몸체·odom 프레임 (map 은 그대로)


def prefix_frames(node, prefix):
    """시뮬레이션처럼 TF 프레임에 접두어(amr_01/)가 붙는 경우: 프레임 파라미터에 접두어를 붙인다. 'map' 은 공유 프레임이라 제외."""
    if isinstance(node, dict):
        for key, val in node.items():
            if key in FRAME_KEYS and isinstance(val, str) and val != 'map':
                node[key] = prefix + val
            elif key == 'global_frame' and val == 'odom':
                node[key] = prefix + val
            else:
                prefix_frames(val, prefix)


def set_sim_time(node):
    if isinstance(node, dict):
        if 'ros__parameters' in node and isinstance(node['ros__parameters'], dict):
            node['ros__parameters']['use_sim_time'] = True
        for val in node.values():
            set_sim_time(val)
# ---- 복사 끝 ----


def merge(base, extra):
    """extra 의 값을 base 에 깊이 덮어쓴다 (dict 는 안쪽까지)."""
    for key, val in (extra or {}).items():
        if isinstance(val, dict) and isinstance(base.get(key), dict):
            merge(base[key], val)
        else:
            base[key] = val
    return base


def launch_setup(context):
    ns = LaunchConfiguration('namespace').perform(context).strip('/')
    map_yaml = LaunchConfiguration('map').perform(context)
    src = LaunchConfiguration('params_file').perform(context)
    overrides = LaunchConfiguration('overrides_file').perform(context)
    scan_topic = LaunchConfiguration('scan_topic').perform(context).strip('/') or 'scan'
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() in ('true', '1')
    frame_prefix = LaunchConfiguration('frame_prefix').perform(context)
    if not map_yaml:
        raise RuntimeError('map:=<지도.yaml> 을 지정하세요 (예: map:=$HOME/mission4_3_clean_1cm.yaml)')

    with open(src, 'r', encoding='utf-8') as f:
        data = {k: v for k, v in (yaml.safe_load(f) or {}).items() if k in LOCALIZATION_NODES}
    if overrides:
        with open(overrides, 'r', encoding='utf-8') as f:
            merge(data, yaml.safe_load(f))
    data.setdefault('amcl', {}).setdefault('ros__parameters', {})['scan_topic'] = scan_topic
    out = fqn_params(data, ns)
    if frame_prefix:
        prefix_frames(out, frame_prefix)
    if use_sim_time:
        set_sim_time(out)
    tmp = tempfile.NamedTemporaryFile('w', suffix='_localization_params.yaml', delete=False, encoding='utf-8')
    yaml.safe_dump(out, tmp, allow_unicode=True, sort_keys=False)
    tmp.close()

    nav_share = get_package_share_directory('pinky_navigation')
    actions = [IncludeLaunchDescription(
        AnyLaunchDescriptionSource(os.path.join(nav_share, 'launch', 'localization_launch.xml')),
        launch_arguments={
            'namespace': '',                       # namespace 는 아래 PushRosNamespace 가 한 번만 적용한다
            'map': map_yaml,
            'params_file': tmp.name,
            'use_composition': 'False',            # 컨테이너가 없으므로 노드를 따로 띄운다
            'use_sim_time': 'True' if use_sim_time else 'False',
        }.items(),
    )]
    if scan_topic == ROBOTFREE_SCAN:               # 팀원 robot_nav.launch.xml 과 같은 방식으로 필터를 띄운다
        actions.append(Node(package='pinky_fms_bringup', executable='robot_scan_filter.py', name='robot_scan_filter',
                            output='screen', parameters=[{'use_sim_time': use_sim_time}],
                            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]))
    if ns:
        actions.insert(0, PushRosNamespace(ns))
    return [GroupAction(actions)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='amr_01'),
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('params_file', default_value=os.path.join(
            get_package_share_directory('pinky_fms_bringup'), 'params', 'nav2_params.yaml')),
        DeclareLaunchArgument('overrides_file', default_value=os.path.join(
            get_package_share_directory('pinky_fms_lane_robot'), 'params', 'localization_lane.yaml')),
        DeclareLaunchArgument('scan_topic', default_value='scan', description=f"AMCL 이 쓸 스캔: scan | {ROBOTFREE_SCAN}"),
        DeclareLaunchArgument('use_sim_time', default_value='False'),
        DeclareLaunchArgument('frame_prefix', default_value='', description="시뮬레이션처럼 TF 프레임에 접두어가 붙을 때 (예: 'amr_01/')"),
        OpaqueFunction(function=launch_setup),
    ])
