"""pinky_navigation/bringup_launch.xml 을 namespace 안에서 올바르게 실행하는 내부 launch (robot_nav.launch.xml 이 부른다).

원본은 namespace 로 쓰면 세 가지가 어긋난다. 원본 파일은 고치지 않고 여기서 맞춘다.
  1. 컨테이너에 노드를 올릴 때 컨테이너를 이름만 지정해 namespace 안에서 못 찾는다   -> use_composition=False
  2. 내부 launch 가 같은 이름의 namespace 인자로 한 번 더 적용한다 (/amr_01/amr_01/...) -> 안쪽에는 namespace='' 를 넘긴다
  3. nav2_params.yaml 의 노드 키(amcl:, controller_server: ...)가 /amr_01/amcl 에 매칭되지 않아
     파라미터가 전부 무시된다 (controller_server: No critics defined)
     -> 노드 키를 전체 이름(/amr_01/amcl, /amr_01/local_costmap/local_costmap)으로 바꾼 임시 파일을 만들어 쓴다.
        (실험: 이 ROS 버전은 전체 이름 키만 namespace 노드에 적용되고, 중첩 키(RewrittenYaml 방식)나 /** 는 안 됨)
  4. costmap 의 라이다 구독 토픽(topic: /scan)이 namespace 와 어긋난다.
     costmap 노드는 /amr_01/local_costmap 같은 하위 namespace 에 있어서, 상대 이름 'scan' 은
     /amr_01/local_costmap/scan 이 되고(아무도 발행하지 않음) 절대 이름 '/scan' 은 namespace 를 무시한다.
     -> topic 값을 /amr_01/scan 으로 바꾼다. (실물에서 지역 costmap 의 장애물이 0개였고 로봇이 벽을 무시하고 주행)
기본 params 는 이 패키지의 params/nav2_params.yaml (amr_01 의 튜닝값 기준, 두 로봇 공용)이다. 위 변환(노드 키, topic)은 실행할 때 자동으로 한다.
"""
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace


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


def launch_setup(context):
    ns = LaunchConfiguration('namespace').perform(context).strip('/')
    src = LaunchConfiguration('params_file').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() in ('true', '1')
    frame_prefix = LaunchConfiguration('frame_prefix').perform(context)
    with open(src, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}
    out = fqn_params(data, ns)
    if frame_prefix:
        prefix_frames(out, frame_prefix)
    if use_sim_time:
        set_sim_time(out)
    tmp = tempfile.NamedTemporaryFile('w', suffix='_nav2_params.yaml', delete=False, encoding='utf-8')
    yaml.safe_dump(out, tmp, allow_unicode=True, sort_keys=False)
    tmp.close()

    nav_share = get_package_share_directory('pinky_navigation')
    bringup = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(os.path.join(nav_share, 'launch', 'bringup_launch.xml')),
        launch_arguments={
            'namespace': '',                       # namespace 는 아래 PushRosNamespace 가 한 번만 적용한다
            'map': LaunchConfiguration('map').perform(context),
            'params_file': tmp.name,
            'use_composition': LaunchConfiguration('use_composition').perform(context),
            'use_sim_time': 'True' if use_sim_time else 'False',
        }.items(),
    )
    actions = [bringup]
    if ns:
        actions.insert(0, PushRosNamespace(ns))
    return [GroupAction(actions)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='amr_01'),
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('params_file', default_value=os.path.join(get_package_share_directory('pinky_fms_bringup'), 'params', 'nav2_params.yaml')),
        DeclareLaunchArgument('use_composition', default_value='False'),
        DeclareLaunchArgument('use_sim_time', default_value='False'),
        DeclareLaunchArgument('frame_prefix', default_value='', description="시뮬레이션처럼 TF 프레임에 접두어가 붙을 때 (예: 'amr_01/')"),
        OpaqueFunction(function=launch_setup),
    ])
