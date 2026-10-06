"""DDS 탐색 방식(멀티캐스트 / 유니캐스트) 설정 생성기. ROS 에 의존하지 않는다.

Wi-Fi 에서는 멀티캐스트 탐색이 불안정해서 다중 로봇일 때 통신이 자주 끊긴다. unicast 로 바꾸면
  - 로봇: 멀티캐스트를 끄고 관제PC IP 만 피어로 지정 (로봇끼리는 서로 탐색하지 않아 Wi-Fi 부하도 줄어든다)
  - 관제PC: 멀티캐스트를 끄고 인터페이스를 고정, 자기 자신(localhost)만 피어로 지정 (로봇 IP 는 몰라도 된다: 로봇이 먼저 찾아온다)
RMW 는 rmw_cyclonedds_cpp 를 쓴다 (관제PC 가 이미 사용 중).

CLI (관제PC 쪽 환경변수를 출력하므로 eval 로 적용):
  eval "$(python3 netconf.py pc-env /path/to/robots.yaml)"
"""
import fcntl
import socket
import struct
import sys
from pathlib import Path
from xml.sax.saxutils import quoteattr, escape

import yaml

RMW = 'rmw_cyclonedds_cpp'


def network_cfg(cfg):
    n = (cfg or {}).get('network') or {}
    mode = str(n.get('discovery', 'multicast')).lower()
    if mode not in ('multicast', 'unicast'):
        raise ValueError(f"network.discovery 는 multicast 또는 unicast 여야 함: {mode}")
    return {
        'discovery': mode,
        'interface': str(n.get('interface') or '').strip(),
        'max_participants': int(n.get('max_participants', 100)),
        'extra_peers': [str(p) for p in (n.get('extra_peers') or [])],
    }


def iface_ip(name_or_ip):
    """인터페이스 이름(wlp0s20f3)이면 IPv4 주소로, 이미 IP 면 그대로. 못 찾으면 ''."""
    if not name_or_ip:
        return ''
    try:
        socket.inet_aton(name_or_ip)
        return name_or_ip
    except OSError:
        pass
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        packed = fcntl.ioctl(s.fileno(), 0x8915, struct.pack('256s', name_or_ip[:15].encode()))   # SIOCGIFADDR
        return socket.inet_ntoa(packed[20:24])
    except OSError:
        return ''
    finally:
        s.close()


def _xml(multicast, peers, max_participants, interface=''):
    general = f"<AllowMulticast>{'true' if multicast else 'false'}</AllowMulticast>"
    if interface:
        # 이름(wlp0s20f3)이면 name=, IP 면 address= (NetworkInterfaceAddress 는 deprecated)
        attr = 'address' if iface_ip(interface) == interface else 'name'
        general += f'<Interfaces><NetworkInterface {attr}={quoteattr(interface)}/></Interfaces>'
    peer_xml = ''.join(f'<Peer address={quoteattr(p)}/>' for p in peers)
    disc = ('<ParticipantIndex>auto</ParticipantIndex>'
            f'<MaxAutoParticipantIndex>{int(max_participants)}</MaxAutoParticipantIndex>')
    if peer_xml:
        disc += f'<Peers>{peer_xml}</Peers>'
    return f'<CycloneDDS><Domain><General>{general}</General><Discovery>{disc}</Discovery></Domain></CycloneDDS>'


def robot_xml(net, pc_ip, own_ip=''):
    """로봇에 쓸 설정: 멀티캐스트 끔 + 관제PC 와 로봇 자신을 피어로.

    멀티캐스트를 끄면 로봇 안의 프로세스끼리(bringup, Nav2, TF)도 서로 찾지 못하므로 자기 자신도 피어로 넣는다.
    own_ip 는 관제PC 가 이 로봇에 접속한 IP(ap0 등). 로봇이 Wi-Fi 인터페이스를 여러 개 가질 수 있어서(wlan0 + ap0)
    그 인터페이스로 고정한다 (실물 Pinky: 로봇 안 프로세스 간 탐색 14초 실패 -> 2초 성공 확인).
    own_ip 가 비어 있으면(local 데모) 고정하지 않고 localhost 만 쓴다.
    """
    own = ['localhost'] + ([own_ip] if own_ip else [])
    peers = list(dict.fromkeys([pc_ip] + own))
    return _xml(False, peers, net['max_participants'], own_ip)


def pc_xml(net):
    """관제PC 설정: 멀티캐스트 끔 + 인터페이스 고정.

    멀티캐스트를 끄면 같은 PC 안의 프로세스끼리(조정 노드, rosbridge, ros2 CLI)도 서로 찾지 못하므로
    자기 자신(localhost, 고정한 인터페이스의 IP)을 피어로 넣는다. 로봇 IP 는 로봇이 먼저 찾아오므로 필요 없다.
    """
    own = ['localhost']
    ip = iface_ip(net['interface'])
    if ip:
        own.append(ip)
    peers = list(dict.fromkeys(own + net['extra_peers']))
    return _xml(False, peers, net['max_participants'], net['interface'])


def pc_env(cfg):
    net = network_cfg(cfg)
    if net['discovery'] != 'unicast':
        return ''
    xml = pc_xml(net).replace("'", "'\\''")
    return f"export RMW_IMPLEMENTATION={RMW}\nexport CYCLONEDDS_URI='{xml}'\n"


if __name__ == '__main__':
    if len(sys.argv) == 3 and sys.argv[1] == 'pc-env':
        sys.stdout.write(pc_env(yaml.safe_load(Path(sys.argv[2]).read_text(encoding='utf-8'))))
    else:
        sys.exit(__doc__)
