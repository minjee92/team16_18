"""robots.yaml 로더. 로봇 ID 가 곧 namespace 이고, 로봇 이름은 코드에 쓰지 않는다."""
import yaml


def load_registry(path):
    with open(path, 'r', encoding='utf-8') as f:
        cfg = yaml.safe_load(f) or {}
    cfg['robots'] = {str(rid): (r or {}) for rid, r in (cfg.get('robots') or {}).items()}
    cfg.setdefault('heartbeat_timeout_sec', 3.0)
    return cfg
