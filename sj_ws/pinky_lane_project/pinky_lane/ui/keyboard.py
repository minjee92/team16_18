import os, sys, select, termios, tty

class KeyboardStop:
    ENTER_KEYS = ("\r", "\n")
    ESC = "\x1b"

    def __init__(self) -> None:
        self.enabled = sys.stdin.isatty()
        self.fd = sys.stdin.fileno() if self.enabled else None
        self.old_settings = None

    def __enter__(self):
        if self.enabled:
            self.old_settings = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        if self.enabled and self.old_settings is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)
        return False

    def _readable(self, timeout: float = 0.0) -> bool:
        if not self.enabled:
            return False
        return bool(select.select([sys.stdin], [], [], timeout)[0])

    def requested(self) -> bool:
        if not self.enabled:
            return False

        stop = False
        while self._readable():
            try:
                char = os.read(self.fd, 1).decode("utf-8", errors="ignore")
            except OSError:
                break

            if not char:
                break

            if char in self.ENTER_KEYS:
                stop = True
                continue

            if char == self.ESC:
                # 방향키 escape sequence는 종료로 오인하지 않는다.
                if self._readable(timeout=0.02):
                    while self._readable():
                        os.read(self.fd, 1)
                    continue
                stop = True

        return stop
