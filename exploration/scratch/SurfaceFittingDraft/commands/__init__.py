from .fitSurfaces import entry as fitSurfaces

_commands = [fitSurfaces]


def start():
    for cmd in _commands:
        cmd.start()


def stop():
    for cmd in _commands:
        cmd.stop()
