from .fitSurfaces import entry as fitSurfaces

commands = [fitSurfaces]


def start():
    for command in commands:
        command.start()


def stop():
    for command in commands:
        command.stop()
