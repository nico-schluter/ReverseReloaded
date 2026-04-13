from .fitSurfaces import entry as fitSurfaces
from .. import config

commands = [fitSurfaces]

if config.DEBUG:
    from .addTestMeshes import entry as addTestMeshes
    commands.append(addTestMeshes)


def start():
    for command in commands:
        command.start()


def stop():
    for command in commands:
        command.stop()
