import os
import traceback
import adsk.core

app = adsk.core.Application.get()
ui  = app.userInterface

try:
    from ... import config
    DEBUG = config.DEBUG
except:
    DEBUG = False


def log(message: str,
        level: adsk.core.LogLevels = adsk.core.LogLevels.InfoLogLevel,
        force_console: bool = False):
    print(message)
    if level == adsk.core.LogLevels.ErrorLogLevel:
        app.log(message, level, adsk.core.LogTypes.FileLogType)
    if DEBUG or force_console:
        app.log(message, level, adsk.core.LogTypes.ConsoleLogType)


def handle_error(name: str, show_message_box: bool = False):
    log('===== Error =====', adsk.core.LogLevels.ErrorLogLevel)
    log(f'{name}\n{traceback.format_exc()}', adsk.core.LogLevels.ErrorLogLevel)
    if show_message_box:
        ui.messageBox(f'{name}\n{traceback.format_exc()}')
