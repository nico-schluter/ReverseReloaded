# FusionTesting — Fusion 360 add-in for API exploration during surface fitting development.
#
# Load via Scripts & Add-Ins (Shift+S) → Add-Ins → point to this directory.
# Not production code — exploration only.
#
# Commands included:
#   tableTest        — validates TableCommandInput layout and row mechanics (label | dropdown | count),
#                      add/remove rows, toolbar buttons, row selection. Key lessons fed into src/.
#   customFeatureTest — explores CustomFeature API for potential timeline integration (deferred feature).
#   commandDialog, paletteSend, paletteShow — Autodesk template leftovers, ignore.

from . import commands
from .lib import fusionAddInUtils as futil


def run(context):
    try:
        # This will run the start function in each of your commands as defined in commands/__init__.py
        commands.start()

    except:
        futil.handle_error('run')


def stop(context):
    try:
        # Remove all of the event handlers your app has created
        futil.clear_handlers()

        # This will run the start function in each of your commands as defined in commands/__init__.py
        commands.stop()

    except:
        futil.handle_error('stop')