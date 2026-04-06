import sys
from typing import Callable

import adsk.core
from .general_utils import handle_error

_handlers = []


def add_handler(event: adsk.core.Event,
                callback: Callable,
                *,
                name: str = None,
                local_handlers: list = None):
    module       = sys.modules[event.__module__]
    handler_type = module.__dict__[event.add.__annotations__['handler']]
    handler      = _make_handler(handler_type, callback, name, local_handlers)
    event.add(handler)
    return handler


def clear_handlers():
    global _handlers
    _handlers = []


def _make_handler(handler_type, callback, name=None, local_handlers=None):
    name = name or handler_type.__name__

    class Handler(handler_type):
        def __init__(self):
            super().__init__()

        def notify(self, args):
            try:
                callback(args)
            except:
                handle_error(name)

    h = Handler()
    (local_handlers if local_handlers is not None else _handlers).append(h)
    return h
