import pkgutil
import importlib
from pathlib import Path

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .datatypes import Context

_REGISTRY = {}


def register(func, name=None):
    key = name or func.__name__
    _REGISTRY[key] = func
    return func


def apply(op: str, context: Context, /, **kwargs) -> Context:
    context._in = kwargs.get("in", "_prev")
    context._out = kwargs.get("out", "_prev")
    return _REGISTRY[op](context, **kwargs)


_package_dir = Path(__file__).parent

for _, module_name, _ in pkgutil.iter_modules([str(_package_dir)]):
    if module_name != "__init__":
        importlib.import_module(f"{__name__}.{module_name}")
