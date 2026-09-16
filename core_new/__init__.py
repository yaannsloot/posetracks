import pkgutil
import importlib
from pathlib import Path

_REGISTRY = {}

def register(func, name=None):
    key = name or func.__name__
    _REGISTRY[key] = func
    return func

def apply(input: any, op: str, /, **kwargs):
    return _REGISTRY[op](input, **kwargs)

_package_dir = Path(__file__).parent

for _, module_name, _ in pkgutil.iter_modules([str(_package_dir)]):
    if module_name != "__init__":
        importlib.import_module(f"{__name__}.{module_name}")
