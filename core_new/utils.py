from . import _REGISTRY
from .datatypes import Tensor
import json
import onnxruntime as ort
from pathlib import Path
import time
import sys

MODEL_DEFS = {}
_SESSIONS = {}


class AutoIOSession(ort.InferenceSession):
    def run(self, output_names: list[str], input_feed: dict[str, Tensor], run_options=None, output_device="match"):
        binding = self.io_binding()
        has_ort_inputs = False

        for name, tensor in input_feed.items():
            if tensor.is_ort():
                has_ort_inputs = True
                binding.bind_ortvalue_input(name, tensor.data)
            else:
                binding.bind_cpu_input(name, tensor.data)

        destination = output_device
        if destination == "match":
            destination = "cuda" if has_ort_inputs else "cpu"

        if output_names is None:
            for output in self.get_outputs():
                binding.bind_output(output.name, destination)
        else:
            for name in output_names:
                binding.bind_output(name, destination)

        self.run_with_iobinding(binding, run_options)

        result = [Tensor(output.numpy()) if output.device_name() == "cpu" else Tensor(output)
                  for output in binding.get_outputs()]

        return result


def load_spec(file, model_dir):
    with open(file, "r") as f:
        model_spec = json.load(f)
    if "rel_path" in model_spec:
        model_path = model_dir / model_spec["rel_path"]
    else:
        model_path = Path(model_spec["path"])
    if not model_path.exists():
        raise RuntimeError(
            f"Error parsing {file.name}. Could not locate {model_path}. File does not exist.")
    if "input_type" not in model_spec:
        raise RuntimeError(
            f"Error parsing {file.name}. Input type is undefined.")
    if "output_type" not in model_spec:
        raise RuntimeError(
            f"Error parsing {file.name}. Output type is undefined.")
    for call in model_spec["preprocess"] + model_spec["postprocess"]:
        if call["op"] not in _REGISTRY:
            raise RuntimeError(
                f"Error parsing {file.name}. Unknown operator {call["op"]}.")
    model_spec["path"] = model_path
    model_spec["name"] = model_spec.get("name", file.stem)
    return model_spec


def load_model_defs(defs_dir, model_dir):
    defs_dir = Path(defs_dir)
    model_dir = Path(model_dir)

    for file in defs_dir.iterdir():
        if not file.is_file() or file.suffix != ".json":
            continue
        try:
            model_spec = load_spec(file, model_dir)
        except Exception as e:
            print(e, file=sys.stderr)
            continue
        MODEL_DEFS[model_spec["name"]] = model_spec


def prepare_session(name, model):
    now = time.time_ns() / 1e9
    for n in list(_SESSIONS.keys()):
        if n == name:
            continue
        val = _SESSIONS[n]
        if now - val["last"] > 5 * 60:
            del _SESSIONS[n]
    if name in _SESSIONS:
        _SESSIONS[name]["last"] = now
    else:
        _SESSIONS[name] = {
            "session": AutoIOSession(model,
                                     providers=['CUDAExecutionProvider']),
            "last": now
        }
    return _SESSIONS[name]["session"]


def has_session(name):
    return name in _SESSIONS


def get_session(name):
    session = _SESSIONS.get(name, None)
    if session is not None:
        _SESSIONS[name]["last"] = time.time_ns() / 1e9
    return session["session"] if session else None


def clear_cache():
    _SESSIONS.clear()
