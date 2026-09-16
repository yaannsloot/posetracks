# ORT is the only gpu compute backend and it requires ONNX graphs as input.
# These functions help create graphs for pre/post processing purposes.

from .utils import prepare_session, get_session
from .datatypes import Tensor, DType
from onnx import TensorProto
from onnx.helper import (
    make_model, make_node, make_graph,
    make_tensor_value_info, make_opsetid)
from onnx.numpy_helper import from_array
from onnx.checker import check_model
from typing import Sequence, Optional
import numpy as np


def _create_identity_graph(n_dim: int, dtype: DType):
    dtype = dtype.as_onnx()
    X = make_tensor_value_info("X", dtype, [None] * n_dim)
    Y = make_tensor_value_info("Y", dtype, [None] * n_dim)
    node = make_node("Identity", ["X"], ["Y"])
    graph = make_graph([node], "identity", [X], [Y])
    model = make_model(graph, opset_imports=[make_opsetid("", 17)])
    check_model(model)
    return model


def to_numpy(input: Tensor):
    assert (input.is_ort())
    dtype = input.dtype
    n_dim = len(input.shape)
    graph_id = ("identity", n_dim, dtype)
    session = get_session(graph_id)
    if session is None:
        graph = _create_identity_graph(n_dim, dtype)
        session = prepare_session(graph_id, graph.SerializeToString())
    out = session.run(None, {"X": input}, output_device="cpu")[0]
    return out


def _create_channel_swap_graph(layout: str, include_alpha: bool, dtype: DType):
    dtype = dtype.as_onnx()
    layout = layout.upper()
    swap_axis = layout.find("C")
    X = make_tensor_value_info("X", dtype, [*layout])
    Y = make_tensor_value_info("Y", dtype, [*layout])
    idx = from_array(np.array([2, 1, 0, 4] if include_alpha else [
                     2, 1, 0], dtype=np.int64), "swap_idx")
    node = make_node("Gather", ["X", "swap_idx"], ["Y"], axis=swap_axis)
    graph = make_graph([node], "channel_swap", [X], [Y], [idx])
    model = make_model(graph, opset_imports=[make_opsetid("", 17)])
    check_model(model)
    return model


def channel_swap(input: Tensor, layout="nchw"):
    assert (input.is_ort())
    dtype = input.dtype
    include_alpha = input.shape[-1] == 4
    graph_id = ("channel_swap", layout, include_alpha, dtype)
    session = get_session(graph_id)
    if session is None:
        graph = _create_channel_swap_graph(layout, include_alpha, dtype)
        session = prepare_session(graph_id, graph.SerializeToString())
    out = session.run(None, {"X": input})[0]
    return out


def _create_transpose_graph(axes: Sequence[int], dtype: DType):
    dtype = dtype.as_onnx()
    layout = [str(a) for a in range(len(axes))]
    X = make_tensor_value_info("X", dtype, layout)
    Y = make_tensor_value_info("Y", dtype, [layout[a] for a in axes])
    node = make_node("Transpose", ["X"], ["Y"], perm=list(axes))
    graph = make_graph([node], "transpose", [X], [Y])
    model = make_model(graph, opset_imports=[make_opsetid("", 17)])
    check_model(model)
    return model


def transpose(input: Tensor, axes: Sequence[int]):
    assert (input.is_ort())
    dtype = input.dtype
    graph_id = ("transpose", axes, dtype)
    session = get_session(graph_id)
    if session is None:
        graph = _create_transpose_graph(axes, dtype)
        session = prepare_session(graph_id, graph.SerializeToString())
    out = session.run(None, {"X": input})[0]
    return out


def _create_resize_graph(mode: str, scales: Optional[list[float]],
                         sizes: Optional[list[int]], dtype: DType):
    dtype = dtype.as_onnx()
    if not (scales or sizes):
        raise ValueError(
            "Must specify either axes scales or sizes for resize operator.")
    n_dim = len(sizes) if sizes else len(scales)
    inputs = ["X", "", "", "sizes"] if sizes else ["X", "", "scales", ""]
    arg_name = "sizes" if sizes else "scales"
    arg = from_array(np.array(sizes if sizes else scales,
                     np.int64 if sizes else np.float32), arg_name)
    layout = [str(a) for a in range(n_dim)]
    X = make_tensor_value_info("X", dtype, layout)
    Y = make_tensor_value_info("Y", dtype, layout)
    node = make_node("Resize", inputs, ["Y"], mode=mode)
    graph = make_graph([node], "resize", [X], [Y], [arg])
    model = make_model(graph, opset_imports=[make_opsetid("", 17)])
    check_model(model)
    return model


def resize(input: Tensor, mode="nearest", scales: Optional[list[float]] = None,
           sizes: Optional[list[int]] = None):
    assert (input.is_ort())
    dtype = input.dtype
    graph_id = ("resize", mode, tuple(sizes) if sizes else None,
                None if sizes else tuple(scales), dtype)
    session = get_session(graph_id)
    if session is None:
        graph = _create_resize_graph(mode, scales, sizes, dtype)
        session = prepare_session(graph_id, graph.SerializeToString())
    out = session.run(None, {"X": input})[0]
    return out


def _create_unsqueeze_graph(n_dim: int, axes: list[int], dtype: DType):
    dtype = dtype.as_onnx()
    layout = [str(a) for a in range(n_dim)]
    y_layout = [str(a) for a in range(n_dim + len(axes))]
    X = make_tensor_value_info("X", dtype, layout)
    Y = make_tensor_value_info("Y", dtype, y_layout)
    arg = from_array(np.array(axes, np.int64), "axes")
    node = make_node("Unsqueeze", ["X", "axes"], ["Y"])
    graph = make_graph([node], "unsqueeze", [X], [Y], [arg])
    model = make_model(graph, opset_imports=[make_opsetid("", 17)])
    check_model(model)
    return model


def unsqueeze(input: Tensor, axes: list[int]):
    assert (input.is_ort())
    dtype = input.dtype
    graph_id = ("unsqueeze", len(input.shape), tuple(axes), dtype)
    session = get_session(graph_id)
    if session is None:
        graph = _create_unsqueeze_graph(len(input.shape), axes, dtype)
        session = prepare_session(graph_id, graph.SerializeToString())
    out = session.run(None, {"X": input})[0]
    return out


def _create_concat_graph(n_vals: int, n_dim: int, axis: int, dtype: DType):
    dtype = dtype.as_onnx()
    layout = [str(a) for a in range(n_dim)]
    input_names = [f"input_{i}" for i in range(n_vals)]
    inputs = [make_tensor_value_info(n, dtype, layout) for n in input_names]
    Y = make_tensor_value_info("Y", dtype, layout)
    node = make_node("Concat", input_names, ["Y"], axis=axis)
    graph = make_graph([node], "concat", inputs, [Y])
    model = make_model(graph, opset_imports=[make_opsetid("", 17)])
    check_model(model)
    return model


def concat(input: Sequence[Tensor], axis: int):
    assert (all(v.is_ort() for v in input))
    if not input:
        return None
    dtype = input[0].dtype
    assert (all(v.dtype == input[0].dtype for v in input))
    n_dim = len(input[0].shape)
    assert (all(len(v.shape) == n_dim for v in input))
    graph_id = ("concat", len(input), n_dim, axis, dtype)
    session = get_session(graph_id)
    if session is None:
        graph = _create_concat_graph(len(input), n_dim, axis, dtype)
        session = prepare_session(graph_id, graph.SerializeToString())
    out = session.run(None, {f"input_{i}": v for i, v in enumerate(input)})[0]
    return out


def _create_normalization_graph(layout: str, mean: float | Sequence[float],
                                std: float | Sequence[float], channels: int,
                                cast_out: bool, dtype: DType):
    dtype = dtype.as_onnx()
    X = make_tensor_value_info("X", dtype, [*layout])
    Y = make_tensor_value_info(
        "Y", dtype if cast_out else TensorProto.FLOAT, [*layout])
    reshape_full = [channels if a == "c" else 1 for a in layout]
    reshape_single = [1 for _ in layout]
    mean = np.array(mean, dtype=np.float32).reshape(
        *(reshape_single if isinstance(mean, float) else reshape_full))
    std = np.array(std, dtype=np.float32).reshape(
        *(reshape_single if isinstance(std, float) else reshape_full))
    mean = from_array(mean, "mean")
    std = from_array(std, "std")
    nodes = []
    nodes.append(make_node("Cast", ["X"], ["X_fp"], to=TensorProto.FLOAT))
    nodes.append(make_node("Sub", ["X_fp", "mean"], ["X_sub"]))
    nodes.append(make_node("Div", ["X_sub", "std"], [
                 "Y_fp" if cast_out else "Y"]))
    if cast_out:
        nodes.append(make_node("Cast", ["Y_fp"], ["Y"], to=dtype))
    graph = make_graph(nodes, "normalize", [X], [Y], [mean, std])
    model = make_model(graph, opset_imports=[make_opsetid("", 17)])
    check_model(model)
    return model


def normalize(input: Tensor, mean: float | Sequence[float], std: float | Sequence[float],
              layout: str = "nchw", cast_out: bool = False):
    if not input.is_ort():
        raise ValueError("Input must be an ORT tensor.")
    if not any(isinstance(a, float) for a in (mean, std)) and len(mean) != len(std):
        raise ValueError(
            "List of means and std dev must not differ in length.")
    shape = input.shape
    if len(layout) != len(shape):
        raise ValueError("Layout does not match input shape.")
    layout = layout.lower()
    channel_axis = layout.find("c")
    if any(not isinstance(a, float) and len(a) != shape[channel_axis] for a in (mean, std)):
        raise ValueError(
            "List of values for mean/std must match number of channels.")
    dtype = input.dtype
    mean, std = (v if isinstance(v, float) else tuple(v) for v in (mean, std))
    graph_id = ("normalize", mean, std, layout, cast_out, dtype)
    session = get_session(graph_id)
    if session is None:
        graph = _create_normalization_graph(
            layout, mean, std, shape[channel_axis], cast_out, dtype)
        session = prepare_session(graph_id, graph.SerializeToString())
    out = session.run(None, {"X": input})[0]
    return out


def _create_cast_graph(n_dim: int, input_dtype: DType,
                       output_dtype: DType):
    input_dtype = input_dtype.as_onnx()
    output_dtype = output_dtype.as_onnx()
    layout = [str(a) for a in range(n_dim)]
    X = make_tensor_value_info("X", input_dtype, layout)
    Y = make_tensor_value_info("Y", output_dtype, layout)
    node = make_node("Cast", ["X"], ["Y"], to=output_dtype)
    graph = make_graph([node], "cast", [X], [Y])
    model = make_model(graph, opset_imports=[make_opsetid("", 17)])
    check_model(model)
    return model


def cast(input: Tensor, dtype: DType):
    input_dtype = input.dtype
    output_dtype = dtype
    if input_dtype == output_dtype:
        return input
    n_dim = len(input.shape)
    graph_id = ("cast", n_dim, input_dtype, output_dtype)
    session = get_session(graph_id)
    if session is None:
        graph = _create_cast_graph(n_dim, input_dtype, output_dtype)
        session = prepare_session(graph_id, graph.SerializeToString())
    out = session.run(None, {"X": input})[0]
    return out


def _create_sub_graph(n_dim_min: int, n_dim_sub: int, min_dtype: DType, sub_dtype: DType):
    op_dtype = DType.promote(min_dtype, sub_dtype).as_onnx()
    min_dtype = min_dtype.as_onnx()
    sub_dtype = sub_dtype.as_onnx()
    layout_min = [str(a) for a in range(n_dim_min)]
    layout_sub = [str(a) for a in range(n_dim_sub)]
    min = make_tensor_value_info("MIN", min_dtype, layout_min)
    sub = make_tensor_value_info("SUB", sub_dtype, layout_sub)
    out = make_tensor_value_info("OUT", op_dtype, layout_min)
    nodes = []
    nodes.append(make_node("Cast", ["MIN"], ["MIN_op"], to=op_dtype))
    nodes.append(make_node("Cast", ["SUB"], ["SUB_op"], to=op_dtype))
    nodes.append(make_node("Sub", ["MIN_op", "SUB_op"], ["OUT"]))
    graph = make_graph(nodes, "sub", [min, sub], [out])
    model = make_model(graph, opset_imports=[make_opsetid("", 17)])
    check_model(model)
    return model


def sub(min: Tensor, sub: Tensor):
    min_dtype = min.dtype
    sub_dtype = sub.dtype
    n_dim_min = len(min.shape)
    n_dim_sub = len(sub.shape)
    graph_id = ("sub", n_dim_min, n_dim_sub, min_dtype, sub_dtype)
    session = get_session(graph_id)
    if session is None:
        graph = _create_sub_graph(n_dim_min, n_dim_sub, min_dtype, sub_dtype)
        session = prepare_session(graph_id, graph.SerializeToString())
    out = session.run(None, {"MIN": min, "SUB": sub})[0]
    return out
