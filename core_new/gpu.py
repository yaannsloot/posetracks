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
    graph_id = ("transpose", tuple(axes), dtype)
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


def _create_binary_op_graph(op: str, n_dim_a: int, n_dim_b: int, dtype_a: DType, dtype_b: DType):
    op_dtype = DType.promote(dtype_a, dtype_b).as_onnx()
    dtype_a = dtype_a.as_onnx()
    dtype_b = dtype_b.as_onnx()
    layout_a = [str(a) for a in range(n_dim_a)]
    layout_b = [str(a) for a in range(n_dim_b)]
    A = make_tensor_value_info("A", dtype_a, layout_a)
    B = make_tensor_value_info("B", dtype_b, layout_b)
    out = make_tensor_value_info(
        "OUT", op_dtype, layout_a if n_dim_a > n_dim_b else layout_b)
    nodes = []
    nodes.append(make_node("Cast", ["A"], ["A_op"], to=op_dtype))
    nodes.append(make_node("Cast", ["B"], ["B_op"], to=op_dtype))
    nodes.append(make_node(op.capitalize(), ["A_op", "B_op"], ["OUT"]))
    graph = make_graph(nodes, op.lower(), [A, B], [out])
    model = make_model(graph, opset_imports=[make_opsetid("", 17)])
    check_model(model)
    return model


def _execute_binary_op(op: str, a: Tensor, b: Tensor):
    dtype_a = a.dtype
    dtype_b = b.dtype
    n_dim_a = len(a.shape)
    n_dim_b = len(b.shape)
    graph_id = (op.lower(), n_dim_a, n_dim_b, dtype_a, dtype_b)
    session = get_session(graph_id)
    if session is None:
        graph = _create_binary_op_graph(op, n_dim_a, n_dim_b, dtype_a, dtype_b)
        session = prepare_session(graph_id, graph.SerializeToString())
    out = session.run(None, {"A": a, "B": b})[0]
    return out


def sub(a: Tensor, b: Tensor):
    return _execute_binary_op("sub", a, b)


def add(a: Tensor, b: Tensor):
    return _execute_binary_op("add", a, b)


def div(a: Tensor, b: Tensor):
    return _execute_binary_op("div", a, b)


def mul(a: Tensor, b: Tensor):
    return _execute_binary_op("mul", a, b)
