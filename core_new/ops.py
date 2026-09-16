from . import gpu, register
from .datatypes import Array, Tensor, Image, DType, Number
import numpy as np
from onnxruntime import OrtValue
from typing import Sequence, Optional
import cv2


# ------------------------------  Array Ops  ------------------------------

@register
def to_gpu(input: Array, /, **kwargs):
    if input.is_ort():
        return input
    return type(input)(OrtValue.ortvalue_from_numpy(input.data, "cuda"),
                       **{k: v for k, v in vars(input).items() if k != "data"})


@register
def to_cpu(input: Array, /, **kwargs):
    if input.is_numpy():
        return input
    return type(input)(gpu.to_numpy(input),
                       **{k: v for k, v in vars(input).items() if k != "data"})


@register
def all_to_gpu(input: Sequence[Array], /, **kwargs):
    return [x.to_ort() for x in input]

# ------------------------------  Tensor Ops  ------------------------------


@register
def batch_arrays(input: Sequence[Array], /, **kwargs):
    is_ort = any(a.is_ort() for a in input)
    data = [a.to_ort() if is_ort else a.to_numpy() for a in input]
    if is_ort:
        data = [gpu.unsqueeze(a, [0]) for a in data]
        data = gpu.concat(data, 0)
    else:
        data = np.stack(data)
    return Tensor(data)


@register
def concat(input: Sequence[Tensor], /, axis: int, **kwargs):
    is_ort = any(a.is_ort() for a in input)
    data = [a.to_ort() if is_ort else a.to_numpy() for a in input]
    if is_ort:
        data = gpu.concat(data, axis)
    else:
        data = np.concat([a.data for a in data], axis)
    return Tensor(data)


@register
def transpose(input: Tensor, /, axes: Sequence[int], **kwargs):
    if input.is_ort():
        return gpu.transpose(input, axes)
    return Tensor(np.transpose(input.data, axes))


@register
def cast(input: Tensor, /, dtype: DType, scale: Optional[float] = None):
    if input.is_ort():
        return gpu.cast(input, dtype)
    if input.dtype == dtype:
        return input
    return Tensor(input.data.astype(dtype.as_numpy()))


@register
def normalize(input: Tensor, mean=0.5, std=0.5, layout="nchw", cast_out=False, **kwargs) -> Tensor:
    mean, std = (float(v) if isinstance(v, Number) else v for v in (mean, std))
    if input.is_ort():
        return gpu.normalize(input, mean, std, layout, cast_out)
    if not any(isinstance(a, float) for a in (mean, std)) and len(mean) != len(std):
        raise ValueError(
            "List of means and std dev must not differ in length.")
    shape = input.shape
    if len(layout) != len(shape):
        raise ValueError("Layout does not match input shape.")
    data = input.data
    orig_dtype = data.dtype
    channel_idx = layout.index("c")
    channels = shape[channel_idx]
    reshape_full = [channels if a == "c" else 1 for a in layout]
    reshape_single = [1 for _ in layout]
    mean = np.array(mean, dtype=np.float32).reshape(
        *(reshape_single if isinstance(mean, float) else reshape_full))
    std = np.array(std, dtype=np.float32).reshape(
        *(reshape_single if isinstance(std, float) else reshape_full))
    data = (data.astype(np.float32) - mean) / std
    return Tensor(data.astype(orig_dtype) if cast_out else data)


# ------------------------------  Image Ops  ------------------------------


cv2_interp = {
    "nearest": cv2.INTER_NEAREST,
    "linear": cv2.INTER_LINEAR,
    "cubic": cv2.INTER_CUBIC,
    "area": cv2.INTER_AREA,
    "lanczos": cv2.INTER_LANCZOS4,
}


def resize_image(input: Image, mode, w: int = None, h: int = None,
                 fx: float = None, fy: float = None) -> Image:
    if input.is_ort() and mode in ("nearest", "linear", "cubic"):
        dims = input.shape
        sizes = {"w": w, "h": h}
        sizes = ([dims[i] if v not in sizes else sizes[v]
                   for i, v in enumerate(input.layout)]
                  if w and h else None)
        scales = {"w": fx, "h": fy}
        scales = ([1.0 if v not in scales else scales[v]
                  for i, v in enumerate(input.layout)]
                 if fx and fy else None)
        return Image(gpu.resize(input, mode, scales, sizes), input.color_format, input.layout)
    if mode not in cv2_interp:
        raise ValueError(f"Mode '{mode}' not supported.")
    is_ort = input.is_ort()
    input = input.to_numpy()
    mode = cv2_interp[mode]
    dsize = (w, h) if w and h else None
    if dsize:
        data = cv2.resize(input.data, dsize, interpolation=mode)
    else:
        data = cv2.resize(input.data, dsize, fx=fx, fy=fy, interpolation=mode)
    output = Image(data, input.color_format, input.layout)
    if is_ort:
        output = output.to_ort()
    return output


@register
def resize(input: Sequence[Image], /, mode: str, scale: Optional[tuple[float, float]] = None,
           size: Optional[tuple[int, int]] = None, **kwargs) -> Sequence[Image]:
    size = [None] * 2 if size is None else size
    scale = [None] * 2 if scale is None else scale
    return [resize_image(img, mode, *size, *scale) for img in input]


@register
def img_to_tensor(input: Sequence[Image], /, dtype="float32", **kwargs) -> Tensor:
    input = [Tensor(i.channel_swap() if i.color_format in (
        "bgr", "bgra") else i).cast(dtype) for i in input]
    return Tensor.batch_arrays(input).transpose((0, 3, 1, 2))


@register
def channel_swap(input: Image, /, **kwargs):
    if input.color_format == "gray":
        return input
    fmt_a = ("rgb", "bgr")
    fmt_b = ("rgba", "bgra")
    if input.color_format in fmt_a:
        color_format = fmt_a[fmt_a.index(input.color_format) - 1]
    elif input.color_format in fmt_b:
        color_format = fmt_b[fmt_b.index(input.color_format) - 1]
    else:
        raise ValueError(f"Unsupported color format: {input.color_format}")
    if input.is_ort():
        return Image(gpu.channel_swap(input, input.layout), color_format, input.layout)
    swap_axis = input.layout.find("c")
    return Image(np.take(input.data, [2, 1, 0, 3] if input.shape[-1] == 4 else [2, 1, 0], swap_axis),
                 color_format, input.layout)
