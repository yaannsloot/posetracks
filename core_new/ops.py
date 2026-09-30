from . import register
from . import gpu
from .datatypes import Array, Tensor, Image, DType, Number, Context
import numpy as np
from onnxruntime import OrtValue
from typing import Sequence, Optional
import cv2


# ------------------------------  Array Ops  ------------------------------

@register
def to_gpu(ctx: Context, /, **kwargs):
    try:
        ctx.expect_type(value=Array)
        input = [ctx.value]
        single_val = True
    except:
        ctx.expect_seq_type(value=Array)
        input = ctx.value
        single_val = False
    output = []
    for arr in input:
        if arr.is_ort():
            output.append(arr)
        else:
            output.append(type(arr)(OrtValue.ortvalue_from_numpy(arr.data, "cuda"),
                                    **{k: v for k, v in vars(arr).items() if k != "data"}))
    ctx.update(output[0] if single_val else output)
    return ctx


@register
def to_cpu(ctx: Context, /, **kwargs):
    try:
        ctx.expect_type(value=Array)
        input = [ctx.value]
        single_val = True
    except:
        ctx.expect_seq_type(value=Array)
        input = ctx.value
        single_val = False
    output = []
    for arr in input:
        if arr.is_numpy():
            output.append(arr)
        else:
            output.append(type(arr)(gpu.to_numpy(arr),
                                    **{k: v for k, v in vars(arr).items() if k != "data"}))
    ctx.update(output[0] if single_val else output)
    return ctx


@register
def debug_arr_val(ctx: Context, /, **kwargs):
    try:
        ctx.expect_type(value=Array)
        print(str(ctx.value.data))
    except:
        ctx.expect_seq_type(value=Array)
        for a in ctx.value:
            print(str(a.data))
    return ctx


@register
def debug_arr_shape(ctx: Context, /, **kwargs):
    try:
        ctx.expect_type(value=Array)
        print(str(ctx.value.shape))
    except:
        ctx.expect_seq_type(value=Array)
        for a in ctx.value:
            print(str(a.shape))
    return ctx


# ------------------------------  Tensor Ops  ------------------------------


@register
def batch_arrays(ctx: Context, /, inputs: Optional[list[str]] = None, **kwargs):
    input = []
    for i in inputs if inputs else ["value"]:
        try:
            ctx.expect_type(**{i: Array})
            input.append(ctx[i])
        except:
            ctx.expect_seq_type(**{i: Array})
            input.extend(ctx[i])
    is_ort = any(a.is_ort() for a in input)
    data = [a.to_ort() if is_ort else a.to_numpy() for a in input]
    if is_ort:
        data = [gpu.unsqueeze(a, [0]) for a in data]
        data = gpu.concat(data, 0)
    else:
        data = np.stack(data)
    ctx.update(Tensor(data))
    return ctx


@register
def concat(ctx: Context, /, axis: int, inputs: Optional[list[str]] = None, **kwargs):
    input = []
    for i in inputs if inputs else ["value"]:
        try:
            ctx.expect_type(**{i: Tensor})
            input.append(ctx[i])
        except:
            ctx.expect_seq_type(**{i: Tensor})
            input.extend(ctx[i])
    is_ort = any(a.is_ort() for a in input)
    data = [a.to_ort() if is_ort else a.to_numpy() for a in input]
    if is_ort:
        data = gpu.concat(data, axis)
    else:
        data = np.concat([a.data for a in data], axis)
    ctx.update(Tensor(data))
    return ctx


@register
def transpose(ctx: Context, /, axes: Sequence[int], **kwargs):
    ctx.expect_type(value=Tensor)
    if ctx.value.is_ort():
        ctx.update(gpu.transpose(ctx.value, axes))
    else:
        ctx.update(Tensor(np.transpose(ctx.value.data, axes)))
    return ctx


@register
def cast(ctx: Context, /, dtype: DType, **kwargs):
    ctx.expect_type(value=Tensor)
    if ctx.value.is_ort():
        ctx.update(gpu.cast(ctx.value, dtype))
    elif ctx.value.dtype != dtype:
        ctx.update(Tensor(ctx.value.data.astype(dtype.as_numpy())))
    return ctx


@register
def gather(ctx: Context, /, indices: Sequence[int], axis: int, **kwargs):
    ctx.expect_type(value=Tensor)
    if ctx.value.is_ort():
        ctx.update(gpu.gather(ctx.value, indices, axis))
    else:
        ctx.update(Tensor(np.take(ctx.value.data, indices, axis)))
    return ctx


_slice = slice


@register
def slice(ctx: Context, /, starts: Sequence[int], ends: Sequence[int],
          axes: Sequence[int], steps: Sequence[int]):
    ctx.expect_type(value=Tensor)
    if ctx.value.is_ort():
        ctx.update(gpu.slice(ctx.value, starts, ends, axes, steps))
    else:
        index = [_slice(None)] * len(ctx.value.shape)
        for s, e, a, st in zip(starts, ends, axes, steps):
            index[a] = _slice(s, e, st)
        ctx.update(Tensor(ctx.value.data[tuple(index)]))
    return ctx


@register
def sub(ctx: Context, /, other: str = "other", **kwargs):
    ctx.expect_type(**{"value": Tensor, other: Tensor})
    input = ctx.value
    other = ctx[other]
    other = other.to(input.device)
    if input.is_ort():
        ctx.update(gpu.sub(input, other))
    else:
        ctx.update(Tensor(np.subtract(input.data, other.data)))
    return ctx


@register
def add(ctx: Context, /, other: str = "other", **kwargs):
    ctx.expect_type(**{"value": Tensor, other: Tensor})
    input = ctx.value
    other = ctx[other]
    other = other.to(input.device)
    if input.is_ort():
        ctx.update(gpu.add(input, other))
    else:
        ctx.update(Tensor(np.add(input.data, other.data)))
    return ctx


@register
def div(ctx: Context, /, other: str = "other", **kwargs):
    ctx.expect_type(**{"value": Tensor, other: Tensor})
    input = ctx.value
    other = ctx[other]
    other = other.to(input.device)
    if input.is_ort():
        ctx.update(gpu.div(input, other))
    else:
        ctx.update(Tensor(np.divide(input.data, other.data)))
    return ctx


@register
def mul(ctx: Context, /, other: str = "other", **kwargs):
    ctx.expect_type(**{"value": Tensor, other: Tensor})
    input = ctx.value
    other = ctx[other]
    other = other.to(input.device)
    if input.is_ort():
        ctx.update(gpu.mul(input, other))
    else:
        ctx.update(Tensor(np.multiply(input.data, other.data)))
    return ctx


@register
def normalize(ctx: Context, /, mean=0.5, std=0.5, layout="nchw", **kwargs):
    ctx.expect_type(value=Tensor)
    input = ctx.value
    mean, std = Tensor(mean), Tensor(std)
    m, s = mean.data.size, std.data.size
    n_dim = len(input.shape)
    channel_axis = layout.lower().index('c')
    mean = Tensor(mean.data.reshape(
        [m if n == channel_axis else 1 for n in range(n_dim)])).cast("float32").to(input.device)
    std = Tensor(std.data.reshape(
        [s if n == channel_axis else 1 for n in range(n_dim)])).cast("float32").to(input.device)
    ctx.update(input.sub(mean).div(std))
    return ctx


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
def resize(ctx: Context, /, method: str, scale: Optional[tuple[float, float]] = None,
           size: Optional[tuple[int, int]] = None, **kwargs):
    size = [None] * 2 if size is None else size
    scale = [None] * 2 if scale is None else scale
    try:
        ctx.expect_type(value=Image)
        ctx.update(resize_image(ctx.value, method, *size, *scale))
    except:
        ctx.expect_seq_type(value=Image)
        ctx.update([resize_image(img, method, *size, *scale)
                   for img in ctx.value])
    return ctx


@register
def img_to_tensor(ctx: Context, /, dtype="float32", **kwargs):
    try:
        ctx.expect_type(value=Image)
        input = [ctx.value]
    except:
        ctx.expect_seq_type(value=Image)
        input = ctx.value
    input = [(i.channel_swap() if i.color_format in
              ("bgr", "bgra") else i).cast(dtype) for i in input]
    ctx.update(Tensor.batch_arrays(input).transpose((0, 3, 1, 2)))
    return ctx


@register
def channel_swap(ctx: Context, /, **kwargs):
    try:
        ctx.expect_type(value=Image)
        input = [ctx.value]
        single_val = True
    except:
        ctx.expect_seq_type(value=Image)
        input = ctx.value
        single_val = False
    fmt_a = ("rgb", "bgr")
    fmt_b = ("rgba", "bgra")
    output = []
    for img in input:
        if img.color_format == "gray":
            output.append(img)
            continue
        if img.color_format in fmt_a:
            color_format = fmt_a[fmt_a.index(img.color_format) - 1]
        elif img.color_format in fmt_b:
            color_format = fmt_b[fmt_b.index(img.color_format) - 1]
        else:
            raise ValueError(f"Unsupported color format: {img.color_format}")
        if img.is_ort():
            output.append(Image(gpu.channel_swap(
                img, img.layout), color_format, img.layout))
        else:
            swap_axis = img.layout.find("c")
            output.append(Image(np.take(img.data, [2, 1, 0, 3] if img.shape[-1] == 4 else [2, 1, 0], swap_axis),
                                color_format, img.layout))
    ctx.update(output[0] if single_val else output)
    return ctx


@register
def img_cast(ctx: Context, /, dtype: DType, scale: Number | str = "auto", **kwargs):
    try:
        ctx.expect_type(value=Image)
        input = [ctx.value]
        single_val = True
    except:
        ctx.expect_seq_type(value=Image)
        input = ctx.value
        single_val = False
    output = []
    for img in input:
        color_format = img.color_format
        layout = img.layout
        img = Tensor(img)
        if scale == "auto":
            dtype_orig = img.dtype.as_numpy()
            if np.issubdtype(dtype_orig, np.integer):
                scale = np.iinfo(dtype_orig).max
            else:
                scale = 1.0
            scale = Tensor(scale).cast(dtype)
        img = Image(img.cast(dtype).div(Tensor(scale)), color_format, layout)
        output.append(img)
    ctx.update(output[0] if single_val else output)
    return ctx
