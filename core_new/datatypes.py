from . import apply
from numpy.typing import NDArray
import numpy as np
from dataclasses import dataclass
from onnxruntime import OrtValue
import OpenImageIO as oiio
from typing import Sequence, Self, Optional
from onnx import TensorProto
from onnx.helper import np_dtype_to_tensor_dtype, tensor_dtype_to_np_dtype

Arrays = NDArray | list[NDArray]
Number = int | float | complex


ORT_TYPE_TO_NP_TYPE = {
    "tensor(float)": np.float32,
    "tensor(float16)": np.float16,
    "tensor(double)": np.float64,
    "tensor(int32)": np.int32,
    "tensor(int64)": np.int64,
    "tensor(int8)": np.int8,
    "tensor(int16)": np.int16,
    "tensor(uint8)": np.uint8,
    "tensor(uint16)": np.uint16,
    "tensor(uint32)": np.uint32,
    "tensor(uint64)": np.uint64,
    "tensor(bool)": np.bool_,  # verify
    "tensor(string)": np.object_,  # these
}


NP_TYPE_TO_ORT_TYPE = {
    str(np.dtype(v)): k for k, v in ORT_TYPE_TO_NP_TYPE.items()
}


class DType:
    def __init__(self, dtype: DType | TensorProto.DataType | np.dtype | str):
        if isinstance(dtype, DType):
            dtype = dtype.np_dtype
        elif dtype in TensorProto.DataType.values():
            dtype = tensor_dtype_to_np_dtype(dtype)
        elif dtype in ORT_TYPE_TO_NP_TYPE:
            dtype = ORT_TYPE_TO_NP_TYPE[dtype]
        self.np_dtype = np.dtype(dtype)

    @property
    def value(self):
        return str(self.np_dtype)

    def as_numpy(self):
        return self.np_dtype

    def as_onnx(self):
        return np_dtype_to_tensor_dtype(self.np_dtype)

    def as_ort(self):
        return NP_TYPE_TO_ORT_TYPE[self.value]

    @staticmethod
    def promote(a: DType, b: DType, /):
        return DType(np.promote_types(DType(a).as_numpy(), DType(b).as_numpy()))

    def __str__(self):
        return self.value

    def __repr__(self):
        return f"DType({self.value})"

    def __hash__(self):
        return hash(self.value)

    def __eq__(self, value):
        return str(self) == str(value)


class Array:
    """Managed array from cpu/gpu memory
    """

    def __init__(self, data: NDArray | OrtValue | Sequence[Number] | Number):
        if isinstance(data, Array):
            self.data = data.data
        elif isinstance(data, (np.ndarray, OrtValue)):
            self.data = data
        elif isinstance(data, Number):
            self.data = np.array(data)
        elif isinstance(data, Sequence):
            self.data = np.array([data])
        else:
            raise ValueError("Input type not supported.")

    def is_numpy(self):
        return not self.is_ort()

    def is_ort(self):
        return isinstance(self.data, OrtValue)

    def to_ort(self) -> Self:
        return apply(self, "to_gpu")

    def to_numpy(self) -> Self:
        return apply(self, "to_cpu")

    def to(self, device: str) -> Self:
        if device in ("ort", "gpu"):
            return self.to_ort()
        return self.to_numpy()

    @property
    def dtype(self):
        if self.is_numpy():
            return DType(self.data.dtype)
        return DType(self.data.data_type())

    @property
    def shape(self):
        if isinstance(self.data, OrtValue):
            return tuple(self.data.shape())
        return self.data.shape

    @property
    def device(self):
        return "gpu" if self.is_ort() else "cpu"

    @property
    def backend(self):
        return "ort" if self.is_ort() else "numpy"


class Tensor(Array):
    """Indicates an array that is meant for inference.
    Includes additional functions for manipulating data.
    """

    @staticmethod
    def batch_arrays(data: Sequence[Array]) -> Self:
        return apply(data, "batch_arrays")

    @staticmethod
    def concat(data: Sequence[Array], axis: int) -> Self:
        return apply(data, "concat", axis=axis)

    def transpose(self, axes: Sequence[int]) -> Self:
        return apply(self, "transpose", axes=axes)

    def normalize(self, axis: int, mean: float | Sequence[float] = 0.5,
                  std: float | Sequence[float] = 0.5, cast_out=True) -> Self:
        layout = [str(i) for i in range(len(self.shape))]
        layout[axis] = 'c'
        return apply(self, "normalize", mean=mean, std=std,
                     layout=layout, cast_out=cast_out)

    def cast(self, dtype: DType) -> Self:
        return apply(self, "cast", dtype=DType(dtype))

    def sub(self, other: Tensor) -> Self:
        return apply(self, "sub", other=other)

    def add(self, other: Tensor) -> Self:
        return apply(self, "add", other=other)

    def div(self, other: Tensor) -> Self:
        return apply(self, "div", other=other)

    def mul(self, other: Tensor) -> Self:
        return apply(self, "mul", other=other)


@dataclass
class BoundingBox:
    """TLWH bounding box from object detection operations.
    """
    x: float
    y: float
    width: float
    height: float
    score: float
    id: int

    def __post_init__(self):
        self.x = float(self.x)
        self.y = float(self.y)
        self.width = float(self.width)
        self.height = float(self.height)
        self.score = float(self.score)
        self.id = int(self.id)

    def __iter__(self):
        yield self.x
        yield self.y
        yield self.width
        yield self.height
        yield self.score
        yield self.id


class Image(Array):
    def __init__(self, data: NDArray | OrtValue, color_format: str, layout: str):
        super().__init__(data)
        self.color_format = color_format.lower()
        self.layout = layout.lower()

    @property
    def size(self):
        h, w, c = self.shape
        return h, w

    @classmethod
    def open(cls, path: str) -> Self:
        buf = oiio.ImageBuf(path)
        if buf.has_error:
            raise IOError(buf.geterror())
        pixels = buf.get_pixels(oiio.FLOAT)
        channels = pixels.shape[-1]
        if channels == 4:
            color_format = "rgba"
        elif channels == 3:
            color_format = "rgb"
        else:
            color_format = "gray"
        return cls(pixels, color_format, "hwc")

    def channel_swap(self) -> Self:
        return apply(self, "channel_swap")

    def normalize(self, mean: float | Sequence[float] = 0.5, std: float | Sequence[float] = 0.5) -> Self:
        return Image(apply(self, "normalize", mean=mean, std=std, layout=self.layout),
                     self.color_format, self.layout)

    def to_tensor(self, dtype: np.dtype | str) -> Tensor:
        return apply(self, "img_to_tensor", dtype)

    def resize(self, mode: str, scale: Optional[tuple[float, float]] = None,
               size: Optional[tuple[int, int]] = None) -> Self:
        return apply([self], "resize", mode=mode, scale=scale, size=size)[0]
