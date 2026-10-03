from . import apply
from numpy.typing import NDArray
import numpy as np
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


def _assert_type(value, T, /):
    if not isinstance(value, T):
        raise TypeError(f"Value must be of type {str(T)}")


class BoundingBox:
    """TLWH bounding box from object detection operations.
    """

    # Raw data should be [lx, ty, w, h, score, id]
    def __init__(self, data: NDArray):
        _assert_type(data, np.ndarray)
        if not (data.ndim == 1 and data.shape[0] == 6):
            raise ValueError(
                "Bounding box data must be a flat array of length 6")
        self._data = data

    @property
    def x(self) -> Number:
        """X component of top left corner"""
        return self._data[0]

    @property
    def y(self) -> Number:
        """Y component of top left corner"""
        return self._data[1]

    @property
    def width(self) -> Number:
        """Width of box"""
        return self._data[2]

    @property
    def height(self) -> Number:
        """Height of box"""
        return self._data[3]

    @property
    def score(self) -> Number:
        """Confidence score of detection"""
        return self._data[4]

    @property
    def id(self) -> int:
        """Class id of detection"""
        return int(self._data[5])

    @x.setter
    def x(self, value: Number):
        self._data[0] = value

    @y.setter
    def y(self, value: Number):
        self._data[1] = value

    @width.setter
    def width(self, value: Number):
        self._data[2] = value

    @height.setter
    def height(self, value: Number):
        self._data[3] = value

    @score.setter
    def score(self, value: Number):
        self._data[4] = value

    @id.setter
    def id(self, value: Number):
        self._data[5] = int(value)

    def __iter__(self):
        for i in range(6):
            yield self._data[i] if i < 5 else int(self._data[i])


class Boxes:
    def __init__(self, data: NDArray):
        _assert_type(data, np.ndarray)
        if not (data.ndim == 2 and data.shape[1] == 6):
            raise ValueError(
                "Bounding boxes data must be a 2D array of shape N, 6")
        self._data = data

    def __delitem__(self, idx: int):
        self._data[idx] = self._data[-1]
        self._data = self._data[:-1]

    def __getitem__(self, idx: int):
        return BoundingBox(self._data[idx])

    def __setitem__(self, idx: int, value: BoundingBox):
        _assert_type(value, BoundingBox)
        self._data[idx] = value._data

    def __iter__(self):
        for box in self._data:
            yield BoundingBox(box)

    def __len__(self):
        return self._data.shape[0]

    def add(self, x: float, y: float, width: float, height: float, score: float, id: int):
        """This function will most likely disrupt writeback on any existing BBoxes.
        USE ONLY IF EXPLICITLY NECESSARY."""
        self._data = np.concat((self._data, np.array([[x, y, width, height, score, id]],
                                                     dtype=self._data.dtype)))

    def append(self, box: BoundingBox, /):
        _assert_type(box, BoundingBox)
        self.add(*box)

    def threshold(self, score: float, /) -> Boxes:
        return Boxes(self._data[self._data[:, 4] > score])

    def scaled(self, width: float, height: float) -> Boxes:
        new_data = self._data.copy()
        new_data[:, :4] *= np.array([width, height, width, height])
        return Boxes(new_data)

    def normalized(self, width: float, height: float) -> Boxes:
        new_data = self._data.copy()
        new_data[:, :4] /= np.array([width, height, width, height])
        return Boxes(new_data)


class BatchOfBoxes:
    def __init__(self, data: Array | NDArray):
        _assert_type(data, (Array, np.ndarray))
        if not (len(data.shape) == 3 and data.shape[2] == 6):
            raise ValueError(
                "Batch of boxes data must be a Tensor of shape B, N, 6")
        self._data = Array(data).to_numpy().data
        self._data = {i: Boxes(boxes) for i, boxes in enumerate(self._data)}

    def __delitem__(self, key: int):
        del self._data[key]

    def __getitem__(self, key: int) -> Boxes:
        return self._data[key]

    def __setitem__(self, key: int, value: Boxes):
        _assert_type(value, Boxes)
        self._data[key] = value

    def pop(self, key: int, /):
        self._data.pop(key)

    def get(self, key: int, default: any = None, /) -> Boxes | any:
        return self._data.get(key, default)

    def setdefault(self, key: int, default: Optional[Boxes] = None, /) -> Boxes:
        _assert_type(default, Boxes)
        return self._data.setdefault(key, default)

    def __iter__(self):
        for boxes in self._data.values():
            yield boxes

    def __len__(self):
        return len(self._data)

    def overwrite(self, other: BatchOfBoxes, /, start_idx: Optional[int] = None):
        if start_idx is None:
            start_idx = sorted(self._data.keys())[-1]
        for i, boxes in enumerate(other):
            self[i + start_idx] = boxes


class Keypoint:
    """Keypoint from pose estimation operations"""

    # raw data should be x, y, score, id
    # cholesky should be a 3 val array for parameters Lxx, Lxy, and Lyy, if applicable
    def __init__(self, data: NDArray, cholesky: Optional[NDArray] = None):
        if not (data.ndim == 1 and data.shape[0] == 4):
            raise ValueError(
                "Bounding box data must be a flat array of length 4")
        self._data = data
        if cholesky and not (cholesky.ndim == 1 and cholesky.shape[0] == 3):
            raise ValueError(
                "Cholesky parameters must be a flat array of length 3")
        self._cholesky = cholesky

    @property
    def x(self) -> Number:
        return self._data[0]

    @property
    def y(self) -> Number:
        return self._data[1]

    @property
    def score(self) -> Number:
        return self._data[2]

    @property
    def id(self) -> int:
        return int(self._data[3])

    @x.setter
    def x(self, value: Number):
        self._data[0] = value

    @y.setter
    def y(self, value: Number):
        self._data[1] = value

    @score.setter
    def score(self, value: Number):
        self._data[2] = value

    @id.setter
    def id(self, value: Number):
        self._data[3] = int(value)

    def __iter__(self):
        for i in range(4):
            yield self._data[i] if i < 3 else int(self._data[i])

    # Do something with cholesky later


class Pose:
    """Collection of keypoints from pose estimation operations"""

    def __init__(self, data: NDArray, cholesky: Optional[NDArray] = None):
        if not (data.ndim == 2 and data.shape[1] == 4):
            raise ValueError(
                "Pose data must be a 2D array of shape N, 4")
        self._data = data
        if cholesky and not (cholesky.ndim == 2 and cholesky.shape[1] == 3
                             and cholesky.shape[0] == data.shape[0]):
            raise ValueError(
                f"Cholesky parameters must be a 2D array of shape {data.shape[0]}, 3")
        self._cholesky = cholesky
        self._idx_map = {}

    def __getitem__(self, key) -> Keypoint:
        ...


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
            self.data = np.array([data])
        elif isinstance(data, Sequence):
            self.data = np.array(data)
        else:
            raise ValueError("Input type not supported.")

    def is_numpy(self):
        return not self.is_ort()

    def is_ort(self):
        return isinstance(self.data, OrtValue)

    def to_ort(self) -> Self:
        return apply("to_gpu", Context(self)).value

    def to_numpy(self) -> Self:
        return apply("to_cpu", Context(self)).value

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
        return apply("batch_arrays", Context(data)).value

    @staticmethod
    def concat(data: Sequence[Array], axis: int) -> Self:
        return apply("concat", Context(data), axis=axis).value

    def transpose(self, axes: Sequence[int]) -> Self:
        return apply("transpose", Context(self), axes=axes).value

    def normalize(self, axis: int, mean: float | Sequence[float] = 0.5,
                  std: float | Sequence[float] = 0.5, cast_out=True) -> Self:
        layout = [str(i) for i in range(len(self.shape))]
        layout[axis] = 'c'
        return apply("normalize", Context(self), mean=mean, std=std,
                     layout=layout, cast_out=cast_out).value

    def gather(self, indices: Sequence[int], axis: int) -> Self:
        return apply("gather", Context(self), indices=indices, axis=axis).value

    def slice(self, starts: Sequence[int], ends: Sequence[int],
              axes: Sequence[int], steps: Sequence[int]) -> Self:
        return apply("slice", Context(self), starts=starts, ends=ends, axes=axes, steps=steps).value

    def cast(self, dtype: DType) -> Self:
        return apply("cast", Context(self), dtype=DType(dtype)).value

    def sub(self, other: Tensor) -> Self:
        return apply("sub", Context(self, other=other)).value

    def add(self, other: Tensor) -> Self:
        return apply("add", Context(self, other=other)).value

    def div(self, other: Tensor) -> Self:
        return apply("div", Context(self, other=other)).value

    def mul(self, other: Tensor) -> Self:
        return apply("mul", Context(self, other=other)).value

    def __getitem__(self, key):
        print(key)


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
        return apply("channel_swap", Context(self)).value

    def normalize(self, mean: float | Sequence[float] = 0.5, std: float | Sequence[float] = 0.5) -> Self:
        return Image(apply("normalize", Context(self), mean=mean, std=std, layout=self.layout).value,
                     self.color_format, self.layout)

    def to_tensor(self, dtype: np.dtype | str) -> Tensor:
        return apply("img_to_tensor", Context(self), dtype).value

    def resize(self, method: str, scale: Optional[tuple[float, float]] = None,
               size: Optional[tuple[int, int]] = None) -> Self:
        return apply("resize", Context(self), method=method, scale=scale, size=size).value

    def cast(self, dtype: DType, scale: Number | str = "auto") -> Self:
        return apply("img_cast", Context(self), dtype=DType(dtype), scale=scale).value


class Context:
    def __init__(self, /, value: Optional[any] = None, **kwargs):
        self._prev = value
        for k, v in kwargs.items():
            self[k] = v
        self._in = "_prev"
        self._out = "_prev"

    @staticmethod
    def _check_key(key):
        if not isinstance(key, str):
            raise KeyError("Key must be a string")

    def __setitem__(self, key, value):
        self._check_key(key)
        setattr(self, key, value)

    def __getitem__(self, key):
        self._check_key(key)
        return getattr(self, key)

    def __contains__(self, key):
        self._check_key(key)
        return hasattr(self, key)

    @property
    def value(self):
        return self[self._in]

    def update(self, value: any):
        self[self._out] = value
        self._in = self._out

    def expect(self, *args):
        for key in args:
            if key not in self:
                raise ValueError(f'Context missing expected value "{key}"')

    def expect_type(self, **kwargs):
        for k, v in kwargs.items():
            self.expect(k)
            if not isinstance(self[k], v):
                raise TypeError(f'Expected type {str(v)} for context value "{k}"'
                                f', got {str(type(self[k]))}')

    def expect_seq_type(self, **kwargs):
        for k, v in kwargs.items():
            self.expect_type(**{k: Sequence})
            if isinstance(self[k], str):
                raise ValueError(
                    f'Context value "{k}" must be a non-string sequence')
            for item in self[k]:
                if not isinstance(item, v):
                    raise TypeError(f'Type mismatch for value sequence "{str(k)}" in context:'
                                    f' expected {str(v)}, got {str(type(item))}')
