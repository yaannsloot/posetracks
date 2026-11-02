from . import apply
from numpy.typing import NDArray
import numpy as np
from onnxruntime import OrtValue
import OpenImageIO as oiio
from typing import Sequence, Self, Optional
from onnx import TensorProto
from onnx.helper import np_dtype_to_tensor_dtype, tensor_dtype_to_np_dtype
from typing import Iterable, Type, TypeVar
from abc import ABC, abstractmethod

T = TypeVar("T")

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


def _assert_type(value: any, ty: Type, /):
    if not isinstance(value, ty):
        raise TypeError(f"Value must be of type {str(ty)}")


def _assert_itertype(value: Iterable[any], ty: Type):
    for i in value:
        _assert_type(i, ty)


def _assert_axis_eq(*, axis=0, **arrays: Array | NDArray):
    dim = None
    for arr in arrays.values():
        if dim is None:
            dim = arr.shape[axis]
        elif arr.shape[axis] != dim:
            raise ValueError(
                f"Axis {axis} must be equal for all provided arrays.")


class Constraint:
    def validate(self, value):
        ...


class ShapeConstraint(Constraint):
    def __init__(self, shape: tuple[int], axes: Optional[tuple[int]] = None):
        _assert_itertype(shape, int)
        if axes is not None:
            _assert_itertype(axes, int)
        if axes and len(shape) != len(axes):
            raise ValueError("Shape and axes arguments "
                             "must be the same length.")
        self._shape = shape
        self._axes = axes

    def validate(self, value: Array | NDArray):
        shape = value.shape
        if self._axes is not None:
            shape = [shape[i] for i in self._axes]
        if not all(a == b for a, b in zip(shape, self._shape)):
            raise ValueError(f"In array of shape {value.shape}, "
                             f"expected dims {tuple(self._shape)}, got {shape}.")


class ARankConstraint(Constraint):
    def __init__(self, n_dims: int):
        _assert_type(n_dims, int)
        self._n_dims = n_dims

    def validate(self, value: Array | NDArray):
        if len(value.shape) != self._n_dims:
            raise ValueError(f"Expected an array of rank {self._n_dims}, "
                             f"got {len(value.shape)}.")


class ConstraintList(list[Constraint]):
    def validate(self, value):
        for item in self:
            item.validate(value)


class DataView(ABC):
    # Must be formatted as val_name: constraints
    # Mirrors kwargs layout in array methods
    _CONSTRAINTS: dict[str, Constraint | ConstraintList] = {}

    @classmethod
    def _verify_constraints(cls, **arrays: Array | NDArray):
        if not cls._CONSTRAINTS:
            return
        for name, constraint in cls._CONSTRAINTS.items():
            constraint.validate(arrays[name])

    @classmethod
    def _split_arrays(cls, **arrays: Array | NDArray):
        _assert_itertype(arrays.values(), (Array, np.ndarray))
        _assert_axis_eq(**arrays)
        out = []
        n = next(iter(arrays.values())).shape[0]
        for i in range(n):
            out.append(
                {k: Array(v).to_numpy().data[i] for k, v in arrays.items()})
        return out

    @staticmethod
    def _init_item(**arrays: Array | NDArray):
        return arrays

    @classmethod
    @abstractmethod
    def from_arrays(cls, **arrays: Array | NDArray):
        ...


class DataSequence(dict[int, T], DataView):
    def overwrite(self, other: Self, /, start_idx: Optional[int] = None):
        if start_idx is None:
            start_idx = sorted(self._data.keys())[-1]
        for i, item in enumerate(other):
            self[i + start_idx] = item

    @classmethod
    def from_arrays(cls, **arrays: Array | NDArray):
        cls._verify_constraints(**arrays)
        return cls({i: cls._init_item(**v) for i, v
                    in enumerate(cls._split_arrays(**arrays))})


class DataMap(dict[str, T], DataView):
    @classmethod
    def from_arrays(cls, **arrays: Array | NDArray):
        cls._verify_constraints(**arrays)
        return cls({str(i): cls._init_item(**v) for i, v
                    in enumerate(cls._split_arrays(**arrays))})


class ThresholdedDetections:
    "Mixin for any KV class whose values have a score attribute"

    def threshold(self, score: float, /) -> Self:
        return type(self)({k: v for k, v in self.items() if v.score >= score})


class ScalableDetections:
    "Mixin for any KV class whose values have scaled and normalized functions"

    def scaled(self, width: float, height: float, /) -> Self:
        return type(self)({k: v.scaled(width, height) for k, v in self.items()})

    def normalized(self, width: float, height: float, /) -> Self:
        return type(self)({k: v.normalized(width, height) for k, v in self.items()})


class BoundingBox(DataView):
    """TLWH bounding box from object detection operations.
    """

    _CONSTRAINTS = {
        "data": ConstraintList([
            ARankConstraint(1),
            ShapeConstraint([6])
        ])
    }

    def __init__(self, x, y, width, height, score, id):
        self._values = [0] * 6
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.score = score
        self.id = id

    def __setitem__(self, idx: int, value: Number):
        if idx == 5 or idx == -6:
            self._values[idx] = int(value)
        else:
            self._values[idx] = float(value)

    def __getitem__(self, idx: int) -> Number:
        return self._values[idx]

    def __iter__(self):
        return iter(self._values)

    @property
    def x(self) -> float:
        """X component of top left corner"""
        return self[0]

    @property
    def y(self) -> float:
        """Y component of top left corner"""
        return self[1]

    @property
    def width(self) -> float:
        """Width of box"""
        return self[2]

    @property
    def height(self) -> float:
        """Height of box"""
        return self[3]

    @property
    def score(self) -> float:
        """Confidence score of detection"""
        return self[4]

    @property
    def id(self) -> int:
        """Class id of detection"""
        return self[5]

    @x.setter
    def x(self, value: Number):
        self[0] = value

    @y.setter
    def y(self, value: Number):
        self[1] = value

    @width.setter
    def width(self, value: Number):
        self[2] = value

    @height.setter
    def height(self, value: Number):
        self[3] = value

    @score.setter
    def score(self, value: Number):
        self[4] = value

    @id.setter
    def id(self, value: Number):
        self[5] = value

    def scaled(self, width: Number, height: Number, /) -> Self:
        return type(self)(x=self.x * width,
                          y=self.y * height,
                          width=self.width * width,
                          height=self.height * height,
                          score=self.score,
                          id=self.id)

    def normalized(self, width: Number, height: Number, /) -> Self:
        return type(self)(x=self.x / width,
                          y=self.y / height,
                          width=self.width / width,
                          height=self.height / height,
                          score=self.score,
                          id=self.id)

    @classmethod
    def from_arrays(cls, **arrays):
        cls._verify_constraints(**arrays)
        return cls(*arrays["data"])

    def __repr__(self):
        return (f"BoundingBox(x={self.x}, y={self.y}, width={self.width}, height={self.height}, "
                f"score={self.score}, id={self.id})")


class Boxes(DataMap[BoundingBox], ScalableDetections, ThresholdedDetections):
    _CONSTRAINTS = {
        "data": ConstraintList([
            ARankConstraint(2),
            ShapeConstraint([6], [1])
        ])
    }

    @staticmethod
    def _init_item(**arrays):
        return BoundingBox.from_arrays(**arrays)


class BatchOfBoxes(DataSequence[Boxes], ScalableDetections):
    _CONSTRAINTS = {
        "data": ConstraintList([
            ARankConstraint(3),
            ShapeConstraint([6], [2])
        ])
    }

    @staticmethod
    def _init_item(**arrays):
        return Boxes.from_arrays(**arrays)


class Keypoint(DataView):
    """Keypoint from pose estimation operations"""

    # raw data should be x, y, score, id
    # cholesky should be a 3 val array for parameters Lxx, Lxy, and Lyy, if applicable

    _CONSTRAINTS = {
        "data": ConstraintList([
            ARankConstraint(1),
            ShapeConstraint([4])
        ]),
        "cholesky": ConstraintList([
            ARankConstraint(1),
            ShapeConstraint([3])
        ]),
    }

    def __init__(self, x, y, score, id, **kwargs):
        self._values = [0] * 4
        self.x = x
        self.y = y
        self.score = score
        self.id = id
        self.extra_vals = kwargs

    def __setitem__(self, idx: int, value: Number):
        if idx == 3 or idx == -4:
            self._values[idx] = int(value)
        else:
            self._values[idx] = float(value)

    def __getitem__(self, idx: int) -> Number:
        return self._values[idx]

    def __iter__(self):
        return iter(self._values)

    @property
    def x(self) -> Number:
        return self[0]

    @property
    def y(self) -> Number:
        return self[1]

    @property
    def score(self) -> Number:
        return self[2]

    @property
    def id(self) -> int:
        return self[3]

    @x.setter
    def x(self, value: Number):
        self[0] = value

    @y.setter
    def y(self, value: Number):
        self[1] = value

    @score.setter
    def score(self, value: Number):
        self[2] = value

    @id.setter
    def id(self, value: Number):
        self[3] = value

    def __iter__(self):
        yield from self._values

    def scaled(self, width: Number, height: Number, /) -> Self:
        return type(self)(self.x * width,
                          self.y * height,
                          self.score,
                          self.id,
                          **self.extra_vals)

    def normalized(self, width: Number, height: Number, /) -> Self:
        return type(self)(self.x / width,
                          self.y / height,
                          self.score,
                          self.id,
                          **self.extra_vals)

    @classmethod
    def from_arrays(cls, **arrays):
        cls._verify_constraints(**arrays)
        return cls(*arrays["data"], **{k: v for k, v in arrays.items() if k != "data"})

    def __repr__(self):
        return (f"BoundingBox(x={self.x}, y={self.y}, score={self.score}, id={self.id})")

    # Do something with cholesky later


class Pose(DataMap[Keypoint], ScalableDetections, ThresholdedDetections):
    """Collection of keypoints from pose estimation operations"""

    _CONSTRAINTS = {
        "data": ConstraintList([
            ARankConstraint(2),
            ShapeConstraint([4], [1])
        ]),
        "cholesky": ConstraintList([
            ARankConstraint(2),
            ShapeConstraint([3], [1])
        ]),
    }

    @staticmethod
    def _init_item(**arrays):
        return Keypoint.from_arrays(**arrays)


class Poses(DataMap[Pose], ScalableDetections, ThresholdedDetections):
    """Collection of poses from pose estimation operations"""

    _CONSTRAINTS = {
        "data": ConstraintList([
            ARankConstraint(3),
            ShapeConstraint([4], [2])
        ]),
        "cholesky": ConstraintList([
            ARankConstraint(3),
            ShapeConstraint([3], [2])
        ]),
    }

    def threshold(self, score):
        out = {}
        for k, v in self.items():
            v = v.threshold(score)
            if not v:
                continue
            out[k] = v
        return type(self)(out)

    @staticmethod
    def _init_item(**arrays):
        return Pose.from_arrays(**arrays)


class BatchOfPoses(DataSequence[Poses], ScalableDetections):
    _CONSTRAINTS = {
        "data": ConstraintList([
            ARankConstraint(4),
            ShapeConstraint([4], [3])
        ]),
        "cholesky": ConstraintList([
            ARankConstraint(4),
            ShapeConstraint([3], [3])
        ])
    }

    @staticmethod
    def _init_item(**arrays):
        return Poses.from_arrays(**arrays)


class DType:
    """Backend-agnostic wrapper around an element data type.

    Accepts a NumPy dtype, a dtype string (e.g. ``"float32"``), an ONNX
    ``TensorProto.DataType`` value, an ONNXRuntime type string, or another
    ``DType``, and normalizes it to a NumPy dtype. The normalized type can be
    read back in any of those formats with ``as_numpy``, ``as_onnx`` and
    ``as_ort``. Instances compare equal to anything with the same string form
    and are hashable, so they can be used in comparisons, sets and dict keys.
    """

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
        """Stringified NumPy representation of this datatype."""
        return str(self.np_dtype)

    def as_numpy(self):
        """Returns this datatype in NumPy format."""
        return self.np_dtype

    def as_onnx(self):
        """Returns this datatype in ONNX TensorProto format."""
        return np_dtype_to_tensor_dtype(self.np_dtype)

    def as_ort(self):
        """Returns this datatype in ONNXRuntime format."""
        return NP_TYPE_TO_ORT_TYPE[self.value]

    @staticmethod
    def promote(a: DType, b: DType, /):
        """Returns the data type with the smallest size and smallest 
        scalar kind to which both ``a`` and ``b`` may be safely cast."""
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
    """Managed array for CPU/GPU memory.

    Wraps a NumPy array (CPU backend) or an ONNXRuntime ``OrtValue`` (GPU
    backend) behind one interface, and can be built from either of those, from
    another ``Array``, from a sequence of numbers, or from a single number
    (stored as a one-element array). Use ``to_ort``, ``to_numpy`` or ``to`` to
    move data between devices, and ``dtype``, ``shape``, ``device`` and
    ``backend`` to inspect it. This is the base class for ``Tensor`` and
    ``Image``.
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
        """Returns true when this array is mapped to the NumPy CPU backend."""
        return not self.is_ort()

    def is_ort(self):
        """Returns true when this array is mapped to the ONNXRuntime GPU backend."""
        return isinstance(self.data, OrtValue)

    def to_ort(self) -> Self:
        """Transfer to ONNXRuntime GPU backend, returning the resulting GPU array. If already on ORT, returns this array."""
        return apply("to_gpu", Context(self)).value

    def to_numpy(self) -> Self:
        """Transfer to NumPy CPU backend, returning the resulting CPU array. If already on NumPy, returns this array."""
        return apply("to_cpu", Context(self)).value

    def to(self, device: str) -> Self:
        """Transfer to target device. Will always default to CPU if device is not ``"ort" or "gpu"``."""
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
    """Array intended as model input or output, with data-manipulation ops.

    Adds the operations used to assemble inputs and post-process outputs:
    batching and concatenation, transposing, normalization, slicing and
    gathering, casting, argmax and max reduction, sigmoid, and elementwise
    arithmetic.
    """

    @staticmethod
    def batch_arrays(data: Sequence[Array]) -> Self:
        """Concatenates a list of Arrays along a new batch dimension.

        All arrays must have the same shape and dtype. The result has shape
        ``(len(data), *shape)``.

        Args:
            data: Arrays to stack. The batch index follows the order of the list.

        Returns:
            A tensor with a new leading batch axis.
        """
        return apply("batch_arrays", Context(data)).value

    @staticmethod
    def concat(data: Sequence[Array], axis: int) -> Self:
        """Concatenate a list of arrays along an existing dimension.

        Unlike ``batch_arrays``, no new axis is created. All arrays must have
        the same rank and matching sizes on every axis except ``axis``.

        Args:
            data: Arrays to join, in order.
            axis: Existing axis along which to join. Negative values count
                from the last axis.

        Returns:
            A tensor whose size on ``axis`` is the sum of the inputs' sizes.
        """
        return apply("concat", Context(data), axis=axis).value

    def transpose(self, axes: Sequence[int]) -> Self:
        """Transpose this tensor based on the order of the provided axes.

        Args:
            axes: A permutation of ``range(ndim)``. Output axis ``i`` is
                input axis ``axes[i]``.

        Returns:
            A tensor with permuted dimensions.
        """
        return apply("transpose", Context(self), axes=axes).value

    def normalize(self, axis: int = 0, mean: float | Sequence[float] = 0.5,
                  std: float | Sequence[float] = 0.5) -> Self:
        """Normalize the values of this tensor, computing ``(x - mean) / std``.

        If mean and/or std are a sequence of numbers, axis must be specified with a
        dimension that matches the sequence length (e.g. the channel axis for
        per-channel ImageNet statistics).

        Args:
            axis: Axis that the per-element ``mean``/``std`` values apply to.
                Ignored when both are scalars.
            mean: Scalar, or one value per entry along ``axis``.
            std: Scalar, or one value per entry along ``axis``.

        Returns:
            A tensor of the same shape, normalized.
        """
        layout = [str(i) for i in range(len(self.shape))]
        layout[axis] = 'c'
        return apply("normalize", Context(self), mean=mean, std=std,
                     layout=layout).value

    def gather(self, indices: Sequence[int], axis: int) -> Self:
        """Recombine the entries on the provided axis based on a list of indices.

        Output entry ``i`` along ``axis`` is input entry ``indices[i]``. Indices
        may repeat or be reordered, so the output size on ``axis`` is
        ``len(indices)``.

        Args:
            indices: Positions along ``axis`` to select.
            axis: Axis to select from.

        Returns:
            A tensor with the selected entries on ``axis``.
        """
        return apply("gather", Context(self), indices=indices, axis=axis).value

    def slice(self, starts: Sequence[int], ends: Sequence[int],
              axes: Sequence[int], steps: Sequence[int]) -> Self:
        """Extract a sub-region of this tensor along the given axes.

        The four sequences are parallel: for each ``i``, axis ``axes[i]`` is
        sliced as ``[starts[i]:ends[i]:steps[i]]``. ``ends`` is exclusive.
        Axes not listed are left whole.

        Args:
            starts: Start index per sliced axis (inclusive).
            ends: End index per sliced axis (exclusive).
            axes: Axes being sliced.
            steps: Stride per sliced axis. Use 1 for contiguous slices.

        Returns:
            A tensor with the sliced region.
        """
        return apply("slice", Context(self), starts=starts, ends=ends, axes=axes, steps=steps).value

    def cast(self, dtype: DType) -> Self:
        """Convert this tensor's elements to another data type.

        Args:
            dtype: Target type, e.g. ``"float32"``.

        Returns:
            A tensor with the same shape and converted values.
        """
        return apply("cast", Context(self), dtype=DType(dtype)).value

    def argmax(self, axis: int) -> Self:
        """Return the index of the largest value along an axis.

        Args:
            axis: Axis to reduce.

        Returns:
            Integer indices.
        """
        return apply("argmax", Context(self), axis=axis).value

    def reducemax(self, axes: Sequence[int]) -> Self:
        """Return the maximum value over the given axes.

        Args:
            axes: Axes to reduce.

        Returns:
            The reduced tensor.
        """
        return apply("reducemax", Context(self), axes=axes).value

    def sigmoid(self) -> Self:
        """Apply the logistic function ``1 / (1 + exp(-x))`` elementwise.

        Maps logits to probabilities in ``(0, 1)``.

        Returns:
            A tensor of the same shape.
        """
        return apply("sigmoid", Context(self)).value

    def unsqueeze(self, axes: Sequence[int]) -> Self:
        """Insert new size-one dimensions at the provided positions.

        For each ``i`` in ``axes``, an axis of size one is added at position
        ``i``. The output has ``len(axes)`` more dimensions than this tensor.

        Args:
            axes: Positions of the new size-one axes. Negative values count
                from the last axis.

        Returns:
            A tensor with an additional size-one axis at each position.
        """
        return apply("unsqueeze", Context(self), axes=axes).value

    def sub(self, other: Tensor) -> Self:
        """Subtract ``other`` from this tensor elementwise (``self - other``).

        Args:
            other: Tensor or scalar, broadcastable to this tensor's shape.

        Returns:
            The elementwise difference.
        """
        return apply("sub", Context(self, other=other)).value

    def add(self, other: Tensor) -> Self:
        """Add ``other`` to this tensor elementwise (``self + other``).

        Args:
            other: Tensor or scalar, broadcastable to this tensor's shape.

        Returns:
            The elementwise sum.
        """
        return apply("add", Context(self, other=other)).value

    def div(self, other: Tensor) -> Self:
        """Divide this tensor by ``other`` elementwise (``self / other``).

        Args:
            other: Tensor or scalar divisor, broadcastable to this tensor's shape.

        Returns:
            The elementwise quotient.
        """
        return apply("div", Context(self, other=other)).value

    def mul(self, other: Tensor) -> Self:
        """Multiply this tensor by ``other`` elementwise (``self * other``).

        Args:
            other: Tensor or scalar, broadcastable to this tensor's shape.

        Returns:
            The elementwise product.
        """
        return apply("mul", Context(self, other=other)).value


class Image(Array):
    """Array holding image pixels, tagged with a color format and layout.

    Carries ``color_format`` (e.g. ``"rgb"``, ``"rgba"``, ``"gray"``) and
    ``layout`` (e.g. ``"hwc"``) alongside the pixel data, so operations know
    how to interpret it. Provides loading from file (``open``) and the
    preprocessing steps used before inference: channel swapping, resizing,
    casting, normalization, and conversion to an RGB NCHW ``Tensor`` with
    ``to_tensor``.
    """

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
        """Open image file specified in provided path."""
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
        """Swap channel format from RGB ro BGR or vice versa."""
        return apply("channel_swap", Context(self)).value

    def normalize(self, mean: float | Sequence[float] = 0.5, std: float | Sequence[float] = 0.5) -> Self:
        """Normalize the values of this image, computing ``(x - mean) / std``.

        Args:
            mean: Scalar, or one value per entry along ``axis``.
            std: Scalar, or one value per entry along ``axis``.

        Returns:
            A copy of this image with normalized values.
        """
        return Image(apply("normalize", Context(self), mean=mean, std=std, layout=self.layout).value,
                     self.color_format, self.layout)

    def to_tensor(self, dtype: np.dtype | str) -> Tensor:
        """Convert this image with HWC layout and potentially BGR color format to an RGB NCHW image tensor.

        Args:
            dtype: Element type of the resulting tensor, e.g. ``"float32"``.

        Returns:
            A tensor of shape ``(1, C, H, W)`` in RGB order.
        """
        return apply("img_to_tensor", Context(self), dtype).value

    def resize(self, method: str, scale: Optional[tuple[float, float]] = None,
               size: Optional[tuple[int, int]] = None) -> Self:
        """Resize this image, either to an explicit size or by a scale factor.

        Provide at least one of ``scale`` or ``size``.
        Will default to ``size`` if both are provided.

        Valid interpolation methods are:
        - ``"nearest"``
        - ``"linear"``
        - ``"cubic"``
        - ``"area"`` - CPU ONLY
        - ``"lanczos"`` - CPU ONLY

        CPU only methods will require a round trip swap from GPU -> CPU -> GPU if the image is currently on GPU.

        Args:
            method: Interpolation method, e.g. ``"nearest"`` or ``"linear"``.
            scale: Scale factors applied to the image dimensions. Output
                dimensions are the input dimensions times these factors.
            size: Target output dimensions in pixels.

        Returns:
            The resized image with the same color format and layout.
        """
        return apply("resize", Context(self), method=method, scale=scale, size=size).value

    def cast(self, dtype: DType, scale: Number | str = "auto") -> Self:
        """Convert this image's pixel values to another data type.

        Args:
            dtype: Target element type, e.g. ``"uint8"`` or ``"float32"``.
            scale: Factor applied to the values during conversion. With
                ``"auto"``, values are rescaled to match the new type's
                conventional range, e.g. ``uint8`` 0-255 to ``float32`` 0-1
                and back. A number is used directly as the multiplier.

        Returns:
            The converted image.
        """
        return apply("img_cast", Context(self), dtype=DType(dtype), scale=scale).value


class Context:
    """Holds the inputs and outputs of a single operation.

    A Context wraps one primary value plus any number of named values, all
    stored as plain attributes on the instance. Operations read their inputs
    with ``ctx.value`` or ``ctx["name"]``, validate them with the ``expect*``
    helpers, and write their result back with ``update``.

    The primary value lives under the attribute name ``_prev``. Keyword
    arguments become additional named attributes, so they should not collide
    with the names of this class's own members (``value``, ``update``,
    ``expect``, ``expect_type``, ``expect_seq_type``) or with ``_prev``,
    ``_in`` and ``_out``.

    Example:
        ``Context(image, mean=0.5, std=0.5)`` has the image as its primary
        value and ``ctx["mean"]`` and ``ctx["std"]`` as named values.
    """

    def __init__(self, /, value: Optional[any] = None, **kwargs):
        self._prev = value
        for k, v in kwargs.items():
            self[k] = v
        self._in = "_prev"
        self._out = "_prev"

    def __setitem__(self, key, value):
        _assert_type(key, str)
        setattr(self, key, value)

    def __getitem__(self, key):
        _assert_type(key, str)
        return getattr(self, key)

    def __contains__(self, key):
        _assert_type(key, str)
        return hasattr(self, key)

    @property
    def value(self):
        """The current main value of this context.

        Reads the entry named by the input pointer ``_in``. Until
        ``update`` has been called this is the primary value passed to the
        constructor, and afterwards it is the most recently stored result.
        """
        return self[self._in]

    def update(self, value: any):
        """Store a new result as this context's main value.

        Writes ``value`` to the entry named by the output pointer ``_out``,
        then points the input pointer at it, so subsequent reads of ``value``
        return the new result.

        Args:
            value: The result to store.
        """
        self[self._out] = value
        self._in = self._out

    def expect(self, *args):
        """Check that named values are present.

        Args:
            *args: Names that must exist in this context.

        Raises:
            ValueError: If any name is missing. Presence is all that is
                checked, so a value of ``None`` passes.
        """
        for key in args:
            if key not in self:
                raise ValueError(f'Context missing expected value "{key}"')

    def expect_type(self, **kwargs):
        """Check that named values are present and have the given types.

        Args:
            **kwargs: Mapping of value name to expected type. A tuple of
                types is also accepted.

        Raises:
            ValueError: If a named value is missing.
            TypeError: If a value is not an instance of its expected type.

        Example:
            ``ctx.expect_type(axis=int, mean=(int, float))``
        """
        for k, v in kwargs.items():
            self.expect(k)
            if not isinstance(self[k], v):
                raise TypeError(f'Expected type {str(v)} for context value "{k}"'
                                f', got {str(type(self[k]))}')

    def expect_seq_type(self, **kwargs):
        """Check that named values are sequences whose items have the given type.

        Each named value must be a sequence other than a string, and every
        item in it must be an instance of the expected item type.

        Args:
            **kwargs: Mapping of value name to the expected type of its
                items. A tuple of types is also accepted.

        Raises:
            ValueError: If a named value is missing, or is a string.
            TypeError: If a value is not a sequence, or if any item has the
                wrong type.

        Example:
            ``ctx.expect_seq_type(starts=int, ends=int)``
        """
        for k, v in kwargs.items():
            self.expect_type(**{k: Sequence})
            if isinstance(self[k], str):
                raise ValueError(
                    f'Context value "{k}" must be a non-string sequence')
            for item in self[k]:
                if not isinstance(item, v):
                    raise TypeError(f'Type mismatch for value sequence "{str(k)}" in context:'
                                    f' expected {str(v)}, got {str(type(item))}')
