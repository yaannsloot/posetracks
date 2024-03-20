"""
Deep neural network module
"""

'''
Copyright (C) 2024 Ian Sloat

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
'''

import typing
from typing import ClassVar, List, Tuple, Set, Union, overload
from . import Mat, Point, Rect, TagDictionary

AUTO: ScalingMode
CPU: Executor
CUDA: Executor
DIRECT: ScalingMode
EUCLIDEAN: FeatureDistanceType
FLOAT16: Precision
FLOAT32: Precision
FLOAT64: Precision
INT16: Precision
INT32: Precision
INT64: Precision
INT8: Precision
NONE: Executor
NORMALIZE_INPUT: ScalingMode
NORM_EUCLIDEAN: FeatureDistanceType
TENSORRT: Executor
UNKNOWN: Precision

class CVTagDetector(TagModel):
    """
    Dummy module that uses OpenCV's ArUco detector for tag detection
    """
    def __init__(self) -> None:
        """
        Create a new CVTagDetector instance
        """
    def set_dict_type(self, arg0: TagDictionary) -> None:
        """
        Set the dictionary type used by the detector
        :param arg0: Predefined tag dictionary
        """
    def set_preprocess_size(self, arg0: Tuple[int, int]) -> None:
        """
        If set, resizes the sampled region before detection
        :param arg0: Preprocess size of forwarded samples
        """

class Detection:
    bbox: Rect
    class_id: int
    score: float
    @overload
    def __init__(self) -> None:
        """
        Create a new empty detection
        """
    @overload
    def __init__(self, class_id: int, bbox: Rect, score: float) -> None:
        """
        Create a new detection
        :param class_id: Object class id of the detection
        :param bbox: Bounding box of the detection
        :param score: Detection confidence score
        """
    def scale_detection(self, arg0: float) -> None:
        """
        Scale bounds of detection
        :param arg0: Scale factor
        """

class DetectionModel(ImageModel):
    def __init__(self) -> None:
        """
        Create a new detection model instance. This is a base class and has no inference capabilities.
        """
    @overload
    def infer(self, image: Mat, conf_thresh: float = 0.5, iou_thresh: float = 0.5) -> List[Detection]:
        """
        Run a forward pass on a single image
        :param image: Image to send through the model
        :param conf_thresh: Confidence threshold of detections
        :param iou_thresh: IoU threshold of bounding boxes
        """
    @overload
    def infer(self, images: List[Mat], conf_thresh: float = 0.5, iou_thresh: float = 0.5) -> List[List[Detection]]:
        """
        Run a forward pass on a batch of images
        :param images: Images to send through the model
        :param conf_thresh: Confidence threshold of detections
        :param iou_thresh: IoU threshold of bounding boxes
        """

class Executor:
    __members__: ClassVar[dict]
    CPU: ClassVar[Executor]
    CUDA: ClassVar[Executor]
    NONE: ClassVar[Executor]
    TENSORRT: ClassVar[Executor]
    def __init__(self, value: int) -> None: ...
    def __eq__(self, other: object) -> bool: ...
    def __hash__(self) -> int: ...
    def __index__(self) -> int: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: object) -> bool: ...
    @property
    def name(self) -> str: ...
    @property
    def value(self) -> int: ...

class Feature:
    @overload
    def __init__(self) -> None: ...
    @overload
    def __init__(self, arg0: List[float]) -> None: ...
    def dist(self, other: Feature, d_type: FeatureDistanceType = NORM_EUCLIDEAN) -> float:
        """
        Get the distance between two feature vectors.

        If the distance type is EUCLIDEAN, the return value is the euclidean distance between
        this feature vector and the provided feature vector.

        If the distance type is NORM_EUCLIDEAN, the feature vectors are scaled to unit length before
        their euclidean distance is calculated
        :param other: Other feature to compare to
        :param d_type: Distance type to calculate
        """
    def norm(self) -> float:
        """
        Get the L2 norm of this feature vector
        """
    def __assign__(self, arg0: Feature) -> Feature: ...
    def __getitem__(self, arg0: int) -> float: ...
    def __iter__(self) -> typing.Iterator[float]: ...
    def __len__(self) -> int: ...
    def __setitem__(self, arg0: int, arg1: float) -> None: ...
    def __sub__(self, arg0: Feature) -> Feature: ...
    def __truediv__(self, arg0: float) -> Feature: ...

class FeatureDistanceType:
    __members__: ClassVar[dict]
    EUCLIDEAN: ClassVar[FeatureDistanceType]
    NORM_EUCLIDEAN: ClassVar[FeatureDistanceType]
    def __init__(self, value: int) -> None: ...
    def __eq__(self, other: object) -> bool: ...
    def __hash__(self) -> int: ...
    def __index__(self) -> int: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: object) -> bool: ...
    @property
    def name(self) -> str: ...
    @property
    def value(self) -> int: ...

class FeatureModel(ImageModel):
    def __init__(self) -> None:
        """
        Create a new feature model instance. This is a base class and has no inference capabilities.
        """
    @overload
    def infer(self, arg0: Mat) -> Feature:
        """
        Run a forward pass on a single image
        :param arg0: Image to send through the model
        """
    @overload
    def infer(self, arg0: List[Mat]) -> List[Feature]:
        """
        Run a forward pass on a batch of images
        :param arg0: Images to send through the model
        """

class GenericFeatureModel(FeatureModel):
    def __init__(self) -> None:
        """
        Create a new generic feature model instance
        """

class ImageModel(Model):
    def __init__(self) -> None:
        """
        Create a new image model instance. This is a base class and has no inference capabilities.
        """
    def net_size(self) -> Tuple[int, int]:
        """
        Get the input size of the model
        """

class Joint:
    prob: float
    pt: Point
    @overload
    def __init__(self) -> None:
        """
        Create a new empty joint
        """
    @overload
    def __init__(self, arg0: float, arg1: float, arg2: float) -> None:
        """
        Create a new joint
        :param arg0: X coordinate of joint
        :param arg1: Y coordinate of joint
        :param arg2: Confidence score of joint
        """
    @overload
    def __init__(self, arg0: Point, arg1: float) -> None:
        """
        Create a new joint
        :param arg0: Joint position
        :param arg1: Confidence score of joint
        """

class Model:
    def __init__(self) -> None:
        """
        Create a new model instance. This is a base class and has no inference capabilities.
        """
    def get_executor(self) -> Executor:
        """
        Get the target executor of this model
        """
    def get_precision(self) -> Precision:
        """
        Get the unit precision of this model
        """
    def is_loaded(self) -> bool:
        """
        Check if the model is loaded
        """
    def load(self, model_path: str, target_executor: Executor = CPU) -> None:
        """
        Load a model
        :param model_path: Path to model file
        :param target_executor: Target model executor
        """
    def unload(self) -> None:
        """
        Unload the model
        """

class Pose:
    def __init__(self) -> None:
        """
        Create a new empty pose
        """
    def get_joint(self, arg0: int) -> Joint:
        """
        Retrieve a joint by id. If one does not exist, a new joint will be added
        :param arg0: Joint ID
        """
    def get_joint_ids(self) -> Set[int]:
        """
        Get a list of all current joint ids within this pose
        """
    def has_joint(self, arg0: int) -> bool:
        """
        Check if this pose contains a joint with the provided id
        :param arg0:
        """
    def num_joints(self) -> int:
        """
        Get the total number of joints contained inside this pose
        """
    @overload
    def set_joint(self, arg0: int, arg1: Joint) -> None:
        """
        Set joint data for a given joint id
        :param arg0: Joint ID
        :param arg1: Joint to assign to ID
        """
    @overload
    def set_joint(self, arg0: int, arg1: Point, arg2: float) -> None:
        """
        Set joint data for a given joint id
        :param arg0: Joint ID
        :param arg1: Joint position
        :param arg2: Joint confidence score
        """
    @overload
    def set_joint(self, arg0: int, arg1: float, arg2: float, arg3: float) -> None:
        """
        Set joint data for a given joint id
        :param arg0: Joint ID
        :param arg1: Joint X coordinate
        :param arg2: Joint Y coordinate
        :param arg3: Joint confidence score
        """
    def __getitem__(self, arg0: int) -> Joint: ...
    def __iter__(self) -> typing.Iterator[Joint]: ...

class PoseModel(ImageModel):
    def __init__(self) -> None:
        """
        Create a new pose model instance. This is a base class and has no inference capabilities.
        """
    @overload
    def infer(self, arg0: Mat) -> Pose:
        """
        Run a forward pass on a single image
        :param arg0: Image to send through the model
        """
    @overload
    def infer(self, arg0: List[Mat]) -> List[Pose]:
        """
        Run a forward pass on a batch of images
        :param arg0: Images to send through the model
        """

class Precision:
    __members__: ClassVar[dict]
    FLOAT16: ClassVar[Precision]
    FLOAT32: ClassVar[Precision]
    FLOAT64: ClassVar[Precision]
    INT16: ClassVar[Precision]
    INT32: ClassVar[Precision]
    INT64: ClassVar[Precision]
    INT8: ClassVar[Precision]
    NONE: ClassVar[Precision]
    UNKNOWN: ClassVar[Precision]
    def __init__(self, value: int) -> None: ...
    def __eq__(self, other: object) -> bool: ...
    def __hash__(self) -> int: ...
    def __index__(self) -> int: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: object) -> bool: ...
    @property
    def name(self) -> str: ...
    @property
    def value(self) -> int: ...

class RTMDetModel(DetectionModel):
    def __init__(self) -> None:
        """
        Create a new RTMDet model instance
        """

class RTMPoseModel(PoseModel):
    def __init__(self) -> None:
        """
        Create a new RTMPose model instance
        """

class ScalingMode:
    __members__: ClassVar[dict]
    AUTO: ClassVar[ScalingMode]
    DIRECT: ClassVar[ScalingMode]
    NORMALIZE_INPUT: ClassVar[ScalingMode]
    def __init__(self, value: int) -> None: ...
    def __eq__(self, other: object) -> bool: ...
    def __hash__(self) -> int: ...
    def __index__(self) -> int: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: object) -> bool: ...
    @property
    def name(self) -> str: ...
    @property
    def value(self) -> int: ...

class Tag:
    conf: float
    id: int
    @overload
    def __init__(self) -> None:
        """
        Create a new empty tag
        """
    @overload
    def __init__(self, arg0: int) -> None:
        """
        Create a new tag with a given tag id
        :param arg0: Tag ID
        """
    @overload
    def __init__(self, arg0: int, arg1: float) -> None:
        """
        Create a new tag with a given tag id and confidence score
        :param arg0: Tag ID
        :param arg1: Tag confidence score
        """
    @overload
    def __init__(self, arg0: Union[Point, Tuple[float, float]], arg1: Union[Point, Tuple[float, float]],
                 arg2: Union[Point, Tuple[float, float]], arg3: Union[Point, Tuple[float, float]]) -> None:
        """
        Create a new tag with a given set of corner points.

        Corner points are ordered clockwise starting from the top left corner.
        :param arg0: First corner point
        :param arg1: Second corner point
        :param arg2: Third corner point
        :param arg3: Fourth corner point
        """
    @overload
    def __init__(self, arg0: int, arg1: Union[Point, Tuple[float, float]], arg2: Union[Point, Tuple[float, float]],
                 arg3: Union[Point, Tuple[float, float]], arg4: Union[Point, Tuple[float, float]]) -> None:
        """
        Create a new tag with a given tag id and set of corner points.

        Corner points are ordered clockwise starting from the top left corner.
        :param arg0: Tag ID
        :param arg1: First corner point
        :param arg2: Second corner point
        :param arg3: Third corner point
        :param arg4: Fourth corner point
        """
    @overload
    def __init__(self, arg0: float, arg1: Union[Point, Tuple[float, float]], arg2: Union[Point, Tuple[float, float]],
                 arg3: Union[Point, Tuple[float, float]], arg4: Union[Point, Tuple[float, float]]) -> None:
        """
        Create a new tag with a given confidence score and set of corner points.

        Corner points are ordered clockwise starting from the top left corner.
        :param arg0: Tag confidence score
        :param arg1: First corner point
        :param arg2: Second corner point
        :param arg3: Third corner point
        :param arg4: Fourth corner point
        """
    @overload
    def __init__(self, arg0: int, arg1: float, arg2: Union[Point, Tuple[float, float]],
                 arg3: Union[Point, Tuple[float, float]], arg4: Union[Point, Tuple[float, float]],
                 arg5: Union[Point, Tuple[float, float]]) -> None:
        """
        Create a new tag with a given tag id, confidence score, and set of corner points.
        
        Corner points are ordered clockwise starting from the top left corner.
        :param arg0: Tag ID
        :param arg1: Tag confidence score
        :param arg2: First corner point
        :param arg3: Second corner point
        :param arg4: Third corner point
        :param arg5: Fourth corner point
        """
    def __getitem__(self, arg0: int) -> Point: ...
    def __iter__(self) -> typing.Iterator[Point]: ...
    def __setitem__(self, arg0: int, arg1: Union[Point, Tuple[float, float]]) -> None: ...

class TagModel(ImageModel):
    def __init__(self) -> None:
        """
        Create a new tag model instance. This is a base class and has no inference capabilities.
        """
    @overload
    def infer(self, arg0: Mat) -> List[Tag]:
        """
        Run a forward pass on a single image
        :param arg0: Image to send through the model
        """
    @overload
    def infer(self, arg0: List[Mat]) -> List[List[Tag]]:
        """
        Run a forward pass on a batch of images
        :param arg0: Images to send through the model
        """

class TagNetModel(TagModel):
    def __init__(self) -> None:
        """
        Create a new TagNet model instance
        """

class TopDownPoseDetector:
    detection_model: DetectionModel
    pose_model: PoseModel
    def __init__(self) -> None:
        """
        Create a new TopDownPoseDetector instance.

        This is a special inference module meant for pose estimation models
        that are designed for use in a top-down detection pipeline.
        """
    @overload
    def infer(self, image: Mat, max_pose_batches: int = 1, conf_thresh: float = 0.5,
              iou_thresh: float = 0.5) -> List[Pose]:
        """
        Detect poses on a single image
        :param image: Image to detect poses on
        :param max_pose_batches: Sample batch size for pose model
        :param conf_thresh: Detection model confidence threshold
        :param iou_thresh: Detection model IoU threshold
        """
    @overload
    def infer(self, images: List[Mat], max_detection_batches: int = 1, max_pose_batches: int = 1,
              conf_thresh: float = 0.5, iou_thresh: float = 0.5) -> List[List[Pose]]:
        """
        Detect poses on a set of images
        :param images: Images to detect poses on
        :param max_detection_batches: Batch size for detection model
        :param max_pose_batches: Sample batch size for pose model
        :param conf_thresh: Detection model confidence threshold
        :param iou_thresh: Detection model IoU threshold
        """
    def is_ready(self) -> bool:
        """
        Check if all models are loaded and detector is ready for inference
        """
    def unload_all(self) -> None:
        """
        Unload all models managed by this detector module
        """

class YOLOXModel(DetectionModel):
    def __init__(self) -> None:
        """
        Create a new YOLOX model instance
        """

def LetterboxImage(src: Mat, dst: Mat, out_size: tuple) -> List[float]:
    """
    Resize an image with letterboxing
    :param src: Image to resize
    :param dst: Image to write results to
    :param out_size: New image dimensions
    """
def drawTags(arg0: Mat, arg1: List[Tag]) -> Mat:
    """
    Draw tags on a provided image
    :param arg0: Image to draw tags onto
    :param arg1: Tags to draw
    """
@overload
def fixDetectionCoordinates(detections: List[Detection], src_net_size: Tuple[int, int],
                            target_frame_size: Tuple[int, int],
                            scaling_mode: ScalingMode = NORMALIZE_INPUT) -> List[Detection]:
    """
    Fix the bounding box coordinates for a list of detections.
    These detections will often be scaled and/or normalized to the
    input dimensions of the detection model they came from, thus
    needing adjustments to reflect proper pixel coordinate positions
    in the source image.

     If the scaling mode is NORMALIZE_INPUT, the coordinates will be
     normalized using src_net_size before being scaled to the target
     frame dimensions,

     If the scaling mode is DIRECT, the coordinates will be scaled
     directly to the target frame dimensions. This assumes that the
     output coordinates were already normalized.

     If the scaling mode is AUTO, the appropriate scaling mode will
     be determined automatically.
    :param detections: Detections to adjust
    :param src_net_size: Source detection net size
    :param target_frame_size: Target image frame size
    :param scaling_mode: Scaling mode to use
    """
@overload
def fixDetectionCoordinates(detections: List[List[Detection]], src_net_size: Tuple[int, int],
                            target_frame_size: Tuple[int, int],
                            scaling_mode: ScalingMode = NORMALIZE_INPUT) -> List[List[Detection]]:
    """
    Fix the bounding box coordinates for a list of detections.
    These detections will often be scaled and/or normalized to the
    input dimensions of the detection model they came from, thus
    needing adjustments to reflect proper pixel coordinate positions
    in the source image.

     If the scaling mode is NORMALIZE_INPUT, the coordinates will be
     normalized using src_net_size before being scaled to the target
     frame dimensions,

     If the scaling mode is DIRECT, the coordinates will be scaled
     directly to the target frame dimensions. This assumes that the
     output coordinates were already normalized.

     If the scaling mode is AUTO, the appropriate scaling mode will
     be determined automatically.
    :param detections: Detections to adjust
    :param src_net_size: Source detection net size
    :param target_frame_size: Target image frame size
    :param scaling_mode: Scaling mode to use
    """
def getRoiNoPadding(arg0: Mat, arg1: Rect) -> Mat:
    """
    Get ROI image sample without padding for overhang
    :param arg0: Source image
    :param arg1: ROI region
    """
def getRoiWithPadding(arg0: Mat, arg1: Rect) -> Mat:
    """
    Get ROI image sample, padding for overhang if the region extends out of bounds
    :param arg0: Source image
    :param arg1: ROI region
    """
def isRoiOutsideImage(arg0: Tuple[int, int], arg1: Rect) -> bool:
    """
    Check if an ROI is entirely outside the bounds of an image
    :param arg0: Image size
    :param arg1: ROI region
    """
@overload
def strict_batch_infer(arg0: int, arg1: DetectionModel, arg2: List[Mat], arg3: float, arg4: float) -> List[List[Detection]]:
    """
    Perform strict batch inference on a provided model.
    The input will be broken up into chunks that match
    the desired batch size, padding with extra images
    if necessary.
    :param arg0: Desired batch size
    :param arg1: Detection model
    :param arg2: Input images
    :param arg3: Detection confidence threshold
    :param arg4: Detection IoU threshold
    """
@overload
def strict_batch_infer(arg0: int, arg1: PoseModel, arg2: List[Mat]) -> List[Pose]:
    """
    Perform strict batch inference on a provided model.
    The input will be broken up into chunks that match
    the desired batch size, padding with extra images
    if necessary.
    :param arg0: Desired batch size
    :param arg1: Pose model
    :param arg2: Input images
    """
@overload
def strict_batch_infer(arg0: int, arg1: FeatureModel, arg2: List[Mat]) -> List[Feature]:
    """
    Perform strict batch inference on a provided model.
    The input will be broken up into chunks that match
    the desired batch size, padding with extra images
    if necessary.
    :param arg0: Desired batch size
    :param arg1: Feature model
    :param arg2: Input images
    """
@overload
def strict_batch_infer(arg0: int, arg1: TagModel, arg2: List[Mat]) -> List[List[Tag]]:
    """
    Perform strict batch inference on a provided model.
    The input will be broken up into chunks that match
    the desired batch size, padding with extra images
    if necessary.
    :param arg0: Desired batch size
    :param arg1: Tag model
    :param arg2: Input images
    """
