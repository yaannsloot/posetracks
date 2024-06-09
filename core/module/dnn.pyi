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
    def set_dict_type(self, dict: TagDictionary) -> None:
        """
        Set the dictionary type used by the detector
        :param dict: Predefined tag dictionary
        """
    def set_preprocess_size(self, size: Tuple[int, int]) -> None:
        """
        If set, resizes the sampled region before detection
        :param size: Preprocess size of forwarded samples
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
    def scale_detection(self, scale: float) -> None:
        """
        Scale bounds of detection
        :param scale: Scale factor
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
    def __init__(self, data: List[float]) -> None: ...
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
    def __assign__(self, other: Feature) -> Feature: ...
    def __getitem__(self, index: int) -> float: ...
    def __iter__(self) -> typing.Iterator[float]: ...
    def __len__(self) -> int: ...
    def __setitem__(self, index: int, value: float) -> None: ...
    def __sub__(self, other: Feature) -> Feature: ...
    def __truediv__(self, other: float) -> Feature: ...

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
    def infer(self, image: Mat) -> Feature:
        """
        Run a forward pass on a single image
        :param image: Image to send through the model
        """
    @overload
    def infer(self, images: List[Mat]) -> List[Feature]:
        """
        Run a forward pass on a batch of images
        :param images: Images to send through the model
        """

class GenericFeatureModel(FeatureModel):
    def __init__(self) -> None:
        """
        Create a new generic feature model instance
        """

class FeatureSet:
    """
    Specialized container for fast mean operations on a managed set of feature vectors.

    Tracks the mean by updating it on every insertion. Each inserted feature updates the mean through weighted addition.
    If a feature is removed, the mean is recalculated using the internal list of features.
    This means it is much faster to insert a new feature than it is to remove one.
    """
    def __init__(self, feature_length: int) -> None:
        """
        Specialized container for fast mean operations on a managed set of feature vectors.

        Tracks the mean by updating it on every insertion. Each inserted feature updates the mean through weighted addition.
        If a feature is removed, the mean is recalculated using the internal list of features.
        This means it is much faster to insert a new feature than it is to remove one.
        :param feature_length: Length of individual features within this feature set
        """
    def add(self, f: Feature) -> None:
        """
        Insert a new feature into this feature set, updating the mean
        :param f: Feature to add. Must match the feature length of this feature set
        """
    def at(self, index: int) -> Feature:
        """
        Retrieve a copy of the feature at the specified index
        :param index: Index to reference
        :return: A copy of the indexed feature
        """
    def remove(self, index: int) -> None:
        """
        Remove the feature at the specified index. This set's mean feature will be recalculated.
        :param index: Index to remove
        """
    def mean(self) -> Feature:
        """
        Returns a copy of this set's mean feature
        """
    def size(self) -> int:
        """
        Returns the number of features in this set
        """
    def length(self) -> int:
        """
        Returns the expected feature length of this feature set
        """
    def __getitem__(self, item: int) -> Feature:
        """
        Retrieve a copy of the feature at the specified index
        :param item: Index to reference
        :return: A copy of the indexed feature
        """

class FeatureSpace:
    """
    Specialized container that assigns new features to feature sets within the feature space
    """
    def __init__(self, feature_length: int) -> None:
        """
        Specialized container that assigns new features to feature sets within the feature space
        :param feature_length: Length of individual features within this feature space
        """
    @overload
    def assign(self, input: Feature, threshold: float = 0.4, dist_type: FeatureDistanceType = NORM_EUCLIDEAN) -> int:
        """
        Performs a single feature assignment into this feature space.

        This is not a temporally stable operation. If two features from
        the same time frame are added using this function and are similar
        enough, they will be assigned the same id.
        :param input: Feature to be added to this feature space
        :param threshold: Maximum distance to potential feature sets within this space
        :param dist_type: Distance operation to use
        :return: An Index pointing to the feature set the input was assigned to
        """
    @overload
    def assign(self, input: List[Feature], threshold: float = 0.4, dist_type: FeatureDistanceType = NORM_EUCLIDEAN,
               mask: List[int] = None) -> List[int]:
        """
        Performs a one-to-one assignment of a vector of features to this feature space.

        All provided features are scored relative to existing feature sets and assigned
        in ascending order based on distance, continuing until all selected features are
        exhausted or existing feature sets are exhausted, whichever comes first.
        If there are leftover input features, they will be assigned to new feature sets.
        :param input: List of features to be added to this feature space
        :param threshold: Maximum distance to potential feature sets within this space
        :param dist_type: Distance operation to use
        :param mask: Optional mask list for excluding existing sets from assignment
        :return: A list of indexes that reference the set each input feature was assigned to
        """
    def size(self) -> int:
        """
        Returns the number of tracked feature sets in this feature space
        """
    def length(self) -> int:
        """
        Returns the expected feature length of this feature space
        """
    def clear(self) -> None:
        """
        Removes all feature sets from this feature space
        """
    def at(self, index: int) -> FeatureSet:
        """
        Returns a reference to the feature set at the specified position within this feature space
        :param index: Index of the desired feature set
        """
    def __getitem__(self, item: int) -> FeatureSet:
        """
        Returns a reference to the feature set at the specified position within this feature space
        :param item: Index of the desired feature set
        """

class FeatureTracker:
    """
    Tracks object detections over time using an internal feature space and predictive bounding box filter
    """
    def __init__(self, feature_length: int) -> None:
        """
        Tracks object detections over time using an internal feature space and predictive bounding box filter
        :param feature_length: Length of individual features within this tracker's feature space
        """
    def assign(self, input_boxes: List[Detection], input_features: List[Feature], score_threshold: float = 0.7,
               f_space_threshold: float = 0.4, dist_type: FeatureDistanceType = NORM_EUCLIDEAN,
               mask: List[int] = None) -> List[int]:
        """

        :param input_boxes: Bounding box detections from last frame
        :param input_features: Extracted features from bounding box regions
        :param score_threshold: Maximum feature distance of boxes that overlap predicted box states
        :param f_space_threshold: Maximum feature space distance of boxes that failed to meet overlap criteria
        :param dist_type: Distance operation to use
        :param mask: Optional mask list for excluding existing sets from assignment
        :return: A list of indexes that reference the set each input feature was assigned to
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
    def __init__(self, x: float, y: float, conf: float) -> None:
        """
        Create a new joint
        :param x: X coordinate of joint
        :param y: Y coordinate of joint
        :param conf: Confidence score of joint
        """
    @overload
    def __init__(self, pt: Point, conf: float) -> None:
        """
        Create a new joint
        :param pt: Joint position
        :param conf: Confidence score of joint
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
    def get_joint(self, joint_id: int) -> Joint:
        """
        Retrieve a joint by id. If one does not exist, a new joint will be added
        :param joint_id: Joint ID
        """
    def get_joint_ids(self) -> Set[int]:
        """
        Get a list of all current joint ids within this pose
        """
    def has_joint(self, joint_id: int) -> bool:
        """
        Check if this pose contains a joint with the provided id
        :param joint_id: Joint ID
        """
    def num_joints(self) -> int:
        """
        Get the total number of joints contained inside this pose
        """
    @overload
    def set_joint(self, joint_id: int, joint: Joint) -> None:
        """
        Set joint data for a given joint id
        :param joint_id: Joint ID
        :param joint: Joint to assign to ID
        """
    @overload
    def set_joint(self, joint_id: int, pt: Point, conf: float) -> None:
        """
        Set joint data for a given joint id
        :param joint_id: Joint ID
        :param pt: Joint position
        :param conf: Joint confidence score
        """
    @overload
    def set_joint(self, joint_id: int, x: float, y: float, conf: float) -> None:
        """
        Set joint data for a given joint id
        :param joint_id: Joint ID
        :param x: Joint X coordinate
        :param y: Joint Y coordinate
        :param conf: Joint confidence score
        """
    def __getitem__(self, joint_id: int) -> Joint: ...
    def __iter__(self) -> typing.Iterator[Joint]: ...

class PoseModel(ImageModel):
    def __init__(self) -> None:
        """
        Create a new pose model instance. This is a base class and has no inference capabilities.
        """
    @overload
    def infer(self, image: Mat) -> Pose:
        """
        Run a forward pass on a single image
        :param image: Image to send through the model
        """
    @overload
    def infer(self, images: List[Mat]) -> List[Pose]:
        """
        Run a forward pass on a batch of images
        :param images: Images to send through the model
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
    def __init__(self, id: int) -> None:
        """
        Create a new tag with a given tag id
        :param id: Tag ID
        """
    @overload
    def __init__(self, id: int, conf: float) -> None:
        """
        Create a new tag with a given tag id and confidence score
        :param id: Tag ID
        :param conf: Tag confidence score
        """
    @overload
    def __init__(self, ca: Union[Point, Tuple[float, float]], cb: Union[Point, Tuple[float, float]],
                 cc: Union[Point, Tuple[float, float]], cd: Union[Point, Tuple[float, float]]) -> None:
        """
        Create a new tag with a given set of corner points.

        Corner points are ordered clockwise starting from the top left corner.
        :param ca: First corner point
        :param cb: Second corner point
        :param cc: Third corner point
        :param cd: Fourth corner point
        """
    @overload
    def __init__(self, id: int, ca: Union[Point, Tuple[float, float]], cb: Union[Point, Tuple[float, float]],
                 cc: Union[Point, Tuple[float, float]], cd: Union[Point, Tuple[float, float]]) -> None:
        """
        Create a new tag with a given tag id and set of corner points.

        Corner points are ordered clockwise starting from the top left corner.
        :param id: Tag ID
        :param ca: First corner point
        :param cb: Second corner point
        :param cc: Third corner point
        :param cd: Fourth corner point
        """
    @overload
    def __init__(self, conf: float, ca: Union[Point, Tuple[float, float]], cb: Union[Point, Tuple[float, float]],
                 cc: Union[Point, Tuple[float, float]], cd: Union[Point, Tuple[float, float]]) -> None:
        """
        Create a new tag with a given confidence score and set of corner points.

        Corner points are ordered clockwise starting from the top left corner.
        :param conf: Tag confidence score
        :param ca: First corner point
        :param cb: Second corner point
        :param cc: Third corner point
        :param cd: Fourth corner point
        """
    @overload
    def __init__(self, id: int, conf: float, ca: Union[Point, Tuple[float, float]],
                 cb: Union[Point, Tuple[float, float]], cc: Union[Point, Tuple[float, float]],
                 cd: Union[Point, Tuple[float, float]]) -> None:
        """
        Create a new tag with a given tag id, confidence score, and set of corner points.
        
        Corner points are ordered clockwise starting from the top left corner.
        :param id: Tag ID
        :param conf: Tag confidence score
        :param ca: First corner point
        :param cb: Second corner point
        :param cc: Third corner point
        :param cd: Fourth corner point
        """
    def __getitem__(self, corner_index: int) -> Point: ...
    def __iter__(self) -> typing.Iterator[Point]: ...
    def __setitem__(self, corner_index: int, pt: Union[Point, Tuple[float, float]]) -> None: ...

class TagModel(ImageModel):
    def __init__(self) -> None:
        """
        Create a new tag model instance. This is a base class and has no inference capabilities.
        """
    @overload
    def infer(self, image: Mat) -> List[Tag]:
        """
        Run a forward pass on a single image
        :param image: Image to send through the model
        """
    @overload
    def infer(self, images: List[Mat]) -> List[List[Tag]]:
        """
        Run a forward pass on a batch of images
        :param images: Images to send through the model
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

def letterbox_image(src: Mat, dst: Mat, out_size: tuple) -> List[float]:
    """
    Resize an image with letterboxing
    :param src: Image to resize
    :param dst: Image to write results to
    :param out_size: New image dimensions
    """
def draw_tags(image: Mat, tags: List[Tag]) -> Mat:
    """
    Draw tags on a provided image
    :param image: Image to draw tags onto
    :param tags: Tags to draw
    """
@overload
def fix_detection_coordinates(detections: List[Detection], src_net_size: Tuple[int, int],
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
def fix_detection_coordinates(detections: List[List[Detection]], src_net_size: Tuple[int, int],
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
def get_roi_no_padding(image: Mat, roi: Rect) -> Mat:
    """
    Get ROI image sample without padding for overhang
    :param image: Source image
    :param roi: ROI region
    """
def get_roi_with_padding(image: Mat, roi: Rect) -> Mat:
    """
    Get ROI image sample, padding for overhang if the region extends out of bounds
    :param image: Source image
    :param roi: ROI region
    """
def is_roi_outside_image(img_size: Tuple[int, int], roi: Rect) -> bool:
    """
    Check if an ROI is entirely outside the bounds of an image
    :param img_size: Image size
    :param roi: ROI region
    """
@overload
def strict_batch_infer(batch_size: int, model: DetectionModel, images: List[Mat], conf_thresh: float,
                       iou_thresh: float) -> List[List[Detection]]:
    """
    Perform strict batch inference on a provided model.
    The input will be broken up into chunks that match
    the desired batch size, padding with extra images
    if necessary.
    :param batch_size: Desired batch size
    :param model: Detection model
    :param images: Input images
    :param conf_thresh: Detection confidence threshold
    :param iou_thresh: Detection IoU threshold
    """
@overload
def strict_batch_infer(batch_size: int, model: PoseModel, images: List[Mat]) -> List[Pose]:
    """
    Perform strict batch inference on a provided model.
    The input will be broken up into chunks that match
    the desired batch size, padding with extra images
    if necessary.
    :param batch_size: Desired batch size
    :param model: Pose model
    :param images: Input images
    """
@overload
def strict_batch_infer(batch_size: int, model: FeatureModel, images: List[Mat]) -> List[Feature]:
    """
    Perform strict batch inference on a provided model.
    The input will be broken up into chunks that match
    the desired batch size, padding with extra images
    if necessary.
    :param batch_size: Desired batch size
    :param model: Feature model
    :param images: Input images
    """
@overload
def strict_batch_infer(batch_size: int, model: TagModel, images: List[Mat]) -> List[List[Tag]]:
    """
    Perform strict batch inference on a provided model.
    The input will be broken up into chunks that match
    the desired batch size, padding with extra images
    if necessary.
    :param batch_size: Desired batch size
    :param model: Tag model
    :param images: Input images
    """
