from . import register
from .datatypes import Tensor, BoundingBox
import numpy as np


@register
def accumulate_yolox(model_out: dict[str, Tensor], /,
                     context: dict[str, any],
                     threshold=0.25, **kwargs) -> list[list[BoundingBox]]:
    img_dims = context["input_sizes"]
    boxes_raw = model_out["output"].to_numpy().data
    grid_sizes = [80, 40, 20]
    sizes = [g**2 for g in grid_sizes]
    starts = np.cumsum([0] + sizes[:-1])
    ends = np.cumsum(sizes)

    for g, start, end in zip(grid_sizes, starts, ends):
        cell = 1 / g
        offsets_x = np.tile(np.arange(g) / g, g)
        offsets_y = np.repeat(np.arange(g) / g, g)

        boxes_raw[:, start:end, 0] = boxes_raw[:,
                                               start:end, 0] * cell + offsets_x
        boxes_raw[:, start:end, 1] = boxes_raw[:,
                                               start:end, 1] * cell + offsets_y
        boxes_raw[:, start:end, 2] = np.exp(boxes_raw[:, start:end, 2]) * cell
        boxes_raw[:, start:end, 3] = np.exp(boxes_raw[:, start:end, 3]) * cell

    class_scores = boxes_raw[:, :, 5:]
    class_ids = np.argmax(class_scores, axis=-1)
    class_conf = np.max(class_scores, axis=-1)

    boxes_raw[:, :, 0] -= boxes_raw[:, :, 2] / 2
    boxes_raw[:, :, 1] -= boxes_raw[:, :, 3] / 2

    decoded = np.concatenate([
        boxes_raw[:, :, 0:5],
        class_ids[:, :, None].astype(np.float32),
        class_conf[:, :, None]
    ], axis=-1)

    final = []

    for i in range(decoded.shape[0]):
        arr = decoded[i]
        dims = img_dims[i]
        scores = arr[:, 4] * arr[:, 6]
        mask = scores > threshold
        arr = arr[mask]
        arr[:, (0, 2)] *= dims[0]
        arr[:, (1, 3)] *= dims[1]
        final.append(arr)

    return final


@register
def accumulate_rfdetr(model_out: dict[str, Tensor], /,
                      context: dict[str, any],
                      threshold=0.25,
                      **kwargs) -> list[list[BoundingBox]]:
    img_dims = context["input_sizes"]
    boxes_raw = model_out["dets"].to_numpy().data
    labels_raw = model_out["labels"].to_numpy().data

    ids = np.argmax(labels_raw, axis=-1, keepdims=True)
    scores = np.take_along_axis(labels_raw, ids, axis=-1)
    scores = 1 / (1 + np.exp(-scores))
    valid = (scores > threshold).squeeze(-1)

    boxes_all = np.concatenate([
        boxes_raw, 
        scores,
        ids, 
    ], axis=-1)

    boxes_all[:, :, 0] -= boxes_all[:, :, 2] / 2
    boxes_all[:, :, 1] -= boxes_all[:, :, 3] / 2

    final = []

    for i in range(boxes_raw.shape[0]):
        dims = img_dims[i]
        boxes = boxes_all[i][valid[i]]
        boxes[:, (0, 2)] *= dims[0]
        boxes[:, (1, 3)] *= dims[1]
        boxes = [BoundingBox(*box) for box in boxes]
        final.append(boxes)

    return final