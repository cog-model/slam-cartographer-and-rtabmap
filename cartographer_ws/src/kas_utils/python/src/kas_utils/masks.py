import numpy as np
from numbers import Number


def get_masks_rois(masks):
    if len(masks) == 0:
        return np.empty((0,), dtype=object)

    if isinstance(masks, np.ndarray) and masks.ndim == 2:
        masks = np.expand_dims(masks, axis=0)
        extended = True
    else:
        extended = False

    rows = np.any(masks, axis=2)
    cols = np.any(masks, axis=1)

    rois = list()
    for rs, cs in zip(rows, cols):
        y_min, y_max = np.where(rs)[0][[0, -1]]
        x_min, x_max = np.where(cs)[0][[0, -1]]
        roi = (slice(y_min, y_max + 1), slice(x_min, x_max + 1))
        rois.append(roi)

    if extended:
        rois = rois[0]
    else:
        rois = np.array(rois + [None], dtype=object)[:-1]
    return rois


def get_masks_in_rois(masks, rois, copy=True):
    if len(masks) == 0 and len(rois) == 0:
        return np.empty((0,), dtype=object)

    if isinstance(masks, np.ndarray) and masks.ndim == 2:
        masks = np.expand_dims(masks, axis=0)
        rois = [rois]
        extended = True
    else:
        extended = False

    if len(masks) != len(rois):
        raise RuntimeError("Number of masks and rois is not equal.")

    masks_in_rois = list()
    for mask, roi in zip(masks, rois):
        mask_in_roi = mask[roi]
        if copy:
            masks_in_rois.append(mask_in_roi.copy())
        else:
            masks_in_rois.append(mask_in_roi)

    if extended:
        masks_in_rois = masks_in_rois[0]
    else:
        masks_in_rois = np.array(masks_in_rois + [None], dtype=object)[:-1]
    return masks_in_rois


def get_full_masks(masks_in_rois, rois, widths, heights):
    if len(masks_in_rois) == 0 and len(rois) == 0 and \
            (isinstance(widths, Number) or len(widths) == 0) and \
            (isinstance(heights, Number) or len(heights) == 0):
        return np.empty((0,), dtype=object)

    if isinstance(masks_in_rois, np.ndarray) and masks_in_rois.ndim == 2:
        masks_in_rois = np.expand_dims(masks_in_rois, axis=0)
        rois = [rois]
        widths = [widths]
        heights = [heights]
        extended = True
    else:
        if isinstance(widths, Number):
            widths = [widths] * len(masks_in_rois)
        if isinstance(heights, Number):
            heights = [heights] * len(masks_in_rois)
        extended = False

    if any(l != len(masks_in_rois) for l in (len(rois), len(widths), len(heights))):
        raise RuntimeError("Number of items is not equal.")

    full_masks = list()
    for mask_in_roi, roi, width, height in zip(masks_in_rois, rois, widths, heights):
        full_mask = np.zeros((height, width), dtype=np.uint8)
        full_mask[roi] = mask_in_roi
        full_masks.append(full_mask)

    if extended:
        full_masks = full_masks[0]
    else:
        full_masks = np.array(full_masks + [None], dtype=object)[:-1]
    return full_masks
