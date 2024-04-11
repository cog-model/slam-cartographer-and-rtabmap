import numpy as np
import cv2
import matplotlib.pyplot as plt
from matplotlib.colors import hsv_to_rgb
from .utils import show, select_roi


def get_mask_in_roi(mask, x_range, y_range):
    mask_in_roi = np.zeros_like(mask)
    mask_in_roi[y_range, x_range] = mask[y_range, x_range]
    return mask_in_roi


def refine_mask_by_polygons(mask,
        min_polygon_length=0, max_polygon_length=-1,
        min_polygon_area=0, max_polygon_area=-1,
        min_polygon_area_length_ratio=0,
        select_top_n_polygons_by_length=-1,
        select_top_n_polygons_by_area=-1):
    assert select_top_n_polygons_by_length < 0 or select_top_n_polygons_by_area < 0, \
        "refine_mask_by_polygons: " \
        "'select_top_n_polygons_by_length' and 'select_top_n_polygons_by_area' " \
        "can't both be >= 0"

    polygons, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    lengths = (None,) * len(polygons)
    areas = (None,) * len(polygons)

    laps = list(zip(lengths, areas, polygons))

    if min_polygon_length > 0 or max_polygon_length >= 0:
        if any(l is None for l, _, _ in laps):
            _, areas, polygons = zip(*laps)
            lengths = [len(p) for _, _, p in laps]
            laps = zip(lengths, areas, polygons)
        laps = [(l, a, p) for l, a, p in laps if
            l >= min_polygon_length and
            (max_polygon_length < 0 or l <= max_polygon_length)]

    if min_polygon_area > 0 or max_polygon_area >= 0:
        if any(a is None for _, a, _ in laps):
            lengths, _, polygons = zip(*laps)
            areas = [cv2.contourArea(p) for _, _, p in laps]
            laps = zip(lengths, areas, polygons)
        laps = [(l, a, p) for l, a, p in laps if
            a >= min_polygon_area and
            (max_polygon_area < 0 or a <= max_polygon_area)]

    if min_polygon_area_length_ratio > 0:
        if any(l is None for l, _, _ in laps):
            _, areas, polygons = zip(*laps)
            lengths = [len(p) for _, _, p in laps]
            laps = zip(lengths, areas, polygons)
        if any(a is None for _, a, _ in laps):
            lengths, _, polygons = zip(*laps)
            areas = [cv2.contourArea(p) for _, _, p in laps]
            laps = zip(lengths, areas, polygons)
        laps = [(l, a, p) for l, a, p in laps if
            a / l >= min_polygon_area_length_ratio]

    if select_top_n_polygons_by_length >= 0 and \
            len(laps) > select_top_n_polygons_by_length:
        if any(l is None for l, _, _ in laps):
            _, areas, polygons = zip(*laps)
            lengths = [len(p) for _, _, p in laps]
            laps = zip(lengths, areas, polygons)
        laps = sorted(laps, key=lambda lap: -lap[0])
        laps = laps[:select_top_n_polygons_by_length]

    if select_top_n_polygons_by_area >= 0 and \
            len(laps) > select_top_n_polygons_by_area:
        if any(a is None for _, a, _ in laps):
            lengths, _, polygons = zip(*laps)
            areas = [cv2.contourArea(p) for _, _, p in laps]
            laps = zip(lengths, areas, polygons)
        laps = sorted(laps, key=lambda lap: -lap[1])
        laps = laps[:select_top_n_polygons_by_area]

    if len(laps) > 0:
        lengths, areas, polygons = zip(*laps)
    else:
        lengths, areas, polygons = tuple(), tuple(), tuple()

    refined_mask = np.zeros_like(mask)
    for p in polygons:
        cv2.fillPoly(refined_mask, [p], 255)

    return refined_mask, polygons, lengths, areas


#############


def get_and_apply_mask(image, select_image_roi=True,
        min_h=0, max_h=255, min_s=0, max_s=255, min_v=0, max_v=255,
        min_sv=0, max_sv=255,
        shift_h=0, inverse_mask=False, show_image=True):
    if select_image_roi:
        x_range, y_range = select_roi(image, full_by_default=True)
    else:
        x_range, y_range = slice(0, None), slice(0, None)
    image = image[y_range, x_range].copy()

    if min_sv != 0 or max_sv != 255:
        use_sv_limits = True
    else:
        use_sv_limits = False
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV_FULL)
    hsv[:, :, 0] += shift_h
    if use_sv_limits:
        h = hsv[:, :, 0]
        sv = get_sv(hsv)
        h_sv = np.dstack((h, sv))
        mask = cv2.inRange(h_sv, (min_h, min_sv), (max_h, max_sv))
    else:
        mask = cv2.inRange(hsv, (min_h, min_s, min_v), (max_h, max_s, max_v))

    if not inverse_mask:
        background = (mask == 0)
    else:
        background = (mask != 0)
    image[background] = 0
    if show_image:
        show(image)
    return image, mask


def plot_s_histogram(image, select_image_roi=True, show=True):
    if select_image_roi:
        x_range, y_range = select_roi(image, full_by_default=True)
    else:
        x_range, y_range = slice(0, None), slice(0, None)
    image = image[y_range, x_range]
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV_FULL)
    s = hsv[:, :, 1]
    s = s.flatten()
    plt.hist(s, bins=256, range=(0, 255))
    if show:
        plt.show()


def plot_v_histogram(image, select_image_roi=True, show=True):
    if select_image_roi:
        x_range, y_range = select_roi(image, full_by_default=True)
    else:
        x_range, y_range = slice(0, None), slice(0, None)
    image = image[y_range, x_range]
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV_FULL)
    v = hsv[:, :, 2]
    v = v.flatten()
    plt.hist(v, bins=256, range=(0, 255))
    if show:
        plt.show()


def plot_sv_histogram(image, select_image_roi=True, show=True):
    if select_image_roi:
        x_range, y_range = select_roi(image, full_by_default=True)
    else:
        x_range, y_range = slice(0, None), slice(0, None)
    image = image[y_range, x_range]
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV_FULL)
    sv = get_sv(hsv)
    sv = sv.flatten()
    plt.hist(sv, bins=256, range=(0, 255))
    if show:
        plt.show()


def plot_h_histogram(image, select_image_roi=True,
        min_s=0, max_s=255, min_v=0, max_v=255,
        min_sv=0, max_sv=255, shift_h=0, show=True):
    if select_image_roi:
        x_range, y_range = select_roi(image, full_by_default=True)
    else:
        x_range, y_range = slice(0, None), slice(0, None)
    image = image[y_range, x_range]

    if min_sv != 0 or max_sv != 255:
        use_sv_limits = True
    else:
        use_sv_limits = False
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV_FULL)
    if use_sv_limits:
        sv = get_sv(hsv)
        mask = cv2.inRange(sv, min_sv, max_sv)
    else:
        s = hsv[:, :, 1]
        v = hsv[:, :, 2]
        s_v = np.dstack((s, v))
        mask = cv2.inRange(s_v, (min_s, min_v), (max_s, max_v))

    h = hsv[:, :, 0]
    h = h[mask != 0]
    h += shift_h

    _, _, patches = plt.hist(h, bins=256, range=(0, 255))
    assert len(patches) == 256
    for i, patch in enumerate(patches):
        vis_h = (i - shift_h) % 256 / 255
        r, g, b = hsv_to_rgb([vis_h, 1.0, 1.0])
        patch.set_facecolor([r, g, b])

    if show:
        plt.show()


def show_s(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV_FULL)
    s = hsv[:, :, 1]
    show(s)


def show_v(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV_FULL)
    v = hsv[:, :, 2]
    show(v)


def show_sv(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV_FULL)
    sv = get_sv(hsv)
    show(sv)


def show_h(image, shift_h=0):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV_FULL)
    h = hsv[:, :, 0]
    h += shift_h
    show(h)


def get_sv(hsv):
    s = hsv[:, :, 1]
    v = hsv[:, :, 2]
    sv = s.astype(np.float32) * v.astype(np.float32) / 255
    sv = sv.astype(np.uint8)
    return sv


def plot_sv_points(image, select_image_roi=True,
        min_h=0, max_h=255, shift_h=0, show=True):
    if select_image_roi:
        x_range, y_range = select_roi(image, full_by_default=True)
    else:
        x_range, y_range = slice(0, None), slice(0, None)
    image = image[y_range, x_range]
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV_FULL)
    h = hsv[:, :, 0]
    h += shift_h
    mask = cv2.inRange(h, min_h, max_h)
    hsv = hsv[mask != 0]
    s = hsv[:, 1]
    v = hsv[:, 2]
    plt.xlim([0, 255])
    plt.xlabel('s')
    plt.ylim([0, 255])
    plt.ylabel('v')
    plt.plot(s, v, 'o', markersize=1)
    if show:
        plt.show()

def show_hsv_color(h, s, v):
    hsv = np.zeros((200, 200, 3), dtype=np.uint8)
    hsv[:, :, 0] = h
    hsv[:, :, 1] = s
    hsv[:, :, 2] = v
    bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR_FULL)
    show(bgr)
