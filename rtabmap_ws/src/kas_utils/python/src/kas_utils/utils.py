import numpy as np
import cv2


def is_float(string: str):
    try:
        float(string)
        return True
    except:
        return False


def show(image, window_name="image", destroy_window=True, wait_ms=0):
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.imshow(window_name, image)
    key = cv2.waitKey(wait_ms)
    if destroy_window:
        cv2.destroyWindow(window_name)
    return key


def select_roi(image, full_by_default=False, window_name="select roi"):
    roi = cv2.selectROI(window_name, image, showCrosshair=False)
    cv2.destroyAllWindows()

    x, y, w, h = roi
    if (x, y, w, h) == (0, 0, 0, 0):
        if full_by_default:
            x_range = slice(0, None)
            y_range = slice(0, None)
        else:
            x_range = None
            y_range = None
    else:
        x_range = slice(x, x + w)
        y_range = slice(y, y + h)
    return x_range, y_range


def get_depth_scale(depth):
    if isinstance(depth, np.ndarray):
        dtype = depth.dtype
    else:
        dtype = depth

    if dtype == float or dtype == np.float32:
        scale = 1
    elif dtype == np.uint16:
        scale = 0.001
    else:
        raise RuntimeError(f"Unknown depth type {dtype}")

    return scale
