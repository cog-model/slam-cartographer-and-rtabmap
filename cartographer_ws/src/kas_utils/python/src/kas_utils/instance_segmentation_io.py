import numpy as np
import cv2


def from_instance_segmentation(instance):
    assert instance.dtype == np.uint16

    objects = np.unique(instance)
    objects = objects[objects != 0]
    classes_ids = list()
    masks = list()
    for obj in objects:
        class_id = obj >> 8
        mask = (instance == obj).astype(np.uint8)

        classes_ids.append(class_id)
        masks.append(mask)

    classes_ids = np.array(classes_ids)
    masks = np.array(masks)
    return classes_ids, masks


def to_instance_segmentation(classes_ids, masks):
    assert len(classes_ids) < 2 ** 8, \
        "get_instance_segmentation: Can't save so many objects."

    out_instance = np.zeros((masks.shape[1:]), dtype=np.uint16)
    counter = 1
    for class_id, mask in zip(classes_ids, masks):
        assert class_id < 2 ** 8
        obj = (class_id << 8) + counter
        out_instance[mask != 0] = obj
        counter += 1

    return out_instance


def read_instance_segmentation_file(instance_file):
    assert instance_file.endswith('.png')
    instance = cv2.imread(instance_file, cv2.IMREAD_UNCHANGED)
    classes_ids, masks = from_instance_segmentation(instance)
    return classes_ids, masks


def write_instance_segmentation_file(classes_ids, masks, out_instance_file):
    assert out_instance_file.endswith('.png')
    out_instance = to_instance_segmentation(classes_ids, masks)
    ret = cv2.imwrite(out_instance_file, out_instance)
    return ret
