import cv2
import numpy as np
from shutil import copy, move
import os
import os.path as osp
from .visualization import draw_objects


# Recommended folder structure:
# dataset/
#     all/
#         images/              - for annotated images
#         instances/           - for annotations
#         raw_images/          - for new and already annotated images
#         rejected_raw_images/ - for images that failed to be annotated
#     training/
#     validation/


def check_files(images_files, annotations_files):
    if len(images_files) != len(annotations_files):
        return False
    for image_file, annotation_file in zip(images_files, annotations_files):
        if osp.splitext(osp.basename(image_file))[0] != \
                osp.splitext(osp.basename(annotation_file))[0]:
            return False
    return True


class AnnotateImages:
    def __init__(self, model, model_args, model_kwargs, classes_names):
        self.model = model
        self.model_args = model_args
        self.model_kwargs = model_kwargs
        self.classes_names = classes_names

    def annotate_images(self, raw_images_folder,
            out_images_folder, out_annotations_folder):
        cv2.namedWindow("annotate_images_window", cv2.WINDOW_NORMAL)
        cv2.setWindowTitle("annotate_images_window", "")

        raw_images_files = sorted(os.listdir(raw_images_folder))
        existing_images_files = sorted(os.listdir(out_images_folder))
        raw_images_files = [raw_image_file for raw_image_file in raw_images_files
            if raw_image_file not in existing_images_files]
        raw_images_files = \
            list(map(lambda f: osp.join(raw_images_folder, f), raw_images_files))
        for raw_image_file in raw_images_files:
            masks, classes_ids = self.model(
                raw_image_file, *self.model_args, **self.model_kwargs)
            image = cv2.imread(raw_image_file)
            accepted, accepted_indices, accepted_classes_ids = self._validate_image(
                image, masks, classes_ids)
            if accepted:
                annotation = self._masks_to_annotation(
                    masks, accepted_indices, accepted_classes_ids)

                raw_image_file_basename = osp.basename(raw_image_file)
                out_image_file = osp.join(out_images_folder, raw_image_file_basename)
                out_annotation_file = osp.join(out_annotations_folder,
                    osp.splitext(raw_image_file_basename)[0] + '.png')
                copy(raw_image_file, out_image_file)
                cv2.imwrite(out_annotation_file, annotation)

                print(f"Saved {raw_image_file}")
            else:
                print(f"Rejected {raw_image_file}")
        cv2.destroyWindow("annotate_images_window")

    def _validate_image(self, image, segmentations, classes_ids):
        accepted_indices = list()
        accepted_classes_ids = list()
        for i, (mask, class_id) in enumerate(zip(segmentations, classes_ids)):
            masked_image = image.copy()
            masked_image[mask == 0] //= 6

            prompt_str = f"{self.classes_names[class_id]} [accept(a) / reject(r) / reject image(Esc)]"
            cv2.setWindowTitle("annotate_images_window", prompt_str)
            cv2.imshow("annotate_images_window", masked_image)
            key = -1
            while key not in (ord('a'), ord('r'), 27):  # 27 is 'Esc'
                key = cv2.waitKey(0)
            if key == ord('a'):
                accepted_indices.append(i)
                accepted_classes_ids.append(class_id)
            elif key == 27:
                return False, tuple(), tuple()

        prompt_str = f"Accepted {len(accepted_indices)} masks. Accept image? [y/n]"
        cv2.setWindowTitle("annotate_images_window", prompt_str)
        cv2.imshow("annotate_images_window", image)
        key = -1
        while key not in (ord('y'), ord('n')):
            key = cv2.waitKey(0)
        accepted = (key == ord('y'))
        return accepted, accepted_indices, accepted_classes_ids

    def _masks_to_annotation(self, masks, indices, classes_ids):
        annotation = np.zeros(masks[0].shape, dtype=np.uint16)
        for i, (index, class_id) in enumerate(zip(indices, classes_ids)):
            obj = (class_id << 8) + i + 1
            annotation[masks[index] != 0] = obj
        return annotation


def video_to_images(video_file, k, out_images_save_paths_generator):
    cv2.namedWindow("video_to_images_window", cv2.WINDOW_NORMAL)
    cv2.setWindowTitle("video_to_images_window", "")

    cap = cv2.VideoCapture(video_file)
    assert cap.isOpened()

    ret = True
    counter = 0
    while ret:
        ret, image = cap.read()
        if ret:
            if counter % k == 0:
                prompt_str = f"Save image? [y / n / exit(Esc)]"
                cv2.setWindowTitle("video_to_images_window", prompt_str)
                cv2.imshow("video_to_images_window", image)
                key = -1
                while key not in (ord('y'), ord('n'), 27):  # 27 is 'Esc'
                    key = cv2.waitKey(0)
                if key == ord('y'):
                    out_file = out_images_save_paths_generator()
                    cv2.imwrite(out_file, image)
                    print(f"Saved {out_file}")
                elif key == 27:
                    break
        counter += 1

    cv2.destroyWindow("video_to_images_window")


def split_dataset(images_folder, annotations_folder,
        out_images_folder_1, out_annotations_folder_1,
        out_images_folder_2, out_annotations_folder_2,
        split_rate):
    images_files = sorted(os.listdir(images_folder))
    annotations_files = sorted(os.listdir(annotations_folder))

    out_images_files_1 = sorted(os.listdir(out_images_folder_1))
    out_annotations_files_1 = sorted(os.listdir(out_annotations_folder_1))

    out_images_files_2 = sorted(os.listdir(out_images_folder_2))
    out_annotations_files_2 = sorted(os.listdir(out_annotations_folder_2))

    assert check_files(images_files, annotations_files)
    assert check_files(out_images_files_1, out_annotations_files_1)
    assert check_files(out_images_files_2, out_annotations_files_2)

    out_images_files = out_images_files_1 + out_images_files_2
    unused_images_annotations_files = \
        [(image_file, annotation_file) for image_file, annotation_file in
            zip(images_files, annotations_files) if image_file not in out_images_files]
    print(f"Found {len(unused_images_annotations_files)} new images")
    if len(unused_images_annotations_files) == 0:
        return
    unused_images_files, unused_annotations_files = zip(*unused_images_annotations_files)
    
    unused_images_files = \
        list(map(lambda f: osp.join(images_folder, f), unused_images_files))
    unused_annotations_files = \
        list(map(lambda f: osp.join(annotations_folder, f), unused_annotations_files))

    total_num = len(images_files)
    num_1 = int(total_num * split_rate)
    num_2 = total_num - num_1
    unused_num_1 = num_1 - len(out_images_files_1)
    unused_num_2 = num_2 - len(out_images_files_2)
    assert unused_num_1 >= 0
    assert unused_num_2 >= 0
    assert unused_num_1 + unused_num_2 == len(unused_images_files)

    unused_images_annotations_files = list(zip(unused_images_files, unused_annotations_files))
    np.random.shuffle(unused_images_annotations_files)
    unused_images_annotations_files_1 = unused_images_annotations_files[:unused_num_1]
    unused_images_annotations_files_2 = unused_images_annotations_files[unused_num_1:]

    for image_file, annotation_file in unused_images_annotations_files_1:
        copy(image_file, out_images_folder_1)
        copy(annotation_file, out_annotations_folder_1)

    for image_file, annotation_file in unused_images_annotations_files_2:
        copy(image_file, out_images_folder_2)
        copy(annotation_file, out_annotations_folder_2)


def visualize_annotations(images_folder, annotations_folder, palette):
    images_files = sorted(os.listdir(images_folder))
    annotations_files = sorted(os.listdir(annotations_folder))
    assert check_files(images_files, annotations_files)

    images_files = list(map(lambda f: osp.join(images_folder, f), images_files))
    annotations_files = list(map(lambda f: osp.join(annotations_folder, f), annotations_files))

    cv2.namedWindow("visualize_annotations_window", cv2.WINDOW_NORMAL)
    cv2.setWindowTitle("visualize_annotations_window", "")
    for image_file, annotation_file in zip(images_files, annotations_files):
        image = cv2.imread(image_file)
        annotation = cv2.imread(annotation_file, cv2.IMREAD_UNCHANGED)

        scores = list()
        classes_ids = list()
        boxes = list()
        masks = list()
        objects = np.unique(annotation)
        objects = objects[objects != 0]
        for obj in objects:
            class_id = (obj >> 8) & 0xFF
            mask = (annotation == obj)
            pixels = np.nonzero(mask)
            x1 = min(pixels[1])
            y1 = min(pixels[0])
            x2 = max(pixels[1])
            y2 = max(pixels[0])
            box = [x1, y1, x2, y2]
            scores.append(1)
            classes_ids.append(class_id)
            boxes.append(box)
            masks.append(mask)
        scores = np.array(scores)
        classes_ids = np.array(classes_ids)
        boxes = np.array(boxes)
        masks = np.array(masks, dtype=np.uint8)

        draw_objects(image, scores, classes_ids, boxes, masks, draw_masks=True)

        cv2.setWindowTitle("visualize_annotations_window", image_file)
        cv2.imshow("visualize_annotations_window", image)
        key = cv2.waitKey(0)
        if key == 27:  # 27 is Esc
            break

    cv2.destroyWindow("visualize_annotations_window")


def move_rejected_images_except_last(raw_images_folder, images_folder, out_images_folder):
    raw_images_files = sorted(os.listdir(raw_images_folder))
    images_files = sorted(os.listdir(images_folder))

    rejected_images_files = [raw_image_file for raw_image_file in raw_images_files
        if raw_image_file not in images_files]
    if len(rejected_images_files) == 0:
        return
    if rejected_images_files[-1] == raw_images_files[-1]:
        rejected_images_files = rejected_images_files[:-1]

    out_images_files = \
        list(map(lambda f: osp.join(out_images_folder, f), rejected_images_files))
    rejected_images_files = \
        list(map(lambda f: osp.join(raw_images_folder, f), rejected_images_files))

    for rejected_image_file, out_image_file in zip(rejected_images_files, out_images_files):
        move(rejected_image_file, out_image_file)
