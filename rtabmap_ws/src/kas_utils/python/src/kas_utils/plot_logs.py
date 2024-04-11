import matplotlib.pyplot as plt
plt.plot()  # fixes crash when calling plt.plot() (caused by 'import cv2')
from kas_utils import is_float
import argparse
import numpy as np


def is_float(string: str):
    try:
        float(string)
        return True
    except:
        return False


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('-log', '--log-file', type=str, required=True)
    parser.add_argument('-x', '--x-field', type=str)
    parser.add_argument('-y', '--y-fields', type=str, nargs='+')
    parser.add_argument('-f', '--only-print-fields', action='store_true')
    return parser


def read_logs(log_file, field_names=None):
    with open(log_file) as f:
        lines = f.readlines()
    if len(lines) == 0:
        return np.empty((0,))

    lines = [line.split() for line in lines]
    num_fields = len(lines[0])
    assert all(len(line) == num_fields for line in lines)
    assert field_names is None or len(field_names) == num_fields

    has_header = any(not is_float(elem) for elem in lines[0])
    assert not (has_header and field_names is not None)

    if has_header:
        field_names = lines[0]
        lines.pop(0)
        if len(lines) == 0:
            return {field_name: np.empty((0,)) for field_name in field_names}

    lines = [list(map(float, line)) for line in lines]
    logs = np.array(lines)
    if field_names is not None:
        logs = {field_name: logs[:, i] for i, field_name in enumerate(field_names)}
    return logs


def plot_logs(log_file, x_field=None, y_fields=None, only_print_fields=False):
    logs = read_logs(log_file)
    if isinstance(logs, np.ndarray):
        if only_print_fields:
            print(None)
            return

        assert x_field is None and y_fields is None
        if logs.size == 0:
            print("Could not plot logs. Log file is empty.")
            return
        if logs.shape[1] > 2:
            print("Could not plot logs. Number of unnamed fields > 2.")
            return

        if logs.shape[1] == 1:
            plt.plot(list(range(len(logs[:, 0]))), logs[:, 0])
        if logs.shape[1] == 2:
            plt.plot(logs[:, 0], logs[:, 1])
    elif isinstance(logs, dict):
        if only_print_fields:
            print(list(logs.keys()))
            return

        assert not (x_field is not None and y_fields is None)
        assert y_fields is None or len(y_fields) > 0
        if len(logs) == 0 or any(field_logs.size == 0 for field_logs in logs.values()):
            print("Could not plot logs. Log file is empty.")
            return
        if x_field is not None and x_field not in logs.keys():
            print(f"Could not plot logs. Could not find x_filed '{x_field}' in logs.")
            return
        if y_fields is not None:
            for y_field in y_fields:
                if y_field not in logs.keys():
                    print(f"Could not plot logs. Could not find y_filed '{y_field}' in logs.")
                    return
        if y_fields is None and len(logs) > 2:
            print("Could not plot logs. Number of named fields > 2, but y_fields is not specified.")
            return

        if y_fields is None:
            if len(logs) == 2:
                x_field, y_field = logs.keys()
            elif len(logs) == 1:
                y_field = list(logs.keys())[0]
            y_fields = [y_field]

        ys = [logs[y_field] for y_field in y_fields]
        if x_field is not None:
            x = logs[x_field]
        else:
            x = list(range(len(ys[0])))
        for y in ys:
            plt.plot(x, y)
    plt.show()


if __name__ == "__main__":
    parser = build_parser()
    args = parser.parse_args()
    plot_logs(**vars(args))
