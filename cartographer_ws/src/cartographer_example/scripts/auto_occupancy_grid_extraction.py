import subprocess
import os
import argparse
from tqdm import tqdm


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('-pbstream-fld', '--pbstream-folder', required=True, type=str)
    parser.add_argument('-out-fld', '--out-folder', required=True, type=str)
    return parser


def get_pbstream_filenames(pbstream_folder):
    pbstream_filenames = os.listdir(pbstream_folder)
    pbstream_filenames = [pbstream for pbstream in pbstream_filenames if pbstream.endswith('pbstream')]
    pbstream_filenames.sort()
    return pbstream_filenames


def auto_occupancy_grid_extraction(pbstream_folder, out_folder):
    os.makedirs(out_folder, exist_ok=True)
    pbstreams = get_pbstream_filenames(pbstream_folder)
    topic = '/save_occupancy_grid'

    for pbstream in tqdm(pbstreams):
        pbstream_filename = os.path.abspath(os.path.join(pbstream_folder, pbstream))
        out_filename = os.path.abspath(os.path.join(out_folder, '.'.join(pbstream.split('.')[:-1] + ['png'])))

        command = "rosrun cartographer_ros cartographer_pbstream_map_publisher -pbstream_filename {} \
-map_topic {}".format(pbstream_filename, topic)
        publish_occupancy_grid = subprocess.Popen(command.split())

        command = "rosrun ros_utils save_occupancy_grid.py -topic {} -out {}".format(topic, out_filename)
        save_occupancy_grid = subprocess.Popen(command.split())

        save_occupancy_grid.communicate()
        assert(save_occupancy_grid.returncode == 0)

        publish_occupancy_grid.terminate()
        publish_occupancy_grid.communicate()
        assert(publish_occupancy_grid.returncode == 0)


if __name__ == '__main__':
    parser = build_parser()
    args = parser.parse_args()
    auto_occupancy_grid_extraction(**vars(args))
