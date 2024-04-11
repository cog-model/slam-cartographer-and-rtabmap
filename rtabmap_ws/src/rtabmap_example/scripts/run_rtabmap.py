import argparse
import time
import os
import os.path as osp
import glob
import shutil
if __name__ == '__main__':
    from rtabmap import Rtabmap, RtabmapMounts
else:
    from .rtabmap import Rtabmap, RtabmapMounts


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('-lm', '--local-mapping', action='store_true')
    parser.add_argument('--load-map', type=str)
    parser.add_argument('--save-map', type=str)
    parser.add_argument('--save-tracking-results', type=str)

    parser.add_argument('-sem', '--use-semantic', action='store_true')

    parser.add_argument('--log-rosbag', action='store_true')
    parser.add_argument('--move-rosbags-to', type=str)

    parser.add_argument('--build', action='store_true')
    return parser


def removeOldRosbagLogFiles(folder, max_total_size):  # in MB
    files = sorted(glob.glob(folder + "/*.bag") + glob.glob(folder + "/*.bag.active"))
    sizes = list(map(osp.getsize, files))
    sizes = [size / 1024 / 1024 for size in sizes]

    accum_size = 0
    i = len(files)
    while accum_size <= max_total_size:
        i -= 1
        if i < 0:
            break
        accum_size += sizes[i]

    i += 1
    for j in range(i):
        os.remove(files[j])


def moveRosbagsTo(from_folder, to_folder):
    files = sorted(glob.glob(from_folder + "/*.bag"))
    for file in files:
        shutil.move(file, to_folder)


def run_rtabmap(**kwargs):
    parser = build_parser()
    args = parser.parse_args()
    args.__dict__.update(kwargs)

    rtabmap = Rtabmap()
    rtabmap.create_containter()

    # build and exit
    if args.build:
        rtabmap.build_rtabmap()
        exit(0)

    config_paths = [osp.join(osp.dirname(__file__), "../config/husky.yaml")]
    if args.local_mapping:
        config_paths.append(osp.join(osp.dirname(__file__), "../config/husky_enable_local_mapping.yaml"))
        node_name = "occupancy_grid_local_map"
    else:
        node_name = "occupancy_grid_map"

    time_str = time.strftime("%Y-%m-%d_%H.%M.%S")
    catkin_ws_folder = osp.abspath(osp.join(osp.dirname(__file__), "../../.."))
    logs_folder = osp.abspath(osp.expanduser(osp.join(catkin_ws_folder, "rtabmap_logs")))
    os.makedirs(logs_folder, exist_ok=True)

    if args.log_rosbag:
        removeOldRosbagLogFiles(logs_folder, max_total_size=1024)
        topics_to_record = ['/tf', '/tf_static',
            '/cartographer/tracked_local_odometry', '/cartographer/tracked_global_odometry',
            '/cartographer/trajectory_node_list', '/cartographer/constraint_list']
        docker_out_rosbag_log_file = rtabmap.resolve(osp.join(logs_folder, f"{time_str}.bag"))
        rtabmap.rosrun_async("rosbag", "record",
            arguments=f"{' '.join(topics_to_record)} -O {docker_out_rosbag_log_file}", session='rtabmap_rosbag_log')

    results = rtabmap.run_rtabmap(config_paths,
        load_map_path=args.load_map, save_map_path=args.save_map,
        save_tracking_results_path=args.save_tracking_results,
        node_name=node_name, use_semantic=args.use_semantic)

    if args.log_rosbag:
        rtabmap.stop_session('rtabmap_rosbag_log')
        if args.move_rosbags_to:
            os.makedirs(args.move_rosbags_to, exist_ok=True)
            print("Moving rosbag logs to HDD. Please wait...")
            moveRosbagsTo(logs_folder, args.move_rosbags_to)
            print("Done.")

    with open(osp.join(logs_folder, f"{time_str}.txt"), 'w') as f:
        f.write(results.stdout)


if __name__ == '__main__':
    run_rtabmap()
