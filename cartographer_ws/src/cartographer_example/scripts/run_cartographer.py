import argparse
import time
import os
import os.path as osp
if __name__ == '__main__':
    from cartographer import Cartographer
else:
    from .cartographer import Cartographer


def build_parser():
    parser = argparse.ArgumentParser()
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('-l', '--localization', action='store_true')
    mode.add_argument('-m', '--mapping', action='store_true')
    mode.add_argument('-o', '--odometry', action='store_true')
    parser.add_argument('--load-map', type=str)
    parser.add_argument('--save-map', type=str)

    parser.add_argument('--build', action='store_true')
    return parser


def check_arguments(args):
    if args.build:
        return
    if not args.localization and not args.mapping and not args.odometry:
        raise RuntimeError("Specify cartographer running mode")


def run_cartographer():
    parser = build_parser()
    args = parser.parse_args()
    check_arguments(args)

    cartographer = Cartographer()
    cartographer.create_containter()

    # build and exit
    if args.build:
        cartographer.build_cartographer()
        exit(0)

    if args.localization:
        config_file = osp.join(osp.dirname(__file__), "../config/husky_localization.lua")
    if args.mapping:
        config_file = osp.join(osp.dirname(__file__), "../config/husky_mapping.lua")
    if args.odometry:
        config_file = osp.join(osp.dirname(__file__), "../config/husky_odometry.lua")

    catkin_ws_folder = osp.abspath(osp.join(osp.dirname(__file__), "../../.."))
    logs_folder = osp.abspath(osp.expanduser(osp.join(catkin_ws_folder, "cartographer_logs")))
    cartographer.set_environment_variable("artd_COL_LOG_FOLDER", cartographer.resolve(logs_folder))
    cartographer.set_environment_variable("displacement_COL_LOG_FOLDER", cartographer.resolve(logs_folder))
    cartographer.set_environment_variable("trim_loops_COL_LOG_FOLDER", cartographer.resolve(logs_folder))

    time_str = time.strftime("%Y-%m-%d_%H.%M.%S")
    results = cartographer.run_cartographer(config_file,
        load_state_file=args.load_map,
        save_state_file=args.save_map)

    os.makedirs(logs_folder, exist_ok=True)
    with open(osp.join(logs_folder, f"{time_str}.txt"), 'w') as f:
        f.write(results.stdout)


if __name__ == '__main__':
    run_cartographer()
