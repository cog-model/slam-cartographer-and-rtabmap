import os
import argparse
from docker_helper import RosDockerContainer
from cartographer import CartographerMounts, Cartographer
from auto_evaluation_base import EvaluationOutputPathsHelper, prepare_poses_for_evaluation, run_evaluation


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('-test-bags', '--test-rosbag-files', required=True, type=str, nargs='+', help="rosbag files to test on")
    parser.add_argument('-gt-bags', '--gt-rosbag-files', type=str, nargs='+', help="rosbag files with gt poses")
    parser.add_argument('-gt-topic', '--gt-topic', required=True, type=str, help="topic with gt poses")
    parser.add_argument('-out-fld', '--output-folder', required=True, type=str)
    parser.add_argument('-prefix', '--out-files-prefix', type=str, default='test')

    parser.add_argument('-config', '--config-file', required=True, type=str)
    parser.add_argument('-online', '--use-online-mode', action='store_true')

    parser.add_argument('-transforms-source', '--transforms-source-file', type=str,
        help="rosbag, urdf or launch file to read static transforms from if needed")

    parser.add_argument('--max-union-intersection-time-difference', type=float, default=0.5,
        help="max difference between union and intersection of time ranges where gt and slam poses are set")
    parser.add_argument('--max-time-error', type=float, default=0.03, help="max time error between matched gt and slam poses")
    parser.add_argument('--max-time-step', type=float, default=0.23, help="max time step in gt and slam poses after matching")

    parser.add_argument('--skip-running-cartographer', action='store_true')
    parser.add_argument('--skip-poses-preparation', action='store_true')
    parser.add_argument('--skip-evaluation', action='store_true')

    parser.add_argument('--auto-repeat', action='store_true')
    return parser


def auto_evaluation(test_rosbag_files, gt_topic, output_folder, config_file, gt_rosbag_files=None,
            out_files_prefix='test', use_online_mode=False, transforms_source_file=None,
            max_union_intersection_time_difference=12.0, max_time_error=0.03, max_time_step=0.23,
            skip_running_cartographer=False, skip_poses_preparation=False, skip_evaluation=False):

    os.makedirs(output_folder, exist_ok=True)

    cartographer_mounts = CartographerMounts(files=[test_rosbag_files, gt_rosbag_files, config_file, transforms_source_file], folders=[output_folder])
    cartographer_docker = RosDockerContainer('cartographer:latest', 'cartographer_auto_evaluation')
    cartographer_docker.create_containter(net='bridge', docker_mounts=cartographer_mounts)
    cartographer = Cartographer(cartographer_docker)

    output_paths_helper = EvaluationOutputPathsHelper(cartographer_mounts[output_folder], out_files_prefix)

    if not skip_running_cartographer:
        cartographer_docker.run(f"rm -f {output_paths_helper.results_rosbag_file}")

        cartographer_docker.start_roscore()
        cartographer_docker.use_sim_time(True)
        if transforms_source_file and transforms_source_file.endswith('.launch'):
            cartographer_docker.roslaunch_nopkg_async(cartographer_mounts[transforms_source_file], session='publish_transforms')
        cartographer_docker.rosrun_async("rosbag", "record",
            f'{cartographer.transforms_topic} -O {output_paths_helper.results_rosbag_file} __name:=read_poses', session='read_poses')

        cartographer.run_cartographer(cartographer_mounts[config_file], bag_files=cartographer_mounts[test_rosbag_files],
            sleep_ms_after_first_clock=1000, sleep_ms=1, online=use_online_mode)
        if use_online_mode:
            cartographer_docker.rosbag_play(cartographer_mounts[test_rosbag_files], '-d 1')
            cartographer.stop_cartographer()

        cartographer_docker.run("rosnode kill /read_poses")
        if transforms_source_file and transforms_source_file.endswith('.launch'):
            cartographer_docker.stop_session('publish_transforms')
        cartographer_docker.stop_roscore()

    if not skip_poses_preparation:
        success = prepare_poses_for_evaluation(cartographer_docker,
            cartographer_mounts[gt_rosbag_files] or cartographer_mounts[test_rosbag_files],
            cartographer_mounts[transforms_source_file], output_paths_helper, gt_topic, cartographer.transforms_topic,
            max_union_intersection_time_difference, max_time_error, max_time_step)
        if not success:
            cartographer_docker.stop_container()
            return False

    if not skip_evaluation:
        for projection in ['xy', 'xz', 'yz']:
            run_evaluation(cartographer_docker, output_paths_helper, projection=projection)

    cartographer_docker.stop_container()
    return True


if __name__ == '__main__':
    parser = build_parser()
    args = parser.parse_args()
    kwargs = vars(args)
    auto_repeat = kwargs.pop('auto_repeat')
    repeat_is_needed = True
    while repeat_is_needed:
        success = auto_evaluation(**kwargs)
        repeat_is_needed = not success and auto_repeat
    if success:
        exit(0)
    else:
        exit(1)
