import os.path as osp
from docker_helper import RosDockerContainer


class EvaluationOutputPathsHelper:
    def __init__(self, output_folder, out_files_prefix, subfolders=('',)):
        self.output_folder = output_folder
        self.out_files_prefix = out_files_prefix
        self.subfolders = subfolders

        self.generate_out_paths()

    def generate_out_paths(self):
        self.results_rosbag_file = osp.join(self.output_folder, '{}.bag'.format(self.out_files_prefix))

        self.gt_poses_folder = dict()
        self.results_poses_folder = dict()
        self.output_metrics_folder = dict()
        self.gt_poses_file = dict()
        self.results_poses_file = dict()
        self.trajectories_rosbag_file = dict()
        for subfolder in self.subfolders:
            if subfolder == '':
                self.gt_poses_folder[''] = osp.join(self.output_folder, 'gt')
                self.results_poses_folder[''] = osp.join(self.output_folder, 'results')
                self.output_metrics_folder[''] = dict()
                for projection in ['xy', 'xz', 'yz']:
                    self.output_metrics_folder[''][projection] = \
                        osp.join(self.output_folder, 'output_{}'.format(projection))
                self.gt_poses_file[''] = \
                    osp.join(self.gt_poses_folder[''], '{}.txt'.format(self.out_files_prefix))
                self.results_poses_file[''] = \
                    osp.join(self.results_poses_folder[''], '{}.txt'.format(self.out_files_prefix))
                self.trajectories_rosbag_file[''] = \
                    osp.join(self.output_folder, '{}_trajectories.bag'.format(self.out_files_prefix))
            else:
                self.gt_poses_folder[subfolder] = osp.join(self.output_folder, subfolder, 'gt')
                self.results_poses_folder[subfolder] = osp.join(self.output_folder, subfolder, 'results')
                self.output_metrics_folder[subfolder] = dict()
                for projection in ['xy', 'xz', 'yz']:
                    self.output_metrics_folder[subfolder][projection] = \
                        osp.join(self.output_folder, subfolder, 'output_{}'.format(projection))
                self.gt_poses_file[subfolder] = \
                    osp.join(self.gt_poses_folder[subfolder], '{}_{}.txt'.format(self.out_files_prefix, subfolder))
                self.results_poses_file[subfolder] = \
                    osp.join(self.results_poses_folder[subfolder], '{}_{}.txt'.format(self.out_files_prefix, subfolder))
                self.trajectories_rosbag_file[subfolder] = \
                    osp.join(self.output_folder, subfolder, '{}_trajectories_{}.bag'.format(self.out_files_prefix, subfolder))


def prepare_poses_for_evaluation(ros_docker: RosDockerContainer, gt_rosbag_files, transforms_source_file,
            output_paths_helper: EvaluationOutputPathsHelper, gt_topic, results_topic,
            max_union_intersection_time_difference, max_time_error, max_time_step, subfolder=''):

    args = ''
    args += '-gt-bags {} -gt-topic {} '.format(' '.join(gt_rosbag_files), gt_topic)
    args += '-res-bags {} -res-topic {} '.format(output_paths_helper.results_rosbag_file, results_topic)
    args += '-out-gt {} '.format(output_paths_helper.gt_poses_file[subfolder])
    args += '-out-res {} '.format(output_paths_helper.results_poses_file[subfolder])
    if transforms_source_file:
        args += '-transforms-source {} '.format(transforms_source_file)
    args += '-out-trajectories {} '.format(output_paths_helper.trajectories_rosbag_file[subfolder])
    args += '--max-union-intersection-time-difference {} --max-time-error {} --max-time-step {} '.format(
        max_union_intersection_time_difference, max_time_error, max_time_step)
    
    return_code = ros_docker.rosrun("ros_utils", "prepare_poses_for_evaluation.py", args,
            source_files=osp.join(ros_docker.home_directory, 'ros_utils_ws/devel/setup.bash')).returncode
    if return_code != 0:
        return False
    return True


def run_evaluation(ros_docker: RosDockerContainer, output_paths_helper: EvaluationOutputPathsHelper, projection='xy'):
    for subfolder in output_paths_helper.subfolders:
        args = ''
        args += '--dir_gt {} '.format(output_paths_helper.gt_poses_folder[subfolder])
        args += '--dir_result {} '.format(output_paths_helper.results_poses_folder[subfolder])
        args += '--dir_output {} '.format(output_paths_helper.output_metrics_folder[subfolder][projection])
        args += '--gt_format kitti --result_format kitti --projection {} '.format(projection)
        evaluate_poses_script_file = osp.join(ros_docker.home_directory, 'slam_validation/evaluate_poses.py')
        return_code = ros_docker.run("python3 {} {}".format(evaluate_poses_script_file, args)).returncode
        if return_code != 0:
            raise RuntimeError("Error on evaluation")
