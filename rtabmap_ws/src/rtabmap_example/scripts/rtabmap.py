import os.path as osp
from docker_helper import DockerMounts, RosDockerContainer


class RtabmapMounts(DockerMounts):
    def __init__(self):
        super().__init__()
        host_ws_folder = osp.join(osp.dirname(__file__), '../../..')
        self.add_folder(host_ws_folder, "/home/docker_rtabmap/catkin_ws")


class Rtabmap(RosDockerContainer):
    def __init__(self, image_name="rtabmap:latest", container_name="rtabmap",
            user_name=None):
        super().__init__(image_name, container_name, user_name=user_name)
        self.source_files = [osp.join(osp.dirname(__file__), "../../../devel_isolated/setup.bash")]

    def create_containter(self, mounts: RtabmapMounts=None, net='host'):
        if mounts is None:
            mounts = RtabmapMounts()
        if type(mounts) is not RtabmapMounts:
            raise RuntimeError(f"Type of 'mounts' argument should be RtabmapMounts, not {type(mounts)}")

        super().create_containter(mounts=mounts, net=net)

    def build_rtabmap(self):
        result = self.run("cd ~/catkin_ws && catkin_make_isolated -DCMAKE_BUILD_TYPE=Release")
        return result

    def run_rtabmap(self, config_paths,
            load_map_path=None, save_map_path=None,
            save_tracking_results_path=None,
            optimization_results_topic=None, node_name=None,
            use_semantic=False):
        arguments = str()

        config_paths = list(map(self.resolve, config_paths))
        arguments += f"config_paths:={','.join(config_paths)} "

        if load_map_path is not None:
            load_map_path = self.resolve(load_map_path)
            arguments += f"load_map_path:={load_map_path} "

        if save_map_path is not None:
            save_map_path = self.resolve(save_map_path)
            arguments += f"save_map_path:={save_map_path} "

        if save_tracking_results_path is not None:
            save_tracking_results_path = self.resolve(save_tracking_results_path)
            arguments += f"save_tracking_results_path:={save_tracking_results_path} "

        if optimization_results_topic is not None:
            arguments += f"optimization_results:={optimization_results_topic} "

        if node_name is not None:
            arguments += f"node_name:={node_name} "

        arguments += f"accum/subscribe_rgb:={use_semantic} "

        result = self.roslaunch("rtabmap_example", "occupancy_grid_map.launch",
            arguments=arguments, source_files=self.source_files)
        return result
