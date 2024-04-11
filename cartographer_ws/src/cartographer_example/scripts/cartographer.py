import os.path as osp
from docker_helper import DockerMounts, RosDockerContainer


class CartographerMounts(DockerMounts):
    def __init__(self):
        super().__init__()
        host_ws_folder = osp.join(osp.dirname(__file__), '../../..')
        self.add_folder(host_ws_folder, "/home/docker_cartographer/catkin_ws")


class Cartographer(RosDockerContainer):
    def __init__(self, image_name="cartographer:latest", container_name="cartographer",
            user_name=None):
        super().__init__(image_name, container_name, user_name=user_name)
        self.local_odometry_topic = '/cartographer/tracked_local_odometry'
        self.source_files = [osp.join(osp.dirname(__file__), "../../../devel_isolated/setup.bash")]

    def create_containter(self, mounts: CartographerMounts=None, net='host'):
        if mounts is None:
            mounts = CartographerMounts()
        if type(mounts) is not CartographerMounts:
            raise RuntimeError(f"Type of 'mounts' argument should be CartographerMounts, not {type(mounts)}")

        super().create_containter(mounts=mounts, net=net)

    def build_cartographer(self):
        result = self.run("cd ~/catkin_ws && catkin_make_isolated -DCMAKE_CXX_STANDARD=17 --use-ninja")
        return result

    def run_cartographer(self, config_file,
            load_state_file=None, save_state_file=None):
        arguments = str()

        config_file = self.resolve(config_file)
        arguments += f"config_filename:={config_file} "

        if load_state_file is not None:
            load_state_file = self.resolve(load_state_file)
            arguments += f"load_state_filename:={load_state_file} "

        if save_state_file is not None:
            save_state_file = self.resolve(save_state_file)
            arguments += f"save_state_filename:={save_state_file} "

        result = self.roslaunch("cartographer_example", "cartographer.launch",
            arguments=arguments, source_files=self.source_files)
        return result
