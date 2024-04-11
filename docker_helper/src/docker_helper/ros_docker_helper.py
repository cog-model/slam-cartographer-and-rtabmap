from .docker_helper import DockerMounts, DockerContainer
import os


class RosDockerContainer(DockerContainer):
    def __init__(self, image_name, container_name, user_name=None):
        self.ros_version = None
        super().__init__(image_name, container_name, user_name=user_name)

    def create_containter(self, mounts: DockerMounts=None, net='host'):
        container_created = super().create_containter(mounts=mounts, net=net)
        return container_created

    def _init(self):
        super()._init()

        get_ros_version_command = "rosversion -d"
        self.ros_version = self.run(get_ros_version_command, quiet=True).stdout.rstrip()

        if self.container_ip:
            self.set_environment_variable("ROS_IP", self.container_ip)
        else:
            self.pass_environment_variable("ROS_MASTER_URI")
            self.pass_environment_variable("ROS_IP")

        self.pass_environment_variable("ROS_DOMAIN_ID")
        self.pass_environment_variable("RMW_IMPLEMENTATION")
        self.pass_environment_variable("ROS_LOCALHOST_ONLY")

    def run(self, command, quiet=False):
        if self.ros_version:
            command = f"source /opt/ros/{self.ros_version}/setup.bash && " + command
        result = super().run(command, quiet=quiet)
        return result

    def run_async(self, command, session=''):
        if not session:
            raise RuntimeError("Session name not specified")
        if self.ros_version:
            command = f"source /opt/ros/{self.ros_version}/setup.bash && " + command
        super().run_async(command, session=session)

    def start_roscore(self, wait=True):
        self.run_async("roscore", 'roscore')
        if wait:
            command = "while : ; do rostopic list && break || sleep 0.1; done"
            returncode = self.run(command, quiet=True).returncode
            if returncode != 0:
                raise RuntimeError("Error waiting for ros master")

    def connect_to_ros_master(self, ros_master_ip):
        set_ros_master_uri_command = f"export ROS_MASTER_URI=http://{ros_master_ip}:11311"
        returncode = self.run(f"echo '{set_ros_master_uri_command}' >> ~/.bashrc").returncode
        if returncode != 0:
            raise RuntimeError("Error adding ROS_MASTER_URI to ~/.bashrc")

    def rosrun(self, package, executable, arguments="", source_files=tuple()):
        if isinstance(source_files, str):
            source_files = [source_files]
        source_files = list(map(self.resolve, source_files))
        command = ''.join(f"source {source_file} && " for source_file in source_files)
        command += f"rosrun {package} {executable} {arguments}"
        result = self.run(command)
        return result

    def roslaunch(self, package, launch, arguments="", source_files=tuple()):
        if isinstance(source_files, str):
            source_files = [source_files]
        source_files = list(map(self.resolve, source_files))
        command = ''.join(f"source {source_file} && " for source_file in source_files)
        command += f"roslaunch {package} {launch} {arguments}"
        result = self.run(command)
        return result

    def roslaunch_nopkg(self, launch_file, arguments="", source_files=tuple()):
        if isinstance(source_files, str):
            source_files = [source_files]
        launch_file = self.resolve(launch_file)
        source_files = list(map(self.resolve, source_files))
        command = ''.join(f"source {source_file} && " for source_file in source_files)
        command += f"roslaunch {launch_file} {arguments}"
        result = self.run(command)
        return result

    def rosrun_async(self, package, executable, arguments="", session='', source_files=tuple()):
        if not session:
            raise RuntimeError("Session name not specified")
        if isinstance(source_files, str):
            source_files = [source_files]
        source_files = list(map(self.resolve, source_files))
        command = ''.join(f"source {source_file} && " for source_file in source_files)
        command += f"rosrun {package} {executable} {arguments}"
        self.run_async(command, session)

    def roslaunch_async(self, package, launch, arguments="", session='', source_files=tuple()):
        if not session:
            raise RuntimeError("Session name not specified")
        if isinstance(source_files, str):
            source_files = [source_files]
        source_files = list(map(self.resolve, source_files))
        command = ''.join(f"source {source_file} && " for source_file in source_files)
        command += f"roslaunch {package} {launch} {arguments}"
        self.run_async(command, session)

    def roslaunch_nopkg_async(self, launch_file, arguments="", session='', source_files=tuple()):
        if not session:
            raise RuntimeError("Session name not specified")
        if isinstance(source_files, str):
            source_files = [source_files]
        launch_file = self.resolve(launch_file)
        source_files = list(map(self.resolve, source_files))
        command = ''.join(f"source {source_file} && " for source_file in source_files)
        command += f"roslaunch {launch_file} {arguments}"
        self.run_async(command, session)

    def rosbag_play(self, rosbag_files, arguments=""):
        if isinstance(rosbag_files, str):
            rosbag_files = [rosbag_files]
        rosbag_files = list(map(self.resolve, rosbag_files))
        returncode = self.rosrun("rosbag", "play",
            arguments=f"--clock {arguments} {' '.join(rosbag_files)}").returncode
        if returncode != 0:
            raise RuntimeError("Error playing rosbags")

    def use_sim_time(self, value):
        command = f"rosparam set use_sim_time {str(value)}"
        returncode = self.run(command).returncode
        if returncode != 0:
            raise RuntimeError("Error setting use_sim_time")

    def stop_roscore(self):
        self.stop_session('roscore')
