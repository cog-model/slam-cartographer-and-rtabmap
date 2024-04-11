import subprocess_tee
import os
import os.path as osp
from copy import deepcopy


class DockerMounts:
    def __init__(self):
        self.mounts_counter = 0
        self.mounts = dict()  # host_folder -> docker_folder
        self.is_mount_pinned = dict()  # host_folder -> bool
        self.modes = dict()  # host_folder -> str

    def add_file(self, host_file, docker_destination_folder=None, mode='rw'):
        host_folder = osp.dirname(host_file)
        self.add_folder(host_folder, docker_destination_folder=docker_destination_folder, mode=mode)

    def add_folder(self, host_folder, docker_destination_folder=None, mode='rw'):
        host_folder = osp.realpath(osp.expanduser(host_folder))
        if host_folder in self.mounts:
            raise RuntimeError(f"Folder {host_folder} already mounted to {self.mounts[host_folder]}")
        if docker_destination_folder is None:
            docker_destination_folder = f"/mnt/mount_{self.mounts_counter}"
            self.mounts_counter += 1
            mount_pinned = False
        else:
            docker_destination_folder = osp.normpath(docker_destination_folder)
            mount_pinned = True

        self.mounts[host_folder] = docker_destination_folder
        self.is_mount_pinned[host_folder] = mount_pinned
        self.modes[host_folder] = mode
        self._trim_mounts()

    def _trim_mounts(self):
        host_folders_to_remove = set()
        for i, host_folder in enumerate(self.mounts.keys()):
            for j, host_sub_folder in enumerate(self.mounts.keys()):
                if i == j:
                    continue
                if osp.commonpath([host_folder, host_sub_folder]) != host_sub_folder:
                    continue
                if self.is_mount_pinned[host_folder]:
                    continue
                host_folders_to_remove.add(host_folder)

        for host_folder_to_remove in host_folders_to_remove:
            self.mounts.pop(host_folder_to_remove)
            self.is_mount_pinned.pop(host_folder_to_remove)
            self.modes.pop(host_folder_to_remove)

    def resolve(self, host_path):
        host_path = osp.realpath(osp.expanduser(host_path))
        host_folder = None
        for host_folder_to_check in self.mounts.keys():
            if osp.commonpath([host_folder_to_check, host_path]) == host_folder_to_check:
                if host_folder is None:
                    host_folder = host_folder_to_check
                else:
                    if self.is_mount_pinned[host_folder_to_check] and not self.is_mount_pinned[host_folder]:
                        host_folder = host_folder_to_check
                    elif not self.is_mount_pinned[host_folder_to_check] and self.is_mount_pinned[host_folder]:
                        continue
                    elif len(host_folder_to_check) > len(host_folder):
                        host_folder = host_folder_to_check
        if host_folder is None:
            return None

        resolved_path = osp.join(self.mounts[host_folder], osp.relpath(host_path, host_folder))
        resolved_path = osp.normpath(resolved_path)
        return resolved_path

    def get_mount_arguments(self):
        mount_arguments = str()
        for host_folder, docker_folder in self.mounts.items():
            mount_arguments += f"-v {host_folder}:{docker_folder}:{self.modes[host_folder]} "
        return mount_arguments


class DockerContainer:
    def __init__(self, image_name, container_name, user_name=None):
        self.image_name = image_name
        self.container_name = container_name
        self.user_name = user_name

        self.user_argument = f'--user {user_name}' if user_name else ''

        self.default_mounts = DockerMounts()
        self.default_mounts.add_folder("/tmp/.X11-unix", "/tmp/.X11-unix", "rw")
        self.default_mounts.add_folder("/etc/timezone", "/etc/timezone", "ro")
        self.default_mounts.add_folder("/etc/localtime", "/etc/localtime", "ro")

        self._environment_variables = dict()

        self._mounts = None
        self.container_ip = None
        self.home_directory = None

        container_status = self.get_container_status()
        if container_status is not None and container_status == "running":
            self._init()

    def create_containter(self, mounts: DockerMounts=None, net='host'):
        if mounts is None:
            mounts = DockerMounts()

        container_status = self.get_container_status()
        if container_status is None:
            self._run_container(mounts, net)
        else:
            if container_status != "running":
                raise RuntimeError(
                    f"Found container {self.container_name}, but its status is {container_status}. "
                    f"(should be running).")
            else:
                print("Container is already running.")
        self._init()

        container_created = container_status is None
        return container_created

    def _run_container(self, mounts: DockerMounts, net: str):
        docker_command = f"docker run -it -d --rm --privileged --name {self.container_name} " \
            f"--env DISPLAY={os.environ.get('DISPLAY', '')} --env QT_X11_NO_MITSHM=1 " \
            f"--ipc host --net {net} --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all " \
            f"{self.default_mounts.get_mount_arguments()} " \
            f"{mounts.get_mount_arguments()} " \
            f"{self.image_name}"
        returncode = subprocess_tee.run(docker_command, quiet=True).returncode
        if returncode != 0:
            raise RuntimeError(
                f"Could not create container {self.container_name} from image {self.image_name}, "
                f"though container {self.container_name} does not exist. "
                f"Maybe image {self.image_name} does not exist.")

    def _init(self):
        self._mounts = self._get_container_mounts()

        get_container_ip_command = "docker inspect -f '{{range.NetworkSettings.Networks}}{{.IPAddress}}{{end}}' " + self.container_name
        self.container_ip = subprocess_tee.run(get_container_ip_command, quiet=True).stdout.rstrip()

        suppress_redundant_output_command = "touch ~/.sudo_as_admin_successful"
        self.run(suppress_redundant_output_command, quiet=True)

        get_home_directory_command = "cd ~; pwd"
        self.home_directory = self.run(get_home_directory_command, quiet=True).stdout.rstrip()

    def get_container_status(self):
        get_container_status_command = "docker container inspect -f '{{.State.Status}}' " + self.container_name
        result = subprocess_tee.run(get_container_status_command, quiet=True)
        if result.returncode != 0:
            return None
        container_status = result.stdout.rstrip()
        return container_status

    def _get_container_mounts(self):
        get_container_mounts_command = "docker inspect -f '{{json .Mounts}}' " + self.container_name
        result = subprocess_tee.run(get_container_mounts_command, quiet=True)
        if result.returncode != 0:
            raise RuntimeError(
                f"Could not get mounts for container {self.container_name}. "
                f"Maybe container does not exist.")

        true = True
        false = False
        mounts_list = eval(result.stdout.rstrip())
        mounts = DockerMounts()
        for mount_entry in mounts_list:
            if mount_entry['Source'] in self.default_mounts.mounts:
                continue
            assert mount_entry['Type'] == 'bind'
            mounts.add_folder(mount_entry['Source'], mount_entry['Destination'], mount_entry['Mode'])
        return mounts

    def set_environment_variable(self, name: str, value: str, for_all_container=False):
        if for_all_container:
            value = value.replace('\\', '\\\\').replace('\'', '\\\'')
            set_environment_variable_command = f"export {name}=$'{value}'"
            set_environment_variable_command = \
                set_environment_variable_command.replace('\\', '\\\\').replace('\'', '\\\'')
            add_to_bashrc_command = f"echo $'{set_environment_variable_command}' >> ~/.bashrc"
            set_only_if_not_set_command = f"if [[ ! -v {name} || ${name} != $'{value}' ]]; then {add_to_bashrc_command}; fi"

            override_value = self._environment_variables.pop(name, None)  # make sure _environment_variables do not override bashrc variables
            returncode = self.run(set_only_if_not_set_command, quiet=True).returncode
            if override_value is not None:
                self._environment_variables[name] = override_value
            if returncode != 0:
                raise RuntimeError(f"Error adding environment variable {name}={value} to ~/.bashrc")
        else:
            self._environment_variables[name] = value

    def unset_environment_variable(self, name: str, for_all_container=False):
        self._environment_variables.pop(name, None)
        if for_all_container:
            unset_environment_variable_command = f"unset {name}"
            add_to_bashrc_command = f"echo '{unset_environment_variable_command}' >> ~/.bashrc"
            unset_only_if_set_command = f"if [[ -v {name} ]]; then {add_to_bashrc_command}; fi"
            returncode = self.run(unset_only_if_set_command, quiet=True).returncode
            if returncode != 0:
                raise RuntimeError(f"Error adding unset for environment variable {name} to ~/.bashrc")

    def pass_environment_variable(self, name: str, for_all_container=False):
        value = os.getenv(name)
        if value is not None:
            self.set_environment_variable(name, value, for_all_container=for_all_container)

    def _set_environment_variables_command(self):
        set_environment_variables_command = str()
        for name, value in self._environment_variables.items():
            value = value.replace('\\', '\\\\').replace('\'', '\\\'')
            set_environment_variables_command += f"export {name}=$'{value}' && "
        return set_environment_variables_command

    def run(self, command: str, quiet=False):
        command = self._set_environment_variables_command() + command
        command = command.replace('\\', '\\\\').replace('\'', '\\\'')
        docker_command = f"docker exec -it {self.user_argument} {self.container_name} /bin/bash -ic $'{command}'"
        result = subprocess_tee.run(docker_command, quiet=quiet)
        return result

    def run_async(self, command: str, session=''):
        if not session:
            raise RuntimeError("Session name not specified")
        command = self._set_environment_variables_command() + command
        command = command.replace('\\', '\\\\').replace('\'', '\\\'')
        async_command = f"tmux new -d -s {session} /bin/bash -c $'{command}'"
        async_command = async_command.replace('\\', '\\\\').replace('\'', '\\\'')
        docker_command = f"docker exec -it {self.user_argument} {self.container_name} /bin/bash -ic $'{async_command}'"
        returncode = subprocess_tee.run(docker_command, quiet=True).returncode
        if returncode != 0:
            raise RuntimeError(f"Error running command in async mode:\n  {command}")

    def stop_session(self, session: str):
        stop_command = f"(tmux send-keys -t ={session}: C-c) && (tmux a -t ={session} || true)"
        docker_command = f"docker exec -it {self.user_argument} {self.container_name} /bin/bash -ic '{stop_command}'"
        returncode = subprocess_tee.run(docker_command, quiet=True).returncode
        if returncode != 0:
            raise RuntimeError(f"Error stopping session '{session}'")
        return  # TODO: return exit status from tmux session

    def stop_container(self):
        docker_command = f"docker stop {self.container_name}"
        returncode = subprocess_tee.run(docker_command).returncode
        if returncode != 0:
            raise RuntimeError("Error stopping docker container")

    def resolve(self, host_path):
        resolved_path = self._mounts.resolve(host_path)
        if resolved_path is None:
            raise RuntimeError(f"Could not resolve path '{host_path}'")
        return resolved_path
