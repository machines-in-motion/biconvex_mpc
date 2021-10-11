import os
import yaml

class Paths:
    def __init__(self, robot_name):
        self.PROJECT_PATH = os.path.abspath(os.path.dirname(os.path.abspath(__file__)) + "/..")
        self.ROBOT_PATH = (self.PROJECT_PATH + "/robots")
        self.YAML_PATH = (self.ROBOT_PATH + "/" + robot_name + "/robot_info.yaml")
        self.URDF_PATH = (self.ROBOT_PATH + "/" + robot_name + "/urdf/" + robot_name + ".urdf")
        print(self.YAML_PATH)
        print(robot_name)
        self.YAML = self.load_yaml(self.YAML_PATH)

    def load_yaml(self, config_file):
        with open(config_file) as config:
            data = yaml.safe_load(config)
        
        return data