import yaml

def loadYaml(config_file):
    with open(config_file) as config:
        data = yaml.safe_load(config)
        name = data['robot_name']

if __name__ == "__main__":
    print("test")
    loadYaml("/home/pshah/Software/biconvex_mpc/robots/solo12/robot_info.yaml")