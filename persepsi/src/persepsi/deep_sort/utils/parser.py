import os
import yaml
from easydict import EasyDict as edict
import roslib
PKG = 'persepsi'
roslib.load_manifest(PKG)





class YamlParser(edict):
    """
    This is yaml parser based on EasyDict.
    """

    def __init__(self, cfg_dict=None, config_file=None):
        if cfg_dict is None:
            cfg_dict = {}

        if config_file is not None:
            assert(os.path.isfile(config_file))
            with open(config_file, 'r') as fo:
                yaml_ = yaml.load(fo.read(), Loader=yaml.FullLoader)
                cfg_dict.update(yaml_)

        super(YamlParser, self).__init__(cfg_dict)

    def merge_from_file(self, config_file):
        config_file_path = os.path.abspath(__file__+"/../../../" + config_file)
        # mydir = roslib.packages.get_pkg_dir(PKG)
        # temp = os.path.join(mydir, "src/persepsi") 
        # config_file_path = os.path.join(temp, config_file) 
        print(config_file_path)
        with open(config_file_path, 'r') as fo:
            yaml_ = yaml.load(fo.read(), Loader=yaml.FullLoader)
            self.update(yaml_)

    def merge_from_dict(self, config_dict):
        self.update(config_dict)


def get_config(config_file=None):
    return YamlParser(config_file=config_file)


if __name__ == "__main__":
    mydir = roslib.packages.get_pkg_dir(PKG)
    cfg_path = os.path.join(mydir, "/../configs/yolov3.yaml") 
    cfg_merge_path = os.path.join(mydir, "/../configs/deep_sort.yaml") 
    print(cfg_path)
    cfg = YamlParser(config_file=cfg_path)
    cfg.merge_from_file(cfg_merge_path)

    import ipdb
    ipdb.set_trace()
