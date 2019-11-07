"""A simple script that can be used to inspect yaml files.
"""

# Make script both python2 and python3 compatible
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

try:
    input = raw_input
except NameError:
    pass

# Import python packages
import os
from autolab_core import YamlConfig

# Get file path
file_path = os.path.abspath(os.path.dirname(os.path.realpath(__file__)))
yaml_file_path = os.path.abspath(
    os.path.join(file_path, "../cfg/moveit_scene_constraints.yaml")
)

# Read and display config
cfg = YamlConfig(yaml_file_path)
print(cfg.config)

try:
    cfg.config["jan"]
except KeyError as e:
    print(e)
input("Press enter to close the script>> ")
