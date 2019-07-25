import subprocess
import sys

test = subprocess.getoutput([sys.executable, "-v", "../perception/setup.py", "egg_info"])
print(test)