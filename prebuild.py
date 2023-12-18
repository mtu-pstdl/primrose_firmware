from datetime import datetime
from platformio import util

# Add build info to build_info.h so it can be included in the firmware

# Build info is comprised of:
# - Build date
# - Build time
# - Build number (incremented each time this script is run)
# - Build type (debug or release)
# - Build git hash
# - Build git branch
# - Compiler version
# - Machine name

def get_git_hash():
    import subprocess
    return subprocess.check_output(['git', 'rev-parse', 'HEAD']).decode('ascii').strip()

def get_git_branch():
    import subprocess
    return subprocess.check_output(['git', 'rev-parse', '--abbrev-ref', 'HEAD']).decode('ascii').strip()
def get_machine_name():
    import socket
    return socket.gethostname()

# if build_info.h doesn't exist, create it with no content
try:
    open('src/build_info.h')
except IOError:
    open('src/build_info.h', 'w').close()

with open('src/build_info.h') as f:
    lines = f.readlines() # Read the first line to get the build number
    if len(lines) == 0:
        build_number = 1
    else:
        build_number = int(lines[1].split()[2]) + 1 # Increment the build number
    build_date = datetime.now().strftime("%Y-%m-%d")
    build_time = datetime.now().strftime("%H:%M:%S")
    build_type = "DEBUG" # TODO: Get this from the build system
    build_git_hash = get_git_hash()
    build_git_branch = get_git_branch()
    build_machine_name = get_machine_name()

# Write the build info to build_info.h
with open('src/build_info.h', 'w') as f:
    f.write("#pragma once\n")
    f.write("#define BUILD_NUMBER " + str(build_number) + "\n")
    f.write("#define BUILD_NUMBER_STR \"" + str(build_number) + "\"\n")
    f.write("#define BUILD_DATE \"" + build_date + "\"\n")
    f.write("#define BUILD_TIME \"" + build_time + "\"\n")
    f.write("#define BUILD_TYPE \"" + build_type + "\"\n")
    f.write("#define BUILD_GIT_HASH \"" + build_git_hash + "\"\n")
    f.write("#define BUILD_GIT_BRANCH \"" + build_git_branch + "\"\n")
    f.write("#define BUILD_MACHINE_NAME \"" + build_machine_name + "\"\n")

print("Build number: " + str(build_number))
print("Build date: " + build_date)
print("Build time: " + build_time)
print("Build type: " + build_type)
print("Build git hash: " + build_git_hash)
print("Build git branch: " + build_git_branch)
print("Build machine name: " + build_machine_name)
