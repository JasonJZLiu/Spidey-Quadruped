#!/bin/bash

# Path to root of the project
# if [[ -z "${SPIDEY_ROOT}" ]]
# then
#     echo "Error: Variable \'SPIDEY_ROOT\' is not set." >/dev/stderr
# fi

export SPIDEY_ROOT=${HOME}/projects/spidey_development/spidey_v2/spidey_simulation/

export SPIDEY_PY=${SPIDEY_ROOT}/spidey_py

# Path to omniverse-kit
OVKIT_PATH=${HOME}/Isaac/isaac-sim-2020.2.2007-linux-x86_64-release
# Path for the omniverse-kit application `omniverse-kit`
export KIT_PATH=${OVKIT_PATH}/_build/target-deps/kit_sdk_release/_build/linux-x86_64/release
export CARB_APP_PATH=${KIT_PATH}

# for sourcing bundled packages
export ISAAC_PATH=${OVKIT_PATH}/_build/linux-x86_64/release
. ${ISAAC_PATH}/setup_python_env.sh
. ${KIT_PATH}/setup_python_env.sh
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${KIT_PATH}/plugins:${KIT_PATH}/plugins/carb_gfx:${KIT_PATH}/plugins/rtx:${KIT_PATH}/libs/mdl:${KIT_PATH}/libs/iray

# for extensions packages
export KIT_EXT_PATH=${OVKIT_PATH}/_build/linux-x86_64/release/exts
# add all subdirectories in this path to the python-path
for ext_dirname in ${KIT_EXT_PATH}/*; do PYTHONPATH="${PYTHONPATH}:${ext_dirname}"; done
# export it to make it available in the environment
export PYTHONPATH=${PYTHONPATH}

# for physics extension packages
export KIT_PHYSX_PATH=${OVKIT_PATH}/_build/target-deps/kit_sdk_release/_build/linux-x86_64/release/extsPhysics
# add all subdirectories in this path to the python-path
for ext_dirname in ${KIT_PHYSX_PATH}/*; do PYTHONPATH="${PYTHONPATH}:${ext_dirname}"; done
# export it to make it available in the environment


PYTHONPATH=$PYTHONPATH:/home/jingzhou/projects/spidey_development/spidey_v2/spidey_simulation/spidey_py
export PYTHONPATH

# define alias for launching simulator
#alias ovkit=" bash -c ${OVKIT_PATH}/_build/linux-x86_64/release/omniverse-kit-robotics.sh"

# EOF
