#!/usr/bin/env bash
# Run as interactive shell so .bashrc is sourced (matches manual workflow exactly)
if [ -z "$G1_INTERFACE" ]; then
    echo "ERROR: G1_INTERFACE environment variable is not set."
    echo "Set it to your network interface, e.g.: G1_INTERFACE=eno2"
    exit 1
fi
if [ "${G1PILOT_RESTRICT_DDS_INTERFACE:-1}" = "1" ]; then
    G1PILOT_SETUP_URI_CMD="source setup_uri.sh ${G1_INTERFACE} &&"
else
    G1PILOT_SETUP_URI_CMD=""
fi
exec bash -ic "
cd /ros2_ws &&
./cbuild \${G1PILOT_CBUILD_TARGET:-} &&
${G1PILOT_SETUP_URI_CMD}
source install/setup.bash &&
ros2 launch g1pilot \${G1PILOT_LAUNCH_FILE:-bringup_launcher.launch.py} \${G1PILOT_LAUNCH_ARGS:-enable_collision_avoidance:=\${ENABLE_COLLISION_AVOIDANCE:-false}}
"
#unset RMW_IMPLEMENTATION &&
