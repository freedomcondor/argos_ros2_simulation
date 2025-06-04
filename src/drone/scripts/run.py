#!/usr/bin/env python3

launch_template = "src/drone/launch/drone_launch_template.xml"
launch_target = "install/drone/share/drone/launch/drone_launch.xml"

argos_template = "build/drone/scripts/configuration_template.argos"
argos_target = "build/drone/scripts/configuration.argos"

#----------------------------------------------------------------------------
def generate_file(template_name, target_name, replacements) :
	'''
	replacements =
	[
		[RANDOMSEED, str(500)],
		[xxx, yyy],
		[xxx, yyy],
	]
	'''
	with open(template_name, 'r') as file :
		filedata = file.read()

	for i in replacements :
		filedata = filedata.replace(i[0], i[1])

	with open(target_name, 'w') as file:
		file.write(filedata)

#----------------------------------------------------------------------------
def generate_drone_launch_xml(drone_name) :
	txt = '''
	<group> <push-ros-namespace namespace="{}" />
		<!--node pkg="drone" exec="droneFlightSystem" output="screen" /-->
		<!--node pkg="drone" exec="droneSwarmSystem" output="screen" /-->
		<node pkg="drone" exec="droneSwarmSystem" output="log"/>
	</group>
	'''.format(drone_name)
	return txt

#----------------------------------------------------------------------------
def generate_drone_argos_xml(drone_name, x, y, z, th) :
	txt = '''
	<drone id="{}">
		<body position="{},{},{}" orientation="{},0,0"/>
		<controller config="drone_ros2_bridge_controller"/>
	</drone>
	'''.format(drone_name, x, y, z, th)
	return txt


#----------------------------------------------------------------------------
#----------------------------------------------------------------------------
N = 50

drones_launch_xml = ""
for i in range(1, N+1):
	drones_launch_xml += generate_drone_launch_xml("drone" + str(i))
launch_replacements = [
	["DRONES",	drones_launch_xml]
]
generate_file(launch_template, launch_target, launch_replacements)

drones_argos_xml = ""
import random

# Set up random positions within reasonable bounds
x_range = (-10, 10)  # meters
y_range = (-10, 10)  # meters
z_range = (10, 20)     # meters
th_range = (-180, 180) # degrees

p = []
for i in range(N):
    x = random.uniform(x_range[0], x_range[1])
    y = random.uniform(y_range[0], y_range[1])
    z = random.uniform(z_range[0], z_range[1])
    th = random.uniform(th_range[0], th_range[1])
    p.append((x, y, z, th))

for i in range(1, N+1):
	drones_argos_xml += generate_drone_argos_xml("drone" + str(i), p[i-1][0], p[i-1][1],p[i-1][2],0)
argos_replacements = [
	["DRONES",	drones_argos_xml]
]
generate_file(argos_template, argos_target, argos_replacements)

import os
os.system("mv ~/.ros/log ~/.ros/log_history_$(date '+%Y%m%d_%H%M%S')")
#os.system("RCUTILS_LOGGING_MIN_SEVERITY=ERROR ros2 launch drone drone_launch.xml > /dev/null 2>&1")
#os.system("RCUTILS_LOGGING_MIN_SEVERITY=ERROR ros2 launch drone drone_launch.xml")
os.system("ros2 launch drone drone_launch.xml > /dev/null")