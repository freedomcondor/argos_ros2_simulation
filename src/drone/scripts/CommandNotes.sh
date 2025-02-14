ros2 topic pub -1 /drone1/dronePoseActuator geometry_msgs/msg/Pose "{position: {x: -1.0, y: 0.0, z: 2.0}, orientation: {x: 0.0, y: 0.0, z: 1, w: 0.0}}"

"{waypoints: [{point: {x: 1.0, y: 0.0, z: 1.0}, yaw: 0.0, yaw_flag: true}, \
              {point: {x: 1.0, y: 1.0, z: 1.0}, yaw: 1.57, yaw_flag: true},\
              {point: {x: 0.0, y: 1.0, z: 1.0}, yaw: 3.14, yaw_flag: true} \
             ] \
}" -1
