# rover_slam_selector

Sélectionne deux poses SLAM avec `/env/exterior` (`std_msgs/Bool`) et publie une
seule TF `map -> odom`.

- `false` : pose SLAM intérieure
- `true` : pose SLAM extérieure
- au changement, un offset est calculé pour conserver la continuité
- la TF est calculée avec :
  `T_map_odom = T_map_base_selected * inverse(T_odom_base)`

## Entrées par défaut

- `/slam_toolbox/pose` : `geometry_msgs/PoseWithCovarianceStamped`
- `/visual_slam/pose` : `geometry_msgs/PoseWithCovarianceStamped`
- `/odometry/filtered` : `nav_msgs/Odometry`
- `/env/exterior` : `std_msgs/Bool`

Les deux poses SLAM doivent représenter la pose de `base_link`, pas directement
celle de la caméra. Pour Stella-VSLAM, ajoute un adaptateur caméra -> base_link
si nécessaire.

## Installation

```bash
cd ~/ros2_ws/src
unzip rover_slam_selector.zip
cd ~/ros2_ws

rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select rover_slam_selector
source install/setup.bash
```

## Lancement

```bash
ros2 launch rover_slam_selector slam_selector.launch.py
```

## Test du sélecteur

Intérieur :

```bash
ros2 topic pub --once /env/exterior std_msgs/msg/Bool "{data: false}"
```

Extérieur :

```bash
ros2 topic pub --once /env/exterior std_msgs/msg/Bool "{data: true}"
```

Vérification :

```bash
ros2 topic echo /localization/active_source
ros2 run tf2_ros tf2_echo map odom
```

## Important

Désactive toute autre publication de `map -> odom`, notamment :

- `slam_toolbox` avec publication TF directe ;
- Stella-VSLAM avec `publish_tf:=true` ;
- un autre EKF global ou nœud de localisation globale.

Un seul nœud doit publier `map -> odom`.
