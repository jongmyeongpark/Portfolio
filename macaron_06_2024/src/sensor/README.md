## `lidar.cpp`
`rosrun [pkg name] lidar_cpp [mode],[mode]`
ex) `rosrun macaron_06 lidar_cpp cone,centroid`

mode
| mode        | topic                    | msg/type              |
|-------------|--------------------------|-----------------------|
| `cone`      | `cluster`                | `lidar_info.cone`     |
| `person`    | `cluster`                | `lidar_info.person`   |
| `delivery`  | `cluster`                | `lidar_info.delivery` |
| `processed` | `processed`              | `PointCloud2`         |
| `minjae`    | `super_minjae_processed` | `PointCloud2`         |
| `centroid`  | `centroid`               | `PointCloud2`         |
