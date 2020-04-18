# lrs_tools

## Description

This is a ros package that summarizes LRS-related nodes.



## Contents

### scripts/lrs_to_pc2.py

This node converts LRS scan topic to PointCLoud2 topic.

#### Subscribe

- lrs_topic : sensor_msgs/LaserScan

#### Publish

- lrs_pc2_topic : sensor_msgs/PointCloud2



### launch/lrs_to_pc2.launch

This launch file launches `lrs_to_pc2.py`.

#### Parameters

- lrs_topic : LRS topic name (ex. /lrs/scan)
- lrs_pc2_topic : Converted PointCloud2 topic name



## Usage

```
roslaunch lrs_tools lrs_to_pc2.launch lrs_topic:="/lrs/scan" lrs_pc2_topic:="lrs/scan/pc2"
```



