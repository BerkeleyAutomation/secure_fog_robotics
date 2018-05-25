# ROS gateway [Work in progress]

A minimalistic GDP gateway for ROS.

- `gdp_sink.py`: A ROS node that listens to specified topics and logs them to a
  specified GDP log.
- `gdp_source.py`: A ROS node that subscribes to a given GDP log and publishes
  the contents to appropriate ROS topics.

