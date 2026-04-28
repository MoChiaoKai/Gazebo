
## Gazebo ROSMASTER X3 Simulation Project

This project uses Gazebo to simulate the Yahboom ROSMASTER X3 robot, providing a complete ROS 2 Humble environment with automated dispatch, navigation, and teleoperation features. 

---

## Usage

### 1. Launch ROS 2 and Gazebo Simulation
Start the Gazebo simulation environment by running the `start_gazebo.sh` script. This will launch Gazebo with the appropriate file and initialize the Yahboom ROSMASTER X3 nodes.

### 2. Run the Dispatch Main Program
In the terminal, run `run_amr_schedule_static.py` and specify your schedule file with the `--schedule` parameter:

```bash
cd /home/mo/Gazebo
python3 run_amr_schedule_static.py --schedule amr_schedule.jsonl
```

The system will automatically:
1. Parse `amr_schedule.jsonl`.
2. Read `temp.world` and build the A* map.
3. Display dispatch information for each round.
4. Call `go_to_point.py` to start driving the robots.

## Dependencies

- Python 3.10+
- ROS 2 Humble
- Gazebo Fortress
- Yahboom ROSMASTER related packages (`yahboom_rosmaster_gazebo`)