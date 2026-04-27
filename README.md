## Usage

### 1. Start ROS 2 and Gazebo Environment
First, ensure your Gazebo simulation environment and ROSMASTER X3 nodes are running, and the corresponding `temp.world` is loaded.

### 2. Run the Dispatch Main Program
In the terminal, run `run_amr_schedule_static.py` and specify your schedule file with the `--schedule` parameter:

```bash
cd /home/mo/yahboom_ws\
bash start_gazebo.py
python3 run_amr_schedule_static.py --schedule amr_schedule.jsonl
```

The system will automatically:
1. Parse `amr_schedule_1.jsonl`.
2. Read `temp.world` and build the A* map.
3. Display dispatch information for each round (DISPATCH ROUND).
4. Call `go_to_point.py` to start driving the robots.

## Dependencies

- Python 3.10+
- ROS 2 Humble
- Ignition Gazebo
- Yahboom ROSMASTER related packages (`yahboom_rosmaster_gazebo`)