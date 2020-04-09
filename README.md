# Sokoban Q learning

## 1 Installation

### Via PIP
```bash
pip install gym-sokoban
```

### From Repository
```bash
git clone git@github.com:mpSchrader/gym-sokoban.git
cd gym-sokoban
pip install -e .
```

## 2 Environment

1. Put Sokoban folder into file:///home/catkin_ws/src 
2. Run 

```bash
roscore
rosrun sokoban server.py
roslaunch sokoban maze.launch
```
3. To change Robot initial position, Check configuration in  launch/maze.launch file
4. To change Room generation, change Random seed in sokoban/scripts/room_utils line 70



## 3. Further Information
Please Refer to https://github.com/mpSchrader/gym-sokoban
