# CSE571_Proj

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

1. Put Sokoban folder into file:///home/cse-571/catkin_ws/src 
2. Run 

```bash
roscore
rosrun sokoban server.py
roslaunch search maze.launch
```
To change Robot initial position, Check configuration in  launch/maze.launch file
To change Room generation, change Random seed in sokoban/scripts/room_utils line 70



## 3. Further Information
Please Refer to https://github.com/mpSchrader/gym-sokoban
