import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/darshan/Task-Inductees/task_ws/install/turtlesim_draw'
