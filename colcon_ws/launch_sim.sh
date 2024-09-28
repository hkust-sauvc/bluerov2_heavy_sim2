#!/bin/bash

SESSION="rov_sim"

# Create a new session, detached
tmux new-session -d -s $SESSION

# Create the first window and send initial commands
tmux rename-window -t $SESSION:0 'Main'
tmux send-keys -t $SESSION:0 'source src/orca4/setup.bash' C-m
tmux send-keys -t $SESSION:0 'ros2 launch orca_bringup sim_launch.py' C-m

# Split the window horizontally and run another command in the new pane
tmux split-window -h -p 50
tmux send-keys -t $SESSION:0.1 'ros2 run joy joy_node' C-m

tmux split-window -v -p 50
tmux send-keys -t $SESSION:0.2 'python3 joy_control.py' C-m

# Select the first pane to be active when attaching
tmux select-pane -t $SESSION:0.0

# Function to kill processes
cleanup() {
  tmux send-keys -t $SESSION:0.0 C-c
  tmux send-keys -t $SESSION:0.1 C-c
  sleep 1  # Give some time to shutdown
  tmux kill-session -t $SESSION
}

# Trap SIGINT to cleanup
trap cleanup SIGINT

# Attach to the session
tmux attach-session -t $SESSION