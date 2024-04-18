# use source to run this.
# sends an RunTask action to run the move_circle task.
ros2 action send_goal -f /RunTask etasl_interfaces/action/RunTask "{task: '$(cat move_circle.json)' }"

