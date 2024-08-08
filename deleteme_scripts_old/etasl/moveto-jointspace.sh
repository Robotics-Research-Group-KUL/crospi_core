# use source to run this.
# sends an RunTask action to run a moveto-jointspace task:
ros2 action send_goal -f /RunTask etasl_interfaces/action/RunTask "{task: '$(cat moveto-jointspace.json)' }"