alias ronosde="rosnode"
alias bot_cam="rosservice call /bot_cam/on True; rosservice call /zed/on False"
alias zed="rosservice call /zed/on True; rosservice call /bot_cam/on False"
alias captain="roslaunch mrobosub_planning captain.launch"
alias arm="rosservice call /arming/cmd True"
alias disarm="rosservice call /arming/cmd False"

