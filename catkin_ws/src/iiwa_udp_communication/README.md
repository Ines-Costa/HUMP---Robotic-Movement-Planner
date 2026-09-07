Package for starting default app in iiwa robot

```sh
roslaunch iiwa_udp_communication start_app.launch
```

Example for changing robot name and ip, for when using multiple robots for
instance,

```sh
roslaunch iiwa_udp_communication start_app.launch robot_name:='iiwa_left' robot_ip:='172.31.1.254'
```

For the other customizable parameters see the launch files
