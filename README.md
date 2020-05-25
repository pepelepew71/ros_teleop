# ros_teleop

## Requirements

```bash
sudo apt install jstest-gtk
sudo apt install ros-kinetic-joy
sudo apt install ros-kinetic-teleop-twist-keyboard
sudo apt install ros-kinetic-twist-mux
```

## Note

The teleop_twist_keyboard will continuously publish cmd_vel when hold the key pressed. The joy_node can use parameter autorepeat_rate to resend the non-changing state. This two informations are important for twist_mux.
