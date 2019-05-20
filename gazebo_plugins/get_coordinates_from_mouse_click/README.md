# Gazebo plugin to get world coordinate from mouse click

Add plugin path to ~/.gazebo/gui.ini
```
...
[overlay_plugins]
filenames=libgui_click_coordinates_widget.so

```

Update GAZEBO_PLUGIN_PATH with the path to the .so

gzserver &

gzclient


You can also include the plugin in a world file (and dont modify gui.ini), see click_coordinates.world file.

