# Gazebo plugin to get world coordinate from mouse click

It is a system plugin, when you load the plugin you only can click on screen you cant move the view, zoom, ...
Probably the reason is that this plugin is a system plugin instead of gui plugin.

gzserver &
gzclient -g libclick_coordinates.so