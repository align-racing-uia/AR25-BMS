# If migrating between CubeMX versions
Delete CMakeFiles folder and regenerate code in CubeMX.
If youre still having problems, regenerate `CMakeLists.txt` as well. (Remember to re-add asl)
# Common Issues
If you have problems with libncurses.so.5 on a debian based linux, add:

`deb http://security.ubuntu.com/ubuntu focal-security main universe`

To

`sudo nano /etc/apt/sources.list`

and run

`sudo apt-get install libncurses5`