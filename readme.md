# If migrating between CubeMX versions
Delete CMakeFiles folder and regenerate code in CubeMX.
If youre still having problems, regenerate `CMakeLists.txt` as well. (Remember to re-add asl)
# Common Issues
## Ncurses
If you have problems with libncurses.so.5 on a debian based linux, add:

`deb http://security.ubuntu.com/ubuntu focal-security main universe`

To

`sudo nano /etc/apt/sources.list`

and run

```
sudo apt-get update
sudo apt-get install libncurses5
```

## float_t
The w25q file defines a type named `float_t` thats based on a type that does not exist in our build environment. Just delete this line in `libs.h` and it should work.