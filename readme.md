If you have problems with libncurses.so.5 on a debian based linux, add:

`deb http://security.ubuntu.com/ubuntu focal-security main universe`

To

`sudo nano /etc/apt/sources.list`

and run

`sudo apt-get install libncurses5`