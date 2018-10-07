# About time management
English follows below Japanese.

# 概要
時刻にはシステムクロックとカメラ内部クロックの2種類がある。カメラ内部クロックは正確な周期で動いているように見えるが、ROSではシステムクロックをメインに扱わなければならない。  
そこで、クロック推定値として、初期値にカメラ内部クロックの値を入れ、時刻の進みにもカメラクロックを用いる。
そのまま長時間放置するとクロック推定値とシステムクロックがずれてしまうことを予想して、クロック推定値とシステムクロックでP制御を行う。  

# 使用上の注意
システムクロックとカメラ内部クロックにはずれがあるため、起動してから1秒程度P制御が収束するまで待ってから使用すること。

# Abstract
There are two time clocks; One is system\_clock, the other is camera internal clock.  
Camera internal clock seems to tick accurately, but the main ROS program needs to use system clock.  
So, I decided to use camera internal clock as the initial estimated clock.  
The estimated clock uses camera internal clock tick.  
This algorithm might lead a big gap between system\_clock and estimated clock, so I used P control for the estimated clock to follow system\_clock.  

# Warning
system\_clock and camera internal clock is a bit different, so you should wait for about a second for the P control synchronization between system\_clock and estimated clock.
