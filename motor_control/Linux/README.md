# README.md

### Notice

support motor: GO-M8010-6 motor

not support motor: A1 motor、 B1 motor (Check A1B1 branch for support)

### Build
```bash
mkdir build
cd build
cmake ..
make
```

### Run
Run examples with 'sudo',e.g.
```bash
sudo ./motorctrl
```

查看电机id
查看和修改电机 ID 需要将电机切换到工厂模式，切换前请确保所有电机已经停止工作，
主机也不再向电机发送运动控制指令。
sudo ./swboot /dev/ttyUSB0

修改电机id
changeid [串口号] [原来的 ID] [要修改的 ID]
changeid /dev/ttyUSB0 0 1 :设置电机 ID0 为 ID1
切换前请确保所有电机已经停止工作，主机也不再向电机发送运动控制指令。
例如：将总线上所有 ID 为 15 的电机修改为 ID 0
sudo ./changed /dev/ttyUSB0 15 0


切回电机模式
切换回电机模式
查看和修改电机 ID 会让电机切换到工厂模式，如果不手动切换回电机模式，即使给电
机重新上电也还会进入工厂模式。
进入工厂模式的电机背部绿色指示灯会变成每秒快速闪烁 3 次的状态。
此时使用命令 ./swmotor 即可切换到电机模式，用法为：
swmotor [串口号]
swmotor /dev/ttyUSB0
即可让该 RS485 总线上所有的电机切换到电机模式，此时电机就可以接收运动控制指
令了。
没有固件的电机不会被启动，并且会在终端上显示。
