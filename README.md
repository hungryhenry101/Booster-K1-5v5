# robocup
## 项目安装
参考 [Robocup 项目安装指南](https://zabo9hes6ge.feishu.cn/wiki/OERXwKIZoiJmNJkUe00cM3zvnY6)

## 文件目录结构说明
```
└── robocup // 仓库主目录
    ├── build  //  colcon 工具自动生成目录，不用管
    ├── install //  colcon 工具自动生成目录，不用管
    ├── log // colcon 工具自动生成目录，不用管
    └── src // ros2 结点代码都放在这里
        └── brain // robocup 策略代码
            ├── config  // 配置文件目录
                ├── behavior_trees  // behaviorTree 配置文件全放在这里
                    ├── test.xml  // 具体的配置文件
                        ...
                ├── config.yaml  // ros2 配置文件，会被 launch 下的 python 文件加载
                    ...
            ├── include  // 项目头文件
                ├── brain_config.h
                ├── brain_data.h
                ├── brain_log.h
                    ...
            ├── launch  // ros2 启动配置文件
                ├── test_launch.py  // 具体的启动文件，里面也有相应的配置值
                        ...
            ├── src   // 项目源文件
                ├── brain_config.cpp
                ├── brain_data.cpp
                ├── brain_log.cpp
                    ...
            ├── CMakeLists.txt 
            ├── LICENSE  
            ├── package.xml
            ├── .gitignore
            └── README.md
```