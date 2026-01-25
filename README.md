# fastlio源码解析

一.首先可通过算法复现过程实现从github上扒代码在ubuntu20.01和ros1的系统上跑通

二.雷达数据流向
1.   在launch文件中设置参数问题，不同的雷达型号选择对应的launch文件，不同launch文件的差异主要是读取雷达数据话题名的不同和外差标定的参数不同。
     雷达数据话题名需要与.bag中雷达数据发布的话题名一致才能接受到数据
（我主要在avia型号的雷达对应文件做了注释说明：avia.launch/avia.yaml）

 2.在主程序laserMapping.cpp中
782行   nh.param<int>("preprocess/lidar_type", lidar_type, AVIA);  设置雷达类型为AVIA
845-847   ros::Subscriber sub_pcl = p_pre->lidar_type == AVIA ? \
        nh.subscribe(lid_topic, 200000, livox_pcl_cbk) : \
        nh.subscribe(lid_topic, 200000, standard_pcl_cbk);    判断idar_type == AVIA ?，是的话回调livox_pcl_cbk函数，不是的话回调standard_pcl_cbk

对应的callback函数的作用是将对应雷达的原始点云数据转换为算法可用格式，并且存入仓库。（详细解析看解析对应文件）

3.原始点云数据在经过对应的callback函数预处理后放入仓库。回到主函数
 869    if(sync_packages(Measures))    
 同步包裹：雷达数据仓库和imu数据仓库中两个数据，它会从 lidar_buffer 里拿出一帧雷达点云（比如 0.1 秒那时刻的）。然后去 imu_buffer 里把这段时间对应的所有 IMU 数据（比如 0.0 秒到 0.1 秒之间的几十个 IMU 数据）全部找出来。把它们打包在一起，放进 Measures 这个变量里。
所以该代码的意思是“如果有凑齐的一包新数据（即函数返回真），那么就进入大括号 {} 里面开始进行卡尔曼滤波和建图计算；否则，就跳过，继续下一轮循环等待。”
