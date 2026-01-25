//302-334

void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg) 
/*livox点云回调函数；const是类型修饰符	表示常量，值不可修；livox_ros_driver::CustomMsg::ConstPtr是Livox 自定义消息的常量指针*/
{
    mtx_buffer.lock();
    //获取互斥锁；锁确保同一时间只有一个线程能执行下面的代码
    double preprocess_start_time = omp_get_wtime();
    /*记录开始时间。omp_get_wtime()-函数调用，来源： OpenMP库（并行计算库），作用： 返回当前时间（单位：秒）*/
    scan_count ++;
    //时间戳检查，msg->header.stamp.toSec()传入的激光雷达数据时间戳  是否小于 上一次激光雷达数据闯入的时间戳  
    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
    }
    //更新时间戳
    last_timestamp_lidar = msg->header.stamp.toSec();
    
    //时间同步检查。abs：雷达和imu时间戳差的绝对值>10，或者雷达和imu数据为空
    if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() && !lidar_buffer.empty() )
    {
        printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n",last_timestamp_imu, last_timestamp_lidar);
    }
 //时间同步设置
    if (time_sync_en && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_imu) > 1 && !imu_buffer.empty())
    {
        timediff_set_flg = true;
        timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;
        printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
    }

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI()); //创建点云对象
    p_pre->process(msg, ptr); //预处理点云，把输入的原始点云msg处理后填入ptr
    lidar_buffer.push_back(ptr);//ptr放入仓库缓冲区
    time_buffer.push_back(last_timestamp_lidar);//保存当前点云时间戳
    
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;//计算处理时间
    mtx_buffer.unlock();//释放锁
    sig_buffer.notify_all();
}
