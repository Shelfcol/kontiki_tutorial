这个项目是从浙大的Lidar_IMU_Calib上借鉴而来，仅用于学习kontiki的用法

内容包含:kontiki使用IMU数据拟合位置和姿态, Pangolin画图, yaml使用


Pangolinan安装:

	third_party文件夹下
		mkdir build-pangolin
		cd build-pangolin
		cmake ../Pangolin/
		make -j8

运行IMU轨迹优化并可视化

	rosrun kontiki_tutorial kontiki_tutorial_node

pangolin轨迹可视化单独轨迹,只需要修改vis_config.yaml里面的文件地址即可,支持TUM和KITTI数据格式
	
	rosrun kontiki_tutorial plotKittiTrajectory

若换了数据, 需要更改config.yaml里面的imu的话题名和文件位置

这里提供了一份含有IMU的数据,在data下面,使用的时候解压即可
		
