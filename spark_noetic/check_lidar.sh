#!/usr/bin/env bash
PATH=/bin:/sbin:/usr/bin:/usr/sbin:/usr/local/bin:/usr/local/sbin:~/bin
export PATH

#=================================================
#	System Required: Ubuntu 20.04
#        litian.zhuang@nxrobo.com
#	Site: http://www.nxrobo.com/
#    SPARK技术讨论与反馈QQ群：6646169  8346256
#=================================================

GAME_ENABLE="no"
sh_ver="2.0"
filepath=$(cd "$(dirname "$0")"; pwd)
Green_font_prefix="\033[32m" && Red_font_prefix="\033[31m" && Green_background_prefix="\033[42;37m" && Red_background_prefix="\033[41;37m" && Yellow_background_prefix="\033[43;37m" && Font_color_suffix="\033[0m" && Yellow_font_prefix="\e[1;33m" && Blue_font_prefix="\e[0;34m"
Info="${Green_font_prefix}[信息]${Font_color_suffix}"
Error="${Red_font_prefix}[错误]${Font_color_suffix}"
Warn="${Yellow_font_prefix}[警告]${Font_color_suffix}"
Tip="${Green_font_prefix}[注意]${Font_color_suffix}"
Separator_1="——————————————————————————————"
password="spark"
Version=$(lsb_release -r --short)
Codename=$(lsb_release -c --short)
OSDescription=$(lsb_release -d --short)
OSArch=$(uname -m)
calibra_default="${filepath}/../.ros/camera_info"


calibration="calibration"
color_block="color_block"

TYPE_LIDAR=$(cat /opt/lidar.txt)
echo ${TYPE_LIDAR}
if [[ "${TYPE_LIDAR}" == "ydlidar_g2" ]]; then
	LIDARTYPE="ydlidar_g2"
elif [[ "${TYPE_LIDAR}" == "3iroboticslidar2" ]]; then
	LIDARTYPE="3iroboticslidar2"
else
	echo "暂不支持的雷达：${TYPE_LIDAR}，使用默认的杉川雷达运行"
	LIDARTYPE="3iroboticslidar2"
fi

#检查系统要求
check_sys(){
        if [[ "${Version}" == "20.04" ]]; then
                ROS_Ver="noetic"
        else
                echo -e "${Error} 当前系统 ${OSDescription} ,此版本只适配ubuntu20.04 ROS noetic!" && exit 1
        fi
}

#检查设备连接
# check_dev(){
# 	#检查底盘
# 	if [ ! -n "$(lsusb -d 1a86:7523)" ]; then
# 		echo -e "${Error} 底盘没有正确连接，请确认正确连接！！"
# 	fi
# 	#检查机械臂
# 	if [ ! -n "$(lsusb -d 2341:0042)" ]; then
# 		echo -e "${Error} 机械臂没有正确连接，请确认正确连接！！"
# 	fi
	
# 	#检查摄像头
# 	check_camera
# 	#检查雷达
# 	check_lidar
# }

#检查雷达设备
check_lidar(){

	lidar_flag=0

	#检查使用哪种设备
	if [ -n "$(lsusb -d 10c4:ea60)" ]; then
		lidar_flag=$[$lidar_flag + 1]
	fi



	if [ $lidar_flag -ge 2 ]; then
		echo -e "${Warn} 正在使用多个雷达设备，请退出并拔掉其中一个再使用!"
		echo -e "${Warn} 退出请输入：Ctrl + c！"
	elif [ $lidar_flag -eq 1 ]; then
		echo -e "${Info} 正在使用${LIDARTYPE}雷达"
	elif [ $lidar_flag -eq 0 ]; then
		echo -e "${Error} 没有找到雷达，请确认雷达已正确连接！！"
	fi	
}
#检查摄像头设备
check_camera(){

	camera_flag=0
	calibra_backup="/opt/nxrobo/camera_info"

	#检查使用哪种设备
	if [ -n "$(lsusb -d 2bc5:0403)" ]; then
		CAMERATYPE="astrapro"

		camera_flag=$[$camera_flag + 1]
	fi
	if [ -n "$(lsusb -d 2bc5:0401)" ]; then
		CAMERATYPE="astra"

		camera_flag=$[$camera_flag + 1]
	fi
	if [ -n "$(lsusb -d 8086:0b07)" ]; then
		CAMERATYPE="d435"

		camera_flag=$[$camera_flag + 1]
	fi

	if [ $camera_flag -ge 2 ]; then
		echo -e "${Warn} 正在使用多个摄像头设备，请退出并拔掉其中一个再使用!"
		echo -e "${Warn} 退出请输入：Ctrl + c！"
	elif [ $camera_flag -eq 1 ]; then
		echo -e "${Info} 正在使用${CAMERATYPE}摄像头"
	elif [ $camera_flag -eq 0 ]; then
		echo -e "${Error} 没有找到摄像头，请确认摄像头已正确连接！！"
	fi	
	
	#根据摄像头类型检查标定文件和备份文件
	case $CAMERATYPE in
		astrapro) 
			#检查是否有默认文件夹和备份文件夹
			if [ ! -d "$calibra_default" ]; then
				mkdir -p "$calibra_default"
			fi

			if [ ! -d "$calibra_backup" ]; then
				echo -e "${Info} 创建标定文件备份文件夹..."
				sudo mkdir  -p "$calibra_backup"
			fi
			#如果没有就拷贝一份
			if [ ! -f "$calibra_backup/camera.yaml" ]||[ ! -f "$calibra_backup/depth_Astra_Orbbec.yaml" ]; then
				if [ -f "$calibra_default/camera.yaml" ]&&[ -f "$calibra_default/depth_Astra_Orbbec.yaml" ]; then
					echo -e "${Info} 备份标定文件..."
					sudo cp -r "$calibra_default/." "$calibra_backup/."
				elif [ ! -f "$calibra_default/camera.yaml" ]||[ ! -f "$calibra_default/depth_Astra_Orbbec.yaml" ]; then
					echo -e "${Warn} 标定文件缺少其中一个，请确认rgb和depth标定文件都存在"
				fi
			elif [ -f "$calibra_backup/camera.yaml" ]&&[ -f "$calibra_backup/depth_Astra_Orbbec.yaml" ]; then
				if [ ! -f "$calibra_default/camera.yaml" ]; then
					echo -e "${Info} 缺少彩色摄像头标定文件，已拷贝"
					cp "$calibra_backup/camera.yaml" $calibra_default
				fi
				if [ ! -f "$calibra_default/depth_Astra_Orbbec.yaml" ]; then
					echo -e "${Info} 缺少深度摄像头标定文件，已拷贝"
					cp "$calibra_backup/depth_Astra_Orbbec.yaml" $calibra_default	
				fi
			fi
			;;
		astra)
			#astra自带标定文件			
			#如果检查到有标定文件就删除
			if [ -d "$calibra_default" ]; then
				rm -rf "$calibra_default"
			fi
			;;
	esac

}

#安装ROS完整版
install_ros_full(){
		sudo sh -c 'echo 1 > /proc/sys/net/ipv6/conf/all/disable_ipv6'
		sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
		sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654		
		sudo apt-get update
		sudo apt-get install -y ros-${ROS_Ver}-desktop-full
		sudo rosdep init
		rosdep update
		echo "source /opt/ros/${ROS_Ver}/setup.bash" >> ~/.bashrc
		source /opt/ros/${ROS_Ver}/setup.bash
		sudo apt-get install -y python-rosinstall python-rosinstall-generator python-wstool build-essential
}

#检测是否需要安装完整版
check_install_ros_full(){
	if [ -f "/usr/bin/rosversion" ]; then
		ROSVER=`/usr/bin/rosversion -d`
		if [ $ROSVER ]; then
			echo -e "${Tip} 检测到当前系统已安装了ROS的${ROSVER}版本!" 
			echo && stty erase ^? && read -p "请选择是否继续安装？ y/n：" choose
			if [[ "${choose}" == "y" ]]; then
				echo -e "${Info}准备安装ROS系统！" 
			else
				exit
			fi
		fi
	fi
	install_ros_full 
}


check_lidar 