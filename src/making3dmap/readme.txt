txttopcd: put the poles.txt in the data folder,
          change the txt_name of the demo.launch into the name of the txt file,
          and change the pcd_name into a name what you want. run the demo.launch, you will get the map.

gpstopcd: 将bag包内的gps数据转换成gps轨迹的pcd文件，可在launch文件内命名，并在rviz内查看轨迹

making3dmap： 可以将背景地图，odom轨迹和杆状物地图融合成3d map，并将该3d map与gps轨迹对比进行旋转平移到一致，launch文件内设置相应的参数

