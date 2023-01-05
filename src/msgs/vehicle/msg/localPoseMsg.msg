uint32    header  #数据头
int64   time	#millisecond
int32 dr_x    #centimeter
int32 dr_y    #centimeter
int32 dr_z    #centimeter
int32 dr_heading #0.01degree
int32 dr_roll   #0.01degree
int32 dr_pitch  #0.01degree
int32 lf_speed #left_front wheel speed,cm/s
int32 rf_speed #right_front wheel speed,cm/s
int32 lr_speed #left_rear wheel speed,cm/s
int32 rr_speed #right_rear wheel speed,cm/s
int32 rot_x #imu三轴陀螺速度 0.01degree/s
int32 rot_y #imu三轴陀螺速度 0.01degree/s
int32 rot_z  #imu三轴陀螺速度 0.01degree/s
int32 acc_x  #imu三轴加速度  0.01m/s^2
int32 acc_y #imu三轴加速度  0.01m/s^2
int32 acc_z  #imu三轴加速度  0.01m/s^2
int32  steer  	#-3000(right)~3000(left) degree  0.01degree/s
int32  brake       #0(zero)~100(full)
int32  fuel          #0(zero)~100(full)
int8 trans  #PARK=0,BACKWARD=1,NEURAL=2,FORWARD=3
int8 VehicleState#车辆状态
int8 mode      #行驶模式 #DIRECT_ACTUATOR=0,REMOTE_PILOT=1,AUTO_PILOT=2
int32 drStatus		#dr运行状态
int32 errorStatus	#错误状态
int32 emergency_flag   #故障标志位
int32 hardswitch_on
