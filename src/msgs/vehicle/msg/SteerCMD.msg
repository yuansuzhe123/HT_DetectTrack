uint8 ValidFlag			#  命令有效标志位 0 ～ 无效 1 ～ 有效
float64 DesiredWheelAngle            	#  期望前轮转角值 左为正，右为负 unit:°
float64 DesiredAnglularAccleration	#  期望前轮转角速度 左为正，右为负 unit:°/s
float64 DesiredCurvature		#  期望曲率 左为正，右为负 unit: 1/m
uint8  Horn			#  喇叭 0 ～ 关 1 ～ 开 2 ～ 闪烁
uint8  LeftLightCtrlCmd      	#  左转向控制命令 0 ～ 关 1 ～ 开 2 ～ 闪烁
uint8  RightLightCtrlCmd     	#  右转向控制命令 0 ～ 关 1 ～ 开 2 ～ 闪烁

