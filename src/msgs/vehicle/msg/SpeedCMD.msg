uint8 ValidFlag		#  命令有效标志位 0 ～ 无效 1 ～ 有效
float64 DesiredBrake            #  期望刹车值 0 ～ 1 对应 最小刹车值 ～ 最大刹车值
float64 DesiredFuel             #  期望油门值 0 ～ 1 对应 最小油门值 ～ 最大油门值
float64 DesireVelocity		#  期望速度 uint：m/s
uint8  DesireTransPos 		#  期望档位 1～N 2～D 3～R 4～P
uint8  hand_brake		#  手刹 0 ～ 松手刹 1 ～ 拉手刹
uint8  FrontWiperCtrlCmd     	#  前雨刮控制命令 0 ～ 停 1 ～ 慢 2 ～ 中 3 ～ 快 4 ~ 最快
uint8  RearWiperCtrlCmd      	#  后雨刮控制命令 0 ～ 停 1 ～ 慢 2 ～ 中 3 ～ 快 4 ~ 最快
uint8  LowBeamCtrlCmd        	#  近光灯控制命令 0 ～ 关 1 ～ 开 2 ～ 闪烁
uint8  HighBeamCtrlCmd       	#  远光灯控制命令 0 ～ 关 1 ～ 开 2 ～ 闪烁
uint8  BackFogLightCtrlCmd   	#  后雾灯控制命令 0 ～ 关 1 ～ 开 2 ～ 闪烁
uint8  FrontFogLightCtrlCmd  	#  前雾灯控制命令 0 ～ 关 1 ～ 开 2 ～ 闪烁
uint8  PositionLampCtrlCmd   	#  位置灯控制命令 0 ～ 关 1 ～ 开 2 ～ 闪烁
uint8  HarzadCtrlCmd         	#  危险报警灯控制命令 0 ～ 关 1 ～ 开 2 ～ 闪烁
uint8  BrakeLampCtrlCmd      	#  刹车灯控制命令 0 ～ 关 1 ～ 开 2 ～ 闪烁
uint8  ReversingLampCtrlCmd  	#  倒车灯控制命令 0 ～ 关 1 ～ 开 2 ～ 闪烁
uint8  AlarmHornCtrlCmd      	#  喇叭控制命令 0 ～ 关 1 ～ 开 2 ～ 闪烁
uint8 estop

