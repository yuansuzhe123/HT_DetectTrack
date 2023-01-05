float64 DesiredSpeed			# 期望速度 unit:m/s
float64 DesiredAcc			# 期望加速度 unit：m/s2
float64 CurrentSpeed			# 实际速度 unit:m/s
float64 CurrentAcc			# 实际加速度 unit:m/s2
float64 DesiredBrake			# 期望刹车值 0 ～ 1
float64 CurrentBrake			# 实际刹车值 0 ～ 1
float64 DesiredFuel			# 期望油门值 0 ～ 1
float64 CurrentFuel			# 实际油门值 0 ～ 1
int32 DesiredTransPos			# 期望档位
int32 CurrentTransPos			# 实际档位
int32 HardSwitchOn			# 硬件开关
int32 EmergenceFlag			# 紧急状态
int32 SpeedServoFlag			# 速度跟踪模块状态，0 ～ 正常 大于1 ～ error
int32 HornOnFlag			# 喇叭状态
int32 EmergencyLightonFlag		# 警示灯状态


