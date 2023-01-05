int32   SteerServoFlag			# 曲率跟踪模块状态，0 ～ 正常 大于1 ～ error
float64 ActualFrontWheelAngle		# 实际前轮转角，unit：° 
float64 DesiredFrontWheelAngle		# 期望前轮转角，unit：°
float64 ActualCurvature			# 实际曲率，unit:1/m
float64 DesiredCurvature		# 期望曲率，unit:1/m
int32   LeftLightFlag			# 左转向灯状态 0 ～ 关 1 ～ 开 2 ～ 闪烁
int32   RightLightFlag			# 右转向灯状态 0 ～ 关 1 ～ 开 2 ～ 闪烁
