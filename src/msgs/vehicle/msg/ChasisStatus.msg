uint32 sta_code 		# 底层控制器状态码
uint32 err_code 		# 底层控制器错误码
uint8 horn    			# 喇叭 0～OFF 1～ON 2～BLINK
uint8 left_signal_light    	# 左转向灯 0～OFF 1～ON 2～BLINK
uint8 right_signal_light    	# 右转向灯 0～OFF 1～ON 2～BLINK
uint8 headlight    		# 前大灯   0～OFF 1～ON 2～BLINK
float64 lon_acc 		# 纵向加速度，m/s2
float64 lat_acc			# 侧向加速度，m/s2
float64 yaw_rate		# 航向角速度
float64 exp_wheel_angle	       	# 期望前轮转角,unit:°
float64 exp_fuel		# 期望油门值，0~1
float64 exp_brake		# 期望刹车值，0~1
float64 exp_gear		# 期望档位,1～N 2～D 3～R 4～P
float64 cur_spd			# 实际速度
float64 cur_wheel_angle       	# 实际前轮转角
float64 cur_wheel_angle_spd	# 实际前轮转速
float64 cur_fuel		# 实际油门值，0~1
float64 cur_brake		# 实际刹车值，0~1
float64 cur_gear		# 实际档位,1～N 2～D 3～R 4～P
float64 oil             	# 电池电量/油量，0~1
float64 rl_whl_spd		# 左后轮速度，m/s
float64 rr_whl_spd		# 右后轮速度，m/s
