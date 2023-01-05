uint32 time_flag                        # 系统时间
uint32 plan_data_id 		        # 规划ID，递增   	
LOCAL_COORDINATE plan_frame		# 该规划帧的坐标				           
int16 effective_point_num		# 有效轨迹点个数	            
int16 is_ok				# 有效标志位 0 ～ 无效 1 ～ 有效
WAYPOINT[50] path			# 路径点
PATH_DATA_PROPERTY[50] path_property	# 道路边界
int16 vehicle_command		         #ES=200,ST,AD_SPEED,AD_DISTANCE,AD_POINT,BK_SPEED,BK_POINT,IG,FO       
int16 sys_state				# 系统状态
int16 plan_state			# 规划状态
int32 speed        			# 期望速度
int16 turning_state			# 转向状态
int16 CurrentState			# 当前系统状态
int16 path_state			# 路径状态
VEHICLE2FOLLOW Vehicle2Flo		# 跟随车的信息
BEHAVIOR_POINT Point2Stop		# 停车点
float32 expVelocity			# 期望速度（未使用）
float32 expCurvature			# 期望曲率（未使用）
