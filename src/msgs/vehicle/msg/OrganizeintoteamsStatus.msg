uint8 Vehicle_mode #驾驶模式 0：手动 1：自动  2：遥控
uint8 Vehicle_gear #档位 0-P 1-R 2-N 3-D
uint8 batteryPower #电池电量 0～100%
uint8 Motot_State #0-耗电 2-发电 3-关闭 4-准备
float64 Error_number #故障代码
float64 Vehicle_angle #CCW = 80°→Byte1~Byte2 = 0X0050；CW = -80°→Byte1~Byte2 = 0XFFB0。
float64 Motor_Speed  #有效值范围：0～65531（数值偏移量-20000表示-20000 r/min～45531 r/min），最小计量单元：1r/min，“0xFF,0xFE”表示异常，“0xFF,0xFF”表示无效。
float64  Motor_Torque #有效值范围：0～65531（数值偏移量-20000表示-2000N*m～4553.1N*m），最小计量单元：0.1N*m，“0xFF,0xFE”表示异常，“0xFF,0xFF”表示无效。（备注：前进时转矩为正值，倒车时转矩为负值）
float64  accumulativemileage #累计里程
float64  Brake_pressure  #0-10Mpa
float64  Motor_LRspeed   #轮胎速度=0.377*物理值*滚动半径（247mm）；偏移量2000，正转=2000+转速；反转=2000-转速
float64  Motor_RRspeed 
float64 cur_spd	 # 实际速度
