#the individual vehicles for detection
int8 HEAD_1 #0xFF
int8 HEAD_2 #0xAA
int8 id  				#the id of the detected vehicle 
int8 flag_valid   			# 1: valid, 0:invalid
int8 role 				#the role in the foarmation
int8 flag   			#detection method, lidar=0, v2v=1
int8 autoMode 			# 1: auto mode, 0: manual mode
int8 formationShape 		# LONGITUDINAL=0  HORIZON  HERRINGBONE INVERTED_HERRINGBONE PENTAGON RHOMBUS
int8 errCode
int16 width 				#the width of vehicle
int16 length				#the length of vehicle
float64 time_stamp	
float64 gpsX   			#0.01 m
float64 gpsY				#0.01 m
float32 heading			#0.01 deg
float32 speed				# cm/s
float32 formationSpeed			# expectedSpeed for formation cm/s
float32 expectedSpeed  		# expectedSpeed for individual vehicle cm/s
float32 acc_x				# 0.01 m/s^2
float32 acc_y				# 0.01 m/s^2
SpeedSliceInfo[5] SpeedInfo 		# speed information for individual vehicle
WAYPOINT[60] Path			# planned path
int16 effectivePathPointNum		# effectivePathPointNum for Path
WAYPOINT Vehicle2Flo			# destination position
uint8[3] reserved
