


int16 gps_flag               #gps info is updated
int16 gps_week
float64 gps_millisecond        #millisecond in a week
int32[2] llhPos    #经纬度，llpos[i]/SC_POS 单位为度
int32[2] gaussPos			  #高斯投影位置,cm，
int32 height                 #cm
int32 pitch                  #欧拉角，单位为0.01度
int32 roll                   #欧拉角，单位为0.01度
int32 azimuth                #欧拉角，单位为0.01度,向东为零度，逆时针0-360
int32 northVelocity          #north速度，单位为cm/s
int32 eastVelocity
int32 velocity_hor
int32 velocity_trk
int32 upVelocity
int32 positionStatus		  #系统运行状态
int16 motion_flag            # motion info is updated
int16 motion_week
int32 motion_millisecond
int32[2] drPos
int32 drHeight
int32 drAzimuth
int32 drNorthVelocity
int32 drEastVelocity
int32 drUpVelocity
int32[2] reserved
int32 number_of_sattelite
float32 variance_x
float32 variance_y
float64 time              #IMU time,millisecond
int32 dr_x                #centimeter
int32 dr_y
int32 dr_z
int32 dr_heading          #0.01degree
int32 dr_roll
int32 dr_pitch
float64 omega_x
float64 omega_y
float64 omega_z
float64 acc_x
float64 acc_y
float64 acc_z
int32 MessageSeqNum
