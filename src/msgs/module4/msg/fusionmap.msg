


int16 gps_flag               #gps info is updated
int16 gps_week
float64 gps_millisecond      #millisecond in a week
int32[2] llhPos    #经纬度，llpos[i]/SC_POS 单位为度
int32[2] gaussPos	     #高斯投影位置,cm，
int32 height                 #cm
int32 pitch                  #欧拉角，单位为0.01度
int32 roll                   #欧拉角，单位为0.01度
int32 azimuth                #欧拉角，单位为0.01度,向东为零度，逆时针0-360
float64 time                 #IMU time,millisecond
int32 dr_x                   #centimeter
int32 dr_y
int32 dr_z
int32 dr_heading             #0.01degree
int32 dr_roll
int32 dr_pitch
uint8[125000] positive_map   #attribute_map
float32 object_x
float32 object_y
