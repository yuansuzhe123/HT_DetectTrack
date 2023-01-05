#the control mode of the formation
# from pad to each vehicle
int8 HEAD_1 #0xFF
int8 HEAD_2 #0xAB
int8 lateralDist
int8 longitudinalDist
int16 formationCtl   # stop=0, ready, running
int16 formationShapde  #LONGITUDINAL=0, HORIZON, HERRINGBONE, INVERTED_HERRINGBONE, PENTAGON, RHOMBUS
float32 formationSpeed
