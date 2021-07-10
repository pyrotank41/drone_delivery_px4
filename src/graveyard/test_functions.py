def roll_pitch(x_deg, y_deg):
    
    angle_threshold = 2
    pitch = 0
    roll = 0
    
    if(abs(x_deg) < angle_threshold): roll = x_deg
    else: roll = angle_threshold*(x_deg//abs(x_deg))
    
    if(abs(y_deg) < angle_threshold): pitch = y_deg
    else: pitch = angle_threshold*(y_deg//abs(y_deg))
 
    print(roll, pitch)
    
roll_pitch(-1,-1)
roll_pitch(1,1)
roll_pitch(-1,1)
roll_pitch(1,-1)
roll_pitch(-3,-3)
roll_pitch(3,3)
roll_pitch(-3,3)
roll_pitch(3,-3)
