import numpy as np
import time

def nav_angle_deg(Rover):
    # Find right-biased heading direction
    if len(Rover.nav_angles) == 0:
        return 0    
    angles = Rover.nav_angles * 180.0/np.pi
    mean_angle = np.mean(angles)

    right_angles = []
    left_angles = []
    for a in angles:
        if a < mean_angle:
            right_angles.append(a)
        elif a > mean_angle:
            left_angles.append(a)
    
    #right_angles = np.select([angles<mean_angle],[angles])
    right_mean = np.mean(right_angles)
    #left_angles = np.select([angles>mean_angle],[angles])
    left_mean = np.mean(left_angles)

    right_path_angles = 0
    center_path_angles = 0
    left_path_angles = 0

    for a in angles:
        if a >= mean_angle-10 and a <= mean_angle+10:
            center_path_angles += 1
        if a >= right_mean-10 and a <= right_mean+10:
            right_path_angles += 1
        if a >= left_mean-10 and a <= left_mean+10:
            left_path_angles += 1
    
    print('angles count {} {} {}'.format(left_path_angles,center_path_angles,right_path_angles))
    if right_path_angles < 100:
        if center_path_angles > 200:
            nav_angle = mean_angle
        else:
            nav_angle = left_mean
    else:
        nav_angle = right_mean
    
    return nav_angle

# normalize angle into -pi:pi range
def normalize(angle_rad):
    rad = angle_rad
    if rad > np.pi:
        rad -= (2*np.pi)
    elif rad < -np.pi:
        rad += (2*np.pi)
    return rad

# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Example:
    # Check if we have vision data to make decisions with

    print('Rover nav_angles {} mode {}'.format(len(Rover.nav_angles),Rover.mode))
    print('Rover progress {} time {}'.format(Rover.progress,time.time()-Rover.time_q))
    
    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'forward':
            Rover.prior_mode = []
            # Check for gold spotted in the vicinity and potentially go search it out
            dx_list = []
            dy_list = []
            for dx in range(-10,10):
                for dy in range(-10,10):
                    if Rover.pos[1]+dy >= 0 and Rover.pos[1]+dy < Rover.worldmap.shape[0] and \
                       Rover.pos[0]+dx >= 0 and Rover.pos[0]+dx < Rover.worldmap.shape[1]:
                        if (Rover.searchmap[int(Rover.pos[1]+dy),int(Rover.pos[0]+dx),0] > 0) & \
                           (Rover.searchmap[int(Rover.pos[1]+dy),int(Rover.pos[0]+dx),2] == 0):
                            print('Gold spotted at dx {} dy {}'.format(dx,dy))
                            if Rover.worldmap[int(Rover.pos[1]+dy),int(Rover.pos[0]+dx),2] < 128:
                                print('gold at dx {} dy {} is not navigable'.format(dx,dy))
                            else:
                                dx_list.append(dx)
                                dy_list.append(dy)
            dx = np.mean(dx_list)
            dy = np.mean(dy_list)
            if abs(dx) > 0 or abs(dy) > 0:
                print('gold mean dx {} dy {} pos[0] {} pos[1] {}'.format(dx,dy,Rover.pos[0],Rover.pos[1]))
                Rover.mission_pos = [Rover.pos[0]+dx, Rover.pos[1]+dy]
                Rover.change_mode('mission')
            
            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.stop_forward:  
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    if Rover.progress == False:
                        print('Throttle {} Velocity {} no progress condition'.format(Rover.throttle,Rover.vel))
                        Rover.sub_call('escape')
                    else:
                        # Set throttle value to throttle setting
                        Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                mean_angle = nav_angle_deg(Rover)
                Rover.steer = np.clip(mean_angle, -15, 15)
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.change_mode('stop')

        elif Rover.mode == 'mission':            
            Rover.prior_mode = []
            # turn in the direction of target location
            y = Rover.mission_pos[1] - Rover.pos[1]
            x = Rover.mission_pos[0] - Rover.pos[0]
            if abs(y) <= 1.0 and abs(x) <= 1.0:
                # arrival at target zone
                print('ARRIVAL at mission target zone!  Rover near sample {}'.format(Rover.near_sample))
                # survey the area and mark the searchmap                
                for dx in range(-2,3):
                    for dy in range(-2,3):
                        if Rover.pos[1]+dy >= 0 and Rover.pos[1]+dy < Rover.worldmap.shape[0] and \
                           Rover.pos[0]+dx >= 0 and Rover.pos[0]+dx < Rover.worldmap.shape[1]:
                            Rover.searchmap[int(Rover.pos[1]+dy),int(Rover.pos[0]+dx),2] = 1  # searched
                # check for navigability in wider range and mark obstacle squares as searched
                for dx in range(-5,6):
                    for dy in range(-5,6):
                        if Rover.pos[1]+dy >= 0 and Rover.pos[1]+dy < Rover.worldmap.shape[0] and \
                           Rover.pos[0]+dx >= 0 and Rover.pos[0]+dx < Rover.worldmap.shape[1]:
                            if Rover.worldmap[int(Rover.pos[1]+dy),int(Rover.pos[0]+dx),2] < 128 or \
                               Rover.worldmap[int(Rover.pos[1]+dy),int(Rover.pos[0]+dx),0] > 96:
                                Rover.searchmap[int(Rover.pos[1]+dy),int(Rover.pos[0]+dx),2] = 1                
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
                Rover.mission_pos = None
                # try to pick up the rock
                Rover.change_mode('collect')
            else:
                theta = np.arctan2([y],[x])[0]
                steer_rad = normalize(theta - (Rover.yaw * np.pi / 180.0))
                print('Mission pos {} yaw {} theta {} target steer_rad {}'.format \
                      (Rover.mission_pos,Rover.yaw,theta,steer_rad))
                if abs(steer_rad * 180.0/np.pi) > 20.0:
                    Rover.target_rad = theta
                    Rover.sub_call('rotate')
                else:
                    # Use obstacle information explicitly instead of simple nav_angles count
                    dx = np.cos(theta)
                    dy = np.sin(theta)
                    fwd_x = int(Rover.pos[0] + dx)
                    fwd_y = int(Rover.pos[1] + dy)
                    if fwd_y >= 0 and fwd_y < Rover.worldmap.shape[0] and \
                       fwd_x >= 0 and fwd_x < Rover.worldmap.shape[1] and \
                       Rover.worldmap[fwd_y,fwd_x,2] >= 128 and \
                       Rover.progress == True:
                        if Rover.vel < Rover.max_vel:
                            Rover.throttle = Rover.throttle_set
                        else:
                            Rover.throttle = 0
                        Rover.brake = 0

                        # FIXME: use navigation
                        
                        # find the closest unobstructed angle                    
                        #delta_rad = Rover.nav_angles - steer_rad
                        #best_rad = min(delta_rad,key=abs)
                        #best_rad += steer_rad
                        
                        best_rad = steer_rad
                        print('Target steer_rad {} best rad {}'.format(steer_rad,best_rad))

                        Rover.steer = np.clip((180.0 / np.pi) * best_rad, -15, 15)                        
                    else:
                        Rover.sub_call('escape')

        elif Rover.mode == 'rotate':
            Rover.throttle = 0
            print('Rotate yaw {} target yaw {} pulse_on {}'. \
                  format(Rover.yaw,(180.0/np.pi) * Rover.target_rad,Rover.pulse_on))
            if Rover.pulse_on == 1:
                steer_rad = Rover.target_rad - (Rover.yaw * np.pi / 180.0)
                if steer_rad < -np.pi:
                    steer_rad += (2.0 * np.pi)
                elif steer_rad > np.pi:
                    steer_rad -= (2.0 * np.pi)
                if abs(steer_rad * 180.0/np.pi) <= 15:
                    Rover.sub_return()
                else:
                    Rover.brake = 0
                    Rover.steer = 15 if steer_rad >= 0 else -15
            else:
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            #Rover.pulse_on += 1
            #Rover.pulse_on %= 2
            Rover.pulse_on = 1
            delta_yaw = abs(Rover.yaw - Rover.yaw_q)  # delta yaw since change of mode
            if delta_yaw < 15:
                Rover.stuck_count += 1
                if Rover.stuck_count > 50:
                    Rover.sub_call('escape')
            else:
                Rover.yaw_q = Rover.yaw
                Rover.stuck_count = 0
            
        elif Rover.mode == 'move_xy':
            if Rover.worldmap[int(Rover.xy_pos[1]),int(Rover.xy_pos[0]),2] < 128:
                print('WARNING: move_xy command to a non-nav position {}'.format(Rover.xy_pos))
            x = Rover.xy_pos[0] - Rover.pos[0]
            y = Rover.xy_pos[1] - Rover.pos[1]
            print('move_xy xy_pos {} x {} y {}'.format(Rover.xy_pos,x,y))
            if np.sqrt(x**2 + y**2) < 0.25:
                Rover.sub_return()
            else:
                # rotate to face in worldmap direction of x,y or xy
                theta = np.arctan2([y],[x])[0]
                steer_rad = normalize(theta - (Rover.yaw * np.pi / 180.0))
                print('move_xy theta {} steer_rad {}'.format(theta,steer_rad))
                steer_deg = steer_rad * 180.0/np.pi
                if abs(steer_deg) > 20.0:
                    Rover.target_rad = theta
                    Rover.sub_call('rotate')
                else:
                    Rover.throttle = Rover.throttle_set
                    Rover.brake = 0
                    Rover.steer = steer_deg
                
        elif Rover.mode == 'escape':
            # turn to try to find somewhere to go
            Rover.throttle = 0
            if Rover.vel > 0.2:
                Rover.brake = Rover.brake_set
            else:
                # target angle adjustment to then return to forward and try again
                delta_yaw = abs(Rover.yaw - Rover.yaw_q)  # delta yaw since change of mode
                if delta_yaw <= 15 or len(Rover.nav_angles) < Rover.go_forward:
                    Rover.brake = 0
                    Rover.steer = 15
                    # autoreverse if current direction isn't working
                    s = int(Rover.stuck_count / 50)
                    if (s % 2) == 1:
                        print('Reverse escape steering direction (stuck count {} s {})'.format(Rover.stuck_count,s))
                        Rover.steer = -15 if Rover.steer > 0 else 15
                    Rover.stuck_count += 1
                else:
                    Rover.brake = 0
                    Rover.throttle = Rover.throttle_set
                    Rover.sub_return()

        elif Rover.mode == 'collect':
            # Attempt to collect sample
            # Rover should be within about 1m of sample on each axis
            Rover.prior_mode = []
            print('Collect attempt: pos {} hires_gold_pos {} vel {}'.format(Rover.pos,Rover.hires_gold_pos,Rover.vel))
            if Rover.vel > 0.1:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            else:
                if Rover.hires_gold_pos == None:
                    # bump the position of rover a little and re-assess
                    if Rover.searchmap[int(Rover.pos[1]),int(Rover.pos[0]),0] == 0:
                        print('Need to move rover near gold')
                        hit_pos = None
                        for dx in range(-1,2):
                            for dy in range(-1,2):
                                if Rover.searchmap[int(Rover.pos[1]+dy),int(Rover.pos[0]+dx),0] >= 1:
                                    if hit_pos == None:
                                        hit_pos = [int(Rover.pos[0]+dx),int(Rover.pos[1]+dy)]
                        if hit_pos == None:
                            # abort collection:  we should be within 1m of gold
                            print('Abort collection due to no gold in range')
                            Rover.change_mode('forward')  # mission failure
                        else:
                            Rover.xy_pos = hit_pos
                            Rover.sub_call('move_xy')
                    else:
                        # turn until spotting gold
                        Rover.throttle = 0
                        Rover.brake = 0
                        Rover.steer = -15
                        Rover.stuck_count += 1
                        if Rover.stuck_count > 100:
                            print('Abort due to no gold spotted')
                            Rover.change_mode('forward')  # mission failure
                else:
                    Rover.throttle = 0
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    if Rover.near_sample == True:
                        print('Near the sample, pick it up!')
                        # mark this region searched so we don't try to find gold here again
                        for dx in range(-5,6):
                            for dy in range(-5,6):
                                if Rover.pos[1]+dy >= 0 and Rover.pos[1]+dy < Rover.worldmap.shape[0] and \
                                   Rover.pos[0]+dx >= 0 and Rover.pos[0]+dx < Rover.worldmap.shape[1]:
                                    Rover.searchmap[int(Rover.pos[1]+dy),int(Rover.pos[0]+dx),2] = 1
                        Rover.send_pickup = True
                        Rover.change_mode('forward')
                    else:
                        print('Use the hires gold location')
                        dx = Rover.hires_gold_pos[0] / 100.0  # 100 pix/m
                        dy = Rover.hires_gold_pos[1] / 100.0
                        Rover.xy_pos = [Rover.pos[0]+(dx/2.0),Rover.pos[1]+(dy/2.0)]
                        Rover.sub_call('move_xy')
            
        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = 15 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    Rover.change_mode('forward')
    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
        
    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
    
    return Rover

