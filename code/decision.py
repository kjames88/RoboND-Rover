import numpy as np
import time


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
            # Check for gold spotted in the vicinity and potentially go search it out
            dx_list = []
            dy_list = []
            for dx in range(-5,5):
                for dy in range(-5,5):
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
                        Rover.change_mode('escape')
                    else:
                        # Set throttle value to throttle setting
                        Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Find an angle directed toward the obstacle on the right side
                angles = Rover.nav_angles * 180.0/np.pi
                mean_angle = np.mean(angles)
                right_angles = np.select([angles<mean_angle],[angles])
                mean_angle = np.mean(right_angles)
                print("mean angle {} steer {}".format(np.mean(angles),mean_angle))
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
            # turn in the direction of target location
            y = Rover.mission_pos[1] - Rover.pos[1]
            x = Rover.mission_pos[0] - Rover.pos[0]
            if abs(y) <= 2 and abs(x) <= 2:
                # arrival at target zone
                print('Arrival at mission target zone!')
                # survey the area and mark the searchmap
                
                for dx in range(-2,3):
                    for dy in range(-2,3):
                        Rover.searchmap[int(Rover.pos[1]+dy),int(Rover.pos[0]+dx),2] = 1  # searched
                # check for navigability in wider range and mark obstacle squares as searched
                for dx in range(-5,6):
                    for dy in range(-5,6):
                        if Rover.worldmap[int(Rover.pos[1]+dy),int(Rover.pos[0]+dx),2] < 128 or \
                           Rover.worldmap[int(Rover.pos[1]+dy),int(Rover.pos[0]+dx),0] > 96:
                            Rover.searchmap[int(Rover.pos[1]+dy),int(Rover.pos[0]+dx),2] = 1
                
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
                Rover.mission_pos = None
                Rover.change_mode('forward')
            else:
                theta = np.arctan2([y],[x])[0]
                steer_rad = theta - (Rover.yaw * np.pi / 180.0)
                if steer_rad > np.pi:
                    steer_rad -= (2*np.pi)
                elif steer_rad < -np.pi:
                    steer_rad += (2*np.pi)
                print('Mission pos {} yaw {} theta {} target steer_rad {}'.format \
                      (Rover.mission_pos,Rover.yaw,theta,steer_rad))
                if abs(steer_rad * 180.0/np.pi) > 30.0:
                    Rover.target_rad = theta
                    Rover.change_mode('rotate')
                else:
                    if len(Rover.nav_angles) < Rover.stop_forward:
                        if Rover.vel > 0.2:
                            Rover.throttle = 0
                            Rover.brake = Rover.brake_set
                            Rover.steer = 0
                        else:
                            Rover.throttle = 0
                            Rover.brake = 0
                            Rover.steer = 15 if steer_rad >= 0 else -15
                            
                            # FIXME this can oscillate and become stuck
                            
                    else:
                        if Rover.vel < Rover.max_vel:
                            if Rover.progress == False:
                                print('Throttle {} Velocity {} no progress condition'.format(Rover.throttle,Rover.vel))
                                Rover.change_mode('escape')
                            else:
                                Rover.throttle = Rover.throttle_set
                        else:
                            Rover.throttle = 0
                            Rover.brake = 0
                            # find the closest unobstructed angle                    
                            delta_rad = Rover.nav_angles - steer_rad
                            best_rad = min(delta_rad,key=abs)
                            best_rad += steer_rad
                            print('Target steer_rad {} best rad {}'.format(steer_rad,best_rad))

                            Rover.steer = np.clip((180.0 / np.pi) * best_rad, -15, 15)

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
                    Rover.change_mode('mission')
                else:
                    Rover.brake = 0
                    Rover.steer = 15 if steer_rad >= 0 else -15
            else:
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            #Rover.pulse_on += 1
            #Rover.pulse_on %= 2
            Rover.pulse_on = 1
            
                    
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
                    angle_sign = 1  # follow cliff wall default
                    if Rover.mission_pos is not None:
                        y = Rover.mission_pos[1] - Rover.pos[1]
                        x = Rover.mission_pos[0] - Rover.pos[0]
                        theta = np.arctan2([y],[x])[0]
                        angle = theta - Rover.yaw
                        angle_sign = 1 if angle >= 0 else -1                        
                    Rover.steer = 15 if angle_sign == 1 else -15
                    # autoreverse if current direction isn't working
                    if Rover.stuck_count > 100:
                        print('Reverse escape steering direction')
                        Rover.steer = -15 if Rover.steer > 0 else 15
                        Rover.stuck_count = 0
                    Rover.stuck_count += 1
                else:
                    Rover.brake = 0
                    Rover.throttle = Rover.throttle_set                    
                    Rover.change_mode('forward' if Rover.mission_pos == None else 'mission')
            
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

