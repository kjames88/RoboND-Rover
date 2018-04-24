import numpy as np
import time

def nav_angle_deg(Rover):
    # Find right-biased heading direction
    if len(Rover.nav_angles) == 0:
        return 0    
    angles = Rover.nav_angles * 180.0/np.pi
    mean_angle = np.mean(angles)

    right_angles = np.select([angles<mean_angle],[angles])
    right_mean = np.mean(right_angles)
    return right_mean

def max_gold_pos(Rover,check_searched):
    # sweep the nearby area for gold
    max_gold = 0
    pos = None
    for dx in range(-10,10):
        for dy in range(-10,10):
            new_y = int(Rover.pos[1] + dy)
            new_x = int(Rover.pos[0] + dx)
            if new_y >= 0 and new_y < 200 and new_x >= 0 and new_x < 200:
                v = Rover.worldmap[new_y,new_x,1]
                if v > max_gold:
                    if check_searched == False or Rover.searchmap[new_y,new_x,2] == 0:
                        max_gold = v
                        if max_gold >= Rover.gold_thresh:
                            pos = [dx,dy]
    if pos is not None:
        return [int(Rover.pos[0]+pos[0]),int(Rover.pos[1]+pos[1])]
    else:
        return None

def any_gold_nearby(Rover):
    for dx in range(-10,10):
        for dy in range(-10,10):
            new_y = int(Rover.pos[1] + dy)
            new_x = int(Rover.pos[0] + dx)
            if new_y >= 0 and new_y < 200 and new_x >= 0 and new_x < 200:
                v = Rover.worldmap[new_y,new_x,1]
                if v > 0:
                    return True
    return False
    
def navigable_neighbor(Rover):
    for dx in range(-1,2):
        for dy in range(-1,2):
            new_y = int(Rover.pos[1] + dy)
            new_x = int(Rover.pos[0] + dx)
            if new_y >= 0 and new_y < 200 and new_x >= 0 and new_x < 200:
                if Rover.worldmap[new_y,new_x,2] >= Rover.nav_thresh and \
                   Rover.worldmap[new_y,new_x,0] < (Rover.obstacle_ratio * Rover.worldmap[new_y,new_x,2]):
                    return [new_x,new_y]
    return None
    
# [x,y]
delta = [[-1,0],[0,-1],[1,0],[0,1],[-1,-1],[-1,1],[1,-1],[1,1]]

def get_routing_dir(deg):
    if deg <= 22.5:
        return 2  # right [1,0]
    elif deg <= 67.5:
        return 7  # up right [1,1]
    elif deg <= 112.5:
        return 3  # up [0,1]
    elif deg <= 157.5:
        return 5  # up left [-1,1]
    elif deg <= 202.5:
        return 0  # left [-1,0]
    elif deg <= 247.5:
        return 4  # down left [-1,-1]
    elif deg <= 292.5:
        return 1  # down [0,-1]
    elif deg <= 337.5:
        return 6  # down right [1,-1]
    else:
        return 2  # right [1,0]

# Simple pre-A* router based on the Udacity AI for Robotics lectures
def route(Rover,start_pos,end_pos):
    # use the worldmap to find a path
    print('Route from {} to {}'.format(start_pos,end_pos))
    if start_pos == end_pos:
        return 201,201  # don't route from grid cell to itself (0m travel)
    queue = []
    closed = np.zeros_like(Rover.worldmap[:,:,0],dtype=np.int32)
    rx_action = np.zeros_like(Rover.worldmap[:,:,0],dtype=np.int32)
    queue.append([0.0,start_pos[0],start_pos[1]]) # [cost, x, y]
    goal_reached = False
    while len(queue) > 0:
        queue.sort(key=lambda v: v[0])
        v = queue.pop(0)
        c = v[0]
        x = int(v[1])
        y = int(v[2])
        print('expand x={} y={} cost={}'.format(x,y,c))
        if end_pos[0] == x and end_pos[1] == y:
            # found path
            goal_reached = True
            break
        for i in range(len(delta)):
            new_x = x + delta[i][0]
            new_y = y + delta[i][1]
            if new_x >= 0 and new_x < 200 and new_y >= 0 and new_y < 200:
                print('cell x={} y={} closed {} obstacle {} nav {}'.format(new_x,new_y,closed[new_y,new_x], \
                                                                           Rover.worldmap[new_y,new_x,0], \
                                                                           Rover.worldmap[new_y,new_x,2]))
                nav = Rover.worldmap[new_y,new_x,2]
                obst = Rover.worldmap[new_y,new_x,0]
                if closed[new_y,new_x] == 0 and nav >= Rover.nav_thresh: #and obst < (Rover.obstacle_ratio * nav):
                    if Rover.navigation_dir_blocked[y,x,i] == 0:
                        cdir = obst/nav # nav > 0 enforced above
                        new_cost = c + cdir
                        queue.append([new_cost,new_x,new_y])
                        closed[new_y,new_x] = 1
                        rx_action[new_y,new_x] = i
    if goal_reached == False:
        return 200,200
    # follow the action back to start
    x = end_pos[0]
    y = end_pos[1]
    while True:
        action = rx_action[y,x]
        new_x = x - delta[action][0]
        new_y = y - delta[action][1]
        if new_x == start_pos[0] and new_y == start_pos[1]:
            return x,y  # this is the first step position
        x = new_x
        y = new_y
    
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
            Rover.transition = 'None'  # any operation on transition should be placed ahead of this
            Rover.prior_mode = []
            Rover.forward_yaw = Rover.yaw
            # Check for gold spotted in the vicinity and potentially go search it out
            dx_list = []
            dy_list = []
            gold_pos = max_gold_pos(Rover,True)
            if gold_pos is not None:
                print('Gold pos {}'.format(gold_pos))
                Rover.mission_pos = gold_pos
                Rover.change_mode('mission')
            #elif any_gold_nearby(Rover):
            #    # try to boost the gold spotted so it triggers the mission above next time
            #    Rover.brake = Rover.brake_set
            #    Rover.throttle = 0
            #    Rover.steer = 0
            #    Rover.sub_call('survey')
            else:
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

        elif Rover.mode == 'resume_forward':
            # jump starts the 2-step operation: rotate
            # sub_return triggers the 2nd step: return to forward
            if Rover.transition == 'jump':
                # rotate back to forward yaw then continue exploring
                Rover.target_rad = Rover.forward_yaw * np.pi/180.0
                Rover.sub_call('rotate')
            else:
                Rover.change_mode('forward')            
            
        elif Rover.mode == 'survey':
            Rover.transition = 'None'  # any operation on transition should be placed ahead of this
            Rover.throttle = 0
            if Rover.vel > 0.2:
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            else:
                Rover.brake = 0
                Rover.steer = 15
            Rover.stuck_count += 1
            if Rover.stuck_count > 50:
                Rover.sub_return()
            
        elif Rover.mode == 'mission':
            abort_mission = False
            if Rover.transition == 'sub_return':
                if Rover.xy_result == 'abort':
                    abort_mission = True
            Rover.transition = 'None'  # any operation on transition should be placed ahead of this
            Rover.prior_mode = []
            Rover.xy_result = 'None'

            # turn in the direction of target location
            y = Rover.mission_pos[1] - Rover.pos[1]
            x = Rover.mission_pos[0] - Rover.pos[0]
            theta = np.arctan2([y],[x])[0]
            if Rover.hires_gold_pos is not None or (abs(y) <= 0.25 and abs(x) <= 0.25):
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
                            if Rover.worldmap[int(Rover.pos[1]+dy),int(Rover.pos[0]+dx),2] < 64 or \
                               Rover.worldmap[int(Rover.pos[1]+dy),int(Rover.pos[0]+dx),0] > 64:
                                Rover.searchmap[int(Rover.pos[1]+dy),int(Rover.pos[0]+dx),2] = 1                
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
                Rover.mission_pos = None
                # try to pick up the rock
                Rover.change_mode('collect')
            elif abort_mission == True:
                # already tried move_xy:  throw it in gear to move the rover and then abort mission
                Rover.throttle = -0.2
                Rover.brake = 0
                Rover.steer = -15
                print('Abort mission due to move_xy failure')
                Rover.change_mode('resume_forward')  # abort the mission
            else:
                steer_rad = normalize(theta - (Rover.yaw * np.pi / 180.0))
                print('Mission pos {} yaw {} theta {} target steer_rad {}'.format \
                      (Rover.mission_pos,Rover.yaw,theta,steer_rad))
                if abs(steer_rad * 180.0/np.pi) > 30.0:
                    Rover.target_rad = theta
                    Rover.sub_call('rotate')
                else:
                    # Use obstacle information explicitly instead of simple nav_angles count
                    if Rover.progress == True:
                        if np.sqrt(x**2 + y**2) < 2.0:  # approaching target: slow down
                            if Rover.vel > 0.75:
                                Rover.brake = Rover.brake_set
                                Rover.throttle = 0
                            else:
                                Rover.brake = 0
                                Rover.throttle = Rover.throttle_set/2
                        else:
                            if Rover.vel < Rover.max_vel:
                                Rover.throttle = Rover.throttle_set
                            else:
                                Rover.throttle = 0
                            Rover.brake = 0
                        print('Target steer_rad {}'.format(steer_rad))
                        Rover.steer = np.clip((180.0 / np.pi) * steer_rad, -15, 15)                        
                    else:
                        # route around obstacle
                        Rover.xy_pos = Rover.mission_pos
                        Rover.sub_call('move_xy')

        elif Rover.mode == 'rotate':
            Rover.transition = 'None'  # any operation on transition should be placed ahead of this
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
                if Rover.stuck_count > 100:
                    Rover.sub_call('escape')
            else:
                Rover.yaw_q = Rover.yaw
                Rover.stuck_count = 0
            
        elif Rover.mode == 'move_xy':
            if Rover.transition == 'sub_call':
                Rover.escape_attempts = 0
            Rover.transition = 'None'  # any operation on transition should be placed ahead of this
            start_pos = [int(Rover.pos[0]),int(Rover.pos[1])]
            end_pos = [int(Rover.xy_pos[0]),int(Rover.xy_pos[1])]
            next_x, next_y = route(Rover,start_pos,end_pos)
            print('ROUTE next x={} y={}'.format(next_x,next_y))
            if next_x < 200 and next_y < 200:
                step_pos = [int(next_x),int(next_y)]
            else:
                #step_pos = navigable_neighbor(Rover)  # move from current position to retry routing
                step_pos = None

            if step_pos is not None:
                if Rover.target_pos_q is not None:
                    if Rover.target_pos_q[0] == int(Rover.pos[0]) and Rover.target_pos_q[0] == int(Rover.pos[1]):
                        Router.escape_attempts = 0
                Rover.target_pos_q = step_pos
                x = step_pos[0] - int(Rover.pos[0])
                y = step_pos[1] - int(Rover.pos[1])
                rem_dist = np.sqrt((Rover.xy_pos[0]-Rover.pos[0])**2 + (Rover.xy_pos[1]-Rover.pos[1])**2)
                print('move_xy step_pos {} x {} y {} rem_dist {}'.format(step_pos,x,y,rem_dist))
                # xy operates on grid cells so check integer position for completion
                if int(Rover.xy_pos[0]) == int(Rover.pos[0]) and int(Rover.xy_pos[1]) == int(Rover.pos[1]):
                    Rover.xy_result = 'target_loc'
                    Rover.sub_return()
                else:
                    # rotate to face in worldmap direction of x,y or xy
                    theta = np.arctan2([y],[x])[0]
                    steer_rad = normalize(theta - (Rover.yaw * np.pi / 180.0))
                    print('move_xy theta {} steer_rad {}'.format(theta,steer_rad))
                    steer_deg = steer_rad * 180.0/np.pi
                    if abs(steer_deg) > 30.0:
                        Rover.target_rad = theta
                        Rover.sub_call('rotate')
                    else:
                        if Rover.progress == True:
                            if rem_dist >= 2.0:
                                if Rover.vel < Rover.max_vel:
                                    Rover.throttle = Rover.throttle_set
                                    Rover.brake = 0
                                else:
                                    Rover.throttle = 0
                                    Rover.brake = 0
                            else:
                                if Rover.vel > 0.75:
                                    Rover.brake = Rover.brake_set
                                    Rover.throttle = 0
                                else:
                                    Rover.throttle = Rover.throttle_set/2.0
                                    Rover.brake = 0
                                    Rover.steer = steer_deg
                        else:
                            if Rover.escape_attempts > 1:
                                d = get_routing_dir(theta * 180.0/np.pi)  # 0-2pi range
                                print('block routing dir {} from pos {}'.format(d,Rover.pos))
                                Rover.navigation_dir_blocked[int(Rover.pos[1]),int(Rover.pos[0]),d] = 1
                                Rover.xy_result = 'abort'
                                Rover.sub_return()  # failed
                            else:
                                Rover.escape_attempts += 1
                                Rover.sub_call('escape')
                    Rover.stuck_count += 1
            else:
                if start_pos == end_pos:
                    Rover.xy_result = 'target_loc'
                else:
                    Rover.xy_result = 'abort'
                Rover.sub_return()  # failure
                
        elif Rover.mode == 'escape':
            Rover.transition = 'None'  # any operation on transition should be placed ahead of this
            # turn to try to find somewhere to go
            Rover.stuck_count += 1
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
            else:
                if Rover.stuck_count < 25:
                    Rover.throttle = -0.2
                    Rover.steer = -15
                    Rover.brake = 0
                else:
                    # target angle adjustment to then return to forward and try again
                    delta_yaw = abs(Rover.yaw - Rover.yaw_q)  # delta yaw since change of mode
                    if delta_yaw <= 15 or len(Rover.nav_angles) < Rover.go_forward:
                        Rover.brake = 0
                        Rover.throttle = 0
                        Rover.steer = 15
                        # autoreverse if current direction isn't working
                        s = int(Rover.stuck_count / 100)
                        if (s % 2) == 1:
                            print('Reverse escape steering direction (stuck count {} s {})'.format(Rover.stuck_count,s))
                            Rover.steer = -15 if Rover.steer > 0 else 15
                    else:
                        Rover.brake = 0
                        Rover.throttle = Rover.throttle_set
                        Rover.sub_return()

        elif Rover.mode == 'collect':
            # Attempt to collect sample
            # Rover should be within about 1m of sample on each axis
            Rover.transition = 'None'  # any operation on transition should be placed ahead of this
            Rover.prior_mode = []
            print('Collect attempt: pos {} hires_gold_pos {} vel {}'.format(Rover.pos,Rover.hires_gold_pos,Rover.vel))
            if Rover.vel > 0.1:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            else:
                if Rover.hires_gold_pos == None:
                    # bump the position of rover a little and re-assess
                    gold_pos = max_gold_pos(Rover,False)
                    if gold_pos is not None:
                        if np.sqrt((Rover.pos[0]-gold_pos[0])**2 + (Rover.pos[1]-gold_pos[1])**2) >= 1.0 and \
                           Rover.xy_pos != gold_pos:
                            print('Need to move rover near gold pos {}'.format(gold_pos))
                            Rover.xy_pos = gold_pos
                            Rover.sub_call('move_xy')
                        else:
                            # turn until spotting gold
                            Rover.stuck_count += 1
                            s = int(Rover.stuck_count / 100)
                            if (s % 2) == 1:
                                Rover.throttle = -0.1
                                Rover.brake = 0
                                Rover.steer = 0
                            else:
                                Rover.throttle = 0
                                Rover.brake = 0
                                Rover.steer = -15
                                if Rover.stuck_count > 1000:
                                    print('Abort due to no gold spotted')
                                    Rover.change_mode('resume_forward')  # mission failure
                    else:
                        print('Abort collection due to no gold in range')
                        Rover.change_mode('resume_forward')    
                else:
                    if Rover.near_sample == True:
                        Rover.throttle = 0
                        Rover.brake = Rover.brake_set
                        Rover.steer = 0
                        print('Near the sample, pick it up!')
                        # mark this region searched so we don't try to find gold here again
                        for dx in range(-5,6):
                            for dy in range(-5,6):
                                if Rover.pos[1]+dy >= 0 and Rover.pos[1]+dy < Rover.worldmap.shape[0] and \
                                   Rover.pos[0]+dx >= 0 and Rover.pos[0]+dx < Rover.worldmap.shape[1]:
                                    Rover.searchmap[int(Rover.pos[1]+dy),int(Rover.pos[0]+dx),2] = 1
                        Rover.send_pickup = True
                        Rover.change_mode('resume_forward')
                    else:
                        Rover.stuck_count += 1
                        if Rover.stuck_count > 100:
                            Rover.sub_call('escape')
                        print('Use the hires gold location pos {} rover polar {}'. \
                              format(Rover.hires_gold_pos,Rover.hires_gold_polar))
                        Rover.xy_pos = Rover.hires_gold_pos
                        # move_xy only works when grid cells do not match; also don't use for < 1m distance
                        if int(Rover.xy_pos[0]) != int(Rover.pos[0]) and int(Rover.xy_pos[1]) != int(Rover.pos[1]) \
                           and np.sqrt((Rover.xy_pos[0]-Rover.pos[0])**2 + (Rover.xy_pos[1]-Rover.pos[1])**2) >= 1.0:
                            Rover.brake = Rover.brake_set
                            Rover.throttle = 0
                            Rover.steer = 0
                            Rover.sub_call('move_xy')
                        else:
                            # nudge toward goal
                            Rover.throttle = 0.1
                            Rover.brake = 0
                            Rover.steer = np.clip(Rover.hires_gold_polar[0],-15,15)
            
        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            Rover.transition = 'None'  # any operation on transition should be placed ahead of this
            Rover.prior_mode = []
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
        Rover.transition = 'None'  # any operation on transition should be placed ahead of this
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
        
    # If in a state where want to pickup a rock send pickup command
    #if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
    #    Rover.send_pickup = True
    
    return Rover

