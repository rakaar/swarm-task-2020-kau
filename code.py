import sys
from api import *
from time import sleep
import numpy as np
from threading import Thread
import multiprocessing 
#######    YOUR CODE FROM HERE #######################
grid =[]

class Node:
    def __init__(self,value,point):
        self.value = value  #0 for blocked,1 for unblocked
        self.point = point
        self.parent = None
        self.move=None
        self.H = 0
        self.G = 0
        
neigh=[[-1,-1],[-1,0],[-1,1],[0,1],[1,1],[1,0],[1,-1],[0,-1]]

def isValid(pt):
    return pt[0]>=0 and pt[1]>=0 and pt[0]<200 and pt[1]<200

def neighbours(point):  #returns valid neighbours
    global grid,neigh
    x,y = point.point
    links=[]
    for i in range(len(neigh)):
        newX=x+neigh[i][0]
        newY=y+neigh[i][1]
        if not isValid((newX,newY)):
            continue
        links.append((i+1,grid[newX][newY]))
    return links
        
def diagonal(point,point2):
    return max(abs(point.point[0] - point2.point[0]),abs(point.point[1]-point2.point[1]))

def aStar(start, goal):
    #The open and closed sets
    openset = set()
    closedset = set()
    #Current point is the starting point
    current = start
    #Add the starting point to the open set
    openset.add(current)
    #While the open set is not empty
    while openset:
        #Find the item in the open set with the lowest G + H score
        current = min(openset, key=lambda o:o.G + o.H)
        #Remove the item from the open set
        openset.remove(current)
        #Add it to the closed set
        closedset.add(current)
        #If it is the item we want, retrace the path and return it
        if current == goal:
            path = []
            while current.parent:
                path.append(current)
                current = current.parent
                if(current.point==start.point):
                    path.append(current)
                    return path[::-1]
        #Loop through the node's children/siblings which are valid and not blocked
        for move,node in neighbours(current):
            #If it is already in the closed set, skip it
            if node in closedset:
                continue
            #if cell is blocked
            if node.value==0:
                continue
            #Otherwise if it is already in the open set
            if node in openset:
                #Check if we beat the G score 
                new_g = current.G + 1 #onl
                if node.G > new_g:
                    #If so, update the node to have a new parent
                    node.G = new_g
                    node.parent = current
                    node.move=move
            else:
                #If it isn't in the open set, calculate the G and H score for the node
                node.G = current.G + 1
                node.H = diagonal(node, goal)
                #Set the parent to our current item
                node.parent = current
                node.move=move
                #Add it to the set
                openset.add(node)
    #Throw an exception if there is no path
    raise ValueError('No Path Found')

'''
PS: You need not write codes for all levels. You must
at least complete the function corresponding to the level
you're attempting at the moment.
'''
    
def level1(botId):
    global grid
    moveType = 5
    botsPose = get_botPose_list()
    obstaclePose = get_obstacles_list()
    greenZone = get_greenZone_list()
    redZone = get_redZone_list()
    originalGreenZone = get_original_greenZone_list()
    for i in range(200):
        grid.append([])
        for j in range(200):
            grid[i].append(Node(1,(i,j)))
    for pt in obstaclePose:
        for i in range(pt[0][0],pt[2][0]+1):
            for j in range(pt[0][1],pt[2][1]+1):
                grid[i][j]=Node(0,(i,j))
    start=grid[botsPose[0][0]][botsPose[0][1]]
    goal=grid[greenZone[0][0][0]][greenZone[0][0][1]]
    path=aStar(start, goal)
    print(len(path))
    print("final pos:",greenZone[0][0])
    pos=get_botPose_list()
    print("initial pos:",pos[0])
    sleep(5)
    for i in range(1,len(path)):
        successful_move, mission_complete = send_command(botId,path[i].move)
        pos=get_botPose_list()
        if successful_move:
            print("YES")
        else:
            print("NO")
        if mission_complete:
            print("MISSION COMPLETE")
        pos=get_botPose_list()
        print(pos[0])
def level2(botId):
    obstaclePose = get_obstacles_list()
    for i in range(200):
        grid.append([])
        for j in range(200):
            grid[i].append(Node(1,(i,j)))
    for pt in obstaclePose:
        for i in range(pt[0][0],pt[2][0]+1):
            for j in range(pt[0][1],pt[2][1]+1):
                grid[i][j]=Node(0,(i,j))
   
    while (len(get_greenZone_list()) != 0):
        print('Initial number of green sqa ', len(get_greenZone_list()))
        botsPose = get_botPose_list()
        current_pos = grid[botsPose[0][0]][botsPose[0][1]]
        print('l2  s1 ',botsPose[0][0])
        print('l2 s2 ', botsPose[0][1])
        print('l2 ',current_pos)
        # min_diagonal_dist = 9999
        # for index, sqaure in enumerate(get_greenZone_list()):
        #     sqaure_v_x = sqaure[0][0]
        #     sqaure_v_y = sqaure[0][1]
        #     diagonal_dist = max(abs(sqaure_v_x - botsPose[0][0]), abs(sqaure_v_y - botsPose[0][1]))
        #     if diagonal_dist < min_diagonal_dist:
        #         req_ind = index
        #         print('req index in green sqs',req_ind)
        #         break
        #     else:
        #         continue
        green_sqs = get_greenZone_list()
        min_diagonal_dist = max(abs(green_sqs[0][0][0]-botsPose[0][0]), abs(green_sqs[0][0][1]-botsPose[0][1]))
        req_ind = 0
        for i in range(1, len(green_sqs)):
            diagonal_dist = max(abs(green_sqs[i][0][0] - botsPose[0][0]), abs(green_sqs[i][0][1] - botsPose[0][1]))
            if diagonal_dist < min_diagonal_dist:
                req_ind = i
            else:
                continue
        
        goal_2 = grid[get_greenZone_list()[req_ind][0][0]][get_greenZone_list()[req_ind][0][1]]
        print('l2 goal1  ',get_greenZone_list()[req_ind][0][0])
        print('l2 goal2 ', get_greenZone_list()[req_ind][0][1])
        print('l2 goal is ',goal_2)
        print('dbg donw')
        

        path = aStar(current_pos, goal_2)
        for i in range(1,len(path)):
            send_command(botId, path[i].move)
            
        print("Squares left ",len(get_greenZone_list()))
        
    
    print('MISSION status ', is_mission_complete())
    
    
def level3(botId):
    '''
    a global list of all green zones to attend
    while mission_complete is false:
        thread1 
            bot0 sees the nearest green sq
            x = pops it out(deletes out from the list)
            reaches x using a*
        thread2
            bot1 sees the nearest in the same global list
            y = pops it out
            reaches y using a*

    Backup plan: If threading fails due to race conditions, asyncio is the next option
    or may be multiprocessing(lets see)
    Threads failing due to concurrency
    lets shift to parallelism

    removing with index had issues with threads

    Each process has its own memory space. Thats why when u remove in a process in other process it stays 

    even after keeping arr in server process
    req index is causing problem
    it causes problem to elements' position after removed index element of array
    shift to array of dicts to keep track of items lets keep a key name visited which is false or true

    In array of dicts, with a visited key value as flag, issue is while one is being in the process and hasnt been marked True, 
    other one also finds out the same thing. Hence both get the same sq
    Prob: Both getting same sqs when near

    UNIDENTIFIED ISSUE: why is the same bot going to the same target twice in the end
    #TODO IMPROVEMENTS
    check the distance from other bot too
    keep that in one bot only who keeps track of other,
    its like one of the guy is taking or giving away responsibility
    REASON: both catch up nearest ones and also keep calculating on the same sq when closer
    '''
    obstaclePose = get_obstacles_list()
    for i in range(200):
        grid.append([])
        for j in range(200):
            grid[i].append(Node(1,(i,j)))
    for pt in obstaclePose:
        for i in range(pt[0][0],pt[2][0]+1):
            for j in range(pt[0][1],pt[2][1]+1):
                grid[i][j]=Node(0,(i,j))
    

    def nearest_sq(botsPose, bot_num, green_sqs):
        try:
            print('bot_num is ', bot_num)
            unvisisted_green_sqs = [sq for sq in green_sqs if sq['visited'] is False]
            print('num of sqs left ', len(unvisisted_green_sqs))
            if len(unvisisted_green_sqs) is 0:
                print('status of mission is ', is_mission_complete())
                return
            first_sq_in_unvisited_dict = unvisisted_green_sqs[0]
            first_sq_unv_key = list(first_sq_in_unvisited_dict.keys())[0]
            first_sq_unv_verts = list(first_sq_in_unvisited_dict.values())[0] 
            min_diagonal_dist = max(abs(botsPose[0] - first_sq_unv_verts[0][0]), abs(botsPose[1] - first_sq_unv_verts[0][1]))
            print('min disat ',min_diagonal_dist, 'botnun ', bot_num)
            req_key = first_sq_unv_key
            
            for index, s in enumerate(unvisisted_green_sqs):
                if index is 0:
                    continue
                key_of_sq= list(s.keys())[0]
                current_sq = list(s.values())[0]
                diagonal_dist = max(abs(current_sq[0][0] - botsPose[0]), abs(current_sq[0][1] - botsPose[1]))
                if diagonal_dist < min_diagonal_dist:
                    req_key = key_of_sq
                else:
                    continue
            print('index is ', req_key, 'for bot num ', bot_num)
            
            # get the target square before that mark its visited as True
            target_sq_dict = greens[req_key]
            target_sq_dict['visited'] = True
            greens[req_key] = target_sq_dict
            print('greens is ',greens)

            target_sq = target_sq_dict[req_key] 
            
            target_obj = grid[target_sq[0][0]][target_sq[0][1]]
            return target_obj
        except Exception as e:
            print('e is nearest_sq',e)
            

    def manage_bot0(green_sqs):
        while is_mission_complete() is False:
            bot0_pose = get_botPose_list()[0]
            current_pos0_obj = grid[bot0_pose[0]][bot0_pose[1]]
            target_obj = nearest_sq(bot0_pose, 0, green_sqs)
            print('nearest 0',target_obj)
            path = aStar(current_pos0_obj, target_obj)
            for i in range(1,len(path)):
                send_command(0, path[i].move)
    
    
    def manage_bot1(green_sqs):
        while is_mission_complete() is False:    
            bot1_pose = get_botPose_list()[1]
            current_pos1_obj = grid[bot1_pose[0]][bot1_pose[1]]
            target_obj = nearest_sq(bot1_pose, 1, green_sqs)
            print('nearest 1 ', target_obj)
            path = aStar(current_pos1_obj, target_obj)
            for i in range(1,len(path)):
                send_command(1, path[i].move)
    
    with multiprocessing.Manager() as manager:
        greens_arr = get_original_greenZone_list()[:]
        greens_dict_arr = []
        for index, sq in enumerate(greens_arr):
            sq_dict = {}
            sq_dict[index] = sq
            sq_dict['visited'] = False
            greens_dict_arr.append(sq_dict)
        
           
        greens = manager.list(greens_dict_arr)
        
        p1 = multiprocessing.Process(target = manage_bot0, args=(greens,))
        p2 = multiprocessing.Process(target = manage_bot1, args=(greens,))

        p1.start()
        p2.start()
        
        p1.join()
        p2.join()

        
    

 
    
def level4(botId):
    pass

def level5(botId):
    pass

def level6(botId):
    pass


#######    DON'T EDIT ANYTHING BELOW  #######################

if  __name__=="__main__":
    botId = int(sys.argv[1])
    level = get_level()
    if level == 1:
        level1(botId)
    elif level == 2:
        level2(botId)
    elif level == 3:
        level3(botId)
    elif level == 4:
        level4(botId)
    elif level == 5:
        level5(botId)
    elif level == 6:
        level6(botId)
    else:
        print("Wrong level! Please restart and select correct level")
