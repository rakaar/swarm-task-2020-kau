import sys
from api import *
from time import sleep
import numpy as np
from threading import Thread
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
    
    
    # botsPose0 = get_botPose_list()[0]
    # botsPose1 = get_botPose_list()[1]

    # current_pos0 = grid[botsPose0[0][0]][botsPose0[0][1]]
    # current_pos1 = grid[botsPose1[0][0]][botsPose1[0][1]]



    # current_pos = grid[botsPose[0][0]][botsPose[0][1]]

        
    green_sqs = get_greenZone_list()[:]

    def nearest_sq(botsPose):
        try:
            min_diagonal_dist = max(abs(green_sqs[0][0][0]-botsPose[0]), abs(green_sqs[0][0][1]-botsPose[1]))
            print('min dia dis ', min_diagonal_dist)
            req_ind = 0
            for i in range(1, len(green_sqs)):
                diagonal_dist = max(abs(green_sqs[i][0][0] - botsPose[0]), abs(green_sqs[i][0][1] - botsPose[1]))
                if diagonal_dist < min_diagonal_dist:
                    req_ind = i
                else:
                    continue
            print('req_inde ', req_ind)
            sleep(2)
            return req_ind
        except Exception as e:
            print('e is ', e)
            

    def manage_bot0():
        bot0_pose = get_botPose_list()[0]
        nearest_sq_index = nearest_sq(bot0_pose)
        try:
            print('near sq index ', nearest_sq_index)
            print('lenht of ', len(green_sqs))
            target_sq = green_sqs.pop(nearest_sq_index)
        except:
            print('ni bot 0')
            

        target_obj = grid[target_sq[0][0]][target_sq[0][1]]
        current_pos0_obj = grid[bot0_pose[0]][bot0_pose[1]]
       
        path = aStar(current_pos0_obj, target_obj)
        for i in range(1,len(path)):
            send_command(0, path[i].move)
    
    
    def manage_bot1():
        bot1_pose = get_botPose_list()[1]
        nearest_sq_index = nearest_sq(bot1_pose)
        try:
            print('near sq index ', nearest_sq_index)
            print('lenht of ', len(green_sqs))
            target_sq = green_sqs.pop(nearest_sq_index)
        except:
            print('ni bot 1')

        target_obj = grid[target_sq[0][0]][target_sq[0][1]]
        current_pos1_obj = grid[bot1_pose[0]][bot1_pose[1]]

        path = aStar(current_pos1_obj, target_obj)
        for i in range(1,len(path)):
            send_command(1, path[i].move)
    

    while is_mission_complete() is False:
        Thread(target=manage_bot0).start()
        Thread(target=manage_bot1).start()
    print('mission done')

    
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
