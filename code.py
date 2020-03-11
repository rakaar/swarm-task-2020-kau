import sys
from api import *
from time import sleep
import numpy as np
from threading import Thread
import multiprocessing 
import os
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
    
    
def level3():
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
    Problem: Both getting same sqs when near
    Solution: using multiprocessing.Queue to communication
    bot1 sends the found sq to bot0, bot0 checks if its req_key matches with key bot1 sent, if matches then it calls manage_bot0 again
    
    ISSUE: why is the same bot going to the same target twice in the end
    As per code in controller.py 
    2 different programmes are running on different threads python3 code.py 0, python3 code.py 1
    hence currently 4 different process are running bot0 and bot1 in thread1, and bot0 and bot1 in thread2
    hence but i want parallelism not concurrency

    '''
    if __name__ == '__main__':
        print('MAIN pID ', os.getpid())
        obstaclePose = get_obstacles_list()
        for i in range(200):
            grid.append([])
            for j in range(200):
                grid[i].append(Node(1,(i,j)))
        for pt in obstaclePose:
            for i in range(pt[0][0],pt[2][0]+1):
                for j in range(pt[0][1],pt[2][1]+1):
                    grid[i][j]=Node(0,(i,j))
        

        def nearest_sq(botsPose, bot_num, green_sqs, que):
            try:
                print('bot_num is ', bot_num)
                unvisisted_green_sqs = [sq for sq in green_sqs if sq['visited'] is False]
                
                print('num of sqs left ', len(unvisisted_green_sqs))
                if len(unvisisted_green_sqs) is 0:
                    print('status of mission is ', is_mission_complete()) # Has  to be true
                    # what happens if in other process a bot is still moving
                    # stop execution of current thread for sometime so as to let the other bot reach its target
                    sleep(5)
                    sys.exit()
                    # Ideally it should close because the mission is complete
                    
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
                print('index is ', req_key, 'for bot num ', bot_num, 'PROCESS IS ', os.getpid())
                
                # get the target square before that mark its visited as True
                target_sq_dict = greens[req_key]
                target_sq_dict['visited'] = True
                greens[req_key] = target_sq_dict
                print('greens is ',greens)

                target_sq = target_sq_dict[req_key] 
                
                target_obj = grid[target_sq[0][0]][target_sq[0][1]]
                
                if bot_num is 1:
                    que.put(req_key)    
                
                return target_obj, req_key
            except Exception as e:
                print('e is nearest_sq',e)
                

        def manage_bot0(green_sqs,que):
            while is_mission_complete() is False:
                bot0_pose = get_botPose_list()[0]
                current_pos0_obj = grid[bot0_pose[0]][bot0_pose[1]]
                target_obj, req_key = nearest_sq(bot0_pose, 0, green_sqs,que)
                if req_key is que.get():
                    print('BOTH are at same', req_key)
                    manage_bot0(green_sqs, que)
                else:
                    print('nearest 0',target_obj)
                    path = aStar(current_pos0_obj, target_obj)
                    for i in range(1,len(path)):
                        send_command(0, path[i].move)
        
        
        def manage_bot1(green_sqs,que):
            while is_mission_complete() is False:    
                bot1_pose = get_botPose_list()[1]
                current_pos1_obj = grid[bot1_pose[0]][bot1_pose[1]]
                target_obj, _ = nearest_sq(bot1_pose, 1, green_sqs,que)
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
            que = multiprocessing.Queue()

            p1 = multiprocessing.Process(target = manage_bot0, args=(greens,que))
            p2 = multiprocessing.Process(target = manage_bot1, args=(greens,que))
            

            p1.start()
            p2.start()

            print('p1.pid ', p1.pid)
            print('p2.pid ', p2.pid)
            
            p1.join()
            p2.join()

        
    

 
    
def level4():
    ''''
    create different num of proces based on input
    assign each process same func and botId
    start all of them one by one
    end all of them

    Write a func manage bot generalized
    same nearest sq function
    same dicts approach

    join() is needed for daemon process
    though the main program is completed, .join() is called so as to wait for the process to complete

    non daemonic have join() implicity mentioned
    '''
    print('number of bots ',get_numbots())
    print('MAIN pID ', os.getpid())
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
                print('status of mission is ', is_mission_complete()) # Has  to be true
                # what happens if in other process a bot is still moving
                # stop execution of current thread for sometime so as to let the other bot reach its target
                sleep(5)
                sys.exit()
                # Ideally it should close because the mission is complete
                
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
            print('index is ', req_key, 'for bot num ', bot_num, 'PROCESS IS ', os.getpid())
            
            # get the target square before that mark its visited as True
            target_sq_dict = greens[req_key]
            target_sq_dict['visited'] = True
            greens[req_key] = target_sq_dict
            print('greens is ',greens)

            target_sq = target_sq_dict[req_key] 
            
            target_obj = grid[target_sq[0][0]][target_sq[0][1]]
            return target_obj, req_key
        
        except Exception as e:
            print('e is nearest_sq',e)
    
    
    def manage_bot(bot_id, greens):
        while is_mission_complete() is False:
            print('greens is ', greens)  
            bot_pose = get_botPose_list()[bot_id]
            current_pos_obj = grid[bot_pose[0]][bot_pose[1]]
            target_obj, _ = nearest_sq(bot_pose, bot_id, greens)
            print('nearest one for  ', bot_id, ' is ', target_obj)
            path = aStar(current_pos_obj, target_obj)
            for i in range(1,len(path)):
                send_command(bot_id, path[i].move)
    
    
    num_of_bots = get_numbots()
    processes = []
    greens_arr = get_original_greenZone_list()[:]
    greens_dict_arr = []
    for index, sq in enumerate(greens_arr):
        sq_dict = {}
        sq_dict[index] = sq
        sq_dict['visited'] = False
        greens_dict_arr.append(sq_dict)
    print('afer looping ', greens_dict_arr)
    
    with multiprocessing.Manager() as manager:
        greens = manager.list(greens_dict_arr)
        print('greens is ', greens)
        for bot_id in range(0, num_of_bots):
            processes.append(multiprocessing.Process(target=manage_bot, args = (bot_id, greens)))
            processes[bot_id].start()

        for p in processes:
            p.join()



def level5(botId):
    pass

def level6(botId):
    pass


#######    DON'T EDIT ANYTHING BELOW  #######################

if  __name__=="__main__":
    print('sys.sys.argv[1] ', len(sys.argv))
    if len(sys.argv) is 1:
        pass
    else:
        botId = sys.argv[1]
    
    # if sys.argv[0] is  'code.py': # if no argument is passed
    #     pass
    # else:
    #     botId = sys.argv[1]
        
    level = get_level()
    if level == 1:
        level1(botId)
    elif level == 2:
        level2(botId)
    elif level == 3:
        level3()
    elif level == 4:
        level4()
    elif level == 5:
        level5(botId)
    elif level == 6:
        level6(botId)
    else:
        print("Wrong level! Please restart and select correct level")
