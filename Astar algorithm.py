
import numpy
from heapq import *


def heuristic(a, b):    #using eucledian distance - can move in any direction
    return (b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2


def astar(array, start, goal):

    neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]

    close_set = set()
    came_from = {}
    gscore = {start:0}
    fscore = {start:heuristic(start, goal)}
    oheap = []  

    heappush(oheap, (fscore[start], start)) #priority queue - push the starting location before anything
    
    
    while oheap:

        current = heappop(oheap)[1]
        
        #print('\nentered loop', current)

        if current == goal: # if goal is reached append all locations we passed through to the list
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            return data

        close_set.add(current)
        for i, j in neighbors:
            #loop through all the 8 neighboring squares
            neighbor = current[0] + i, current[1] + j     
            
            tentative_g_score = gscore[current] + heuristic(current, neighbor)  #calc total score
            
            if 0 <= neighbor[0] < array.shape[0]:   # array bound x walls
                if 0 <= neighbor[1] < array.shape[1]:   # array bound y walls
                    if array[neighbor[0]][neighbor[1]] == 1: #check if there is an obstacle at this square
                        continue
                else:
                    continue
            else:
                continue
                
            
            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue
                
            #print("herio", gscore.get(neighbor, 0))
            if  tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:
                came_from[neighbor] = current   #the shortest path to go to the neighbor is though the current node
                gscore[neighbor] = tentative_g_score    #the g-score of the neighbor aka the score needed to reach it 
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)    #the total score to still
                heappush(oheap, (fscore[neighbor], neighbor)) 
                #push this node as the current next open node to be examined 
                
            #print('\n loop', oheap)
                
    return False


#########################################################


nmap = numpy.array([
    [0,0,0,0,0],
    [0,0,0,0,0],
    [1,1,1,0,0],
    [0,0,0,0,0],
    [0,0,0,0,0]])
    

#print the path 
print (astar(nmap, (1, 1), (4, 1)))