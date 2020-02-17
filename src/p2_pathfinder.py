from heapq import heappop, heappush
from math import sqrt
import queue
from numpy import arange


def find_path (source_point, destination_point, mesh):

    """
    Searches for a path from source_point to destination_point through the mesh

    Args:
        source_point: starting point of the pathfinder
        destination_point: the ultimate goal the pathfinder must reach
        mesh: pathway constraints the path adheres to

    Returns:

        A path (list of points) from source_point to destination_point if exists
        A list of boxes explored by the algorithm
    """

    path = []
    boxes = []
    
    #boxes = breadth_first_search(source_point,destination_point,mesh)
    path, boxes = bi_a_star(source_point, destination_point, mesh)
    
   # path.append((source_point, destination_point))
    return path,boxes


#BFS   
def breadth_first_search(source_point, destination_point, mesh):
    q = queue.Queue()
    boxes = []
    came_from = {}
    
    found = False
     #start and finish boxes
    src = None
    dst = None
    #find source box and destination box
    src = find_box(source_point,mesh)
    dst = find_box(destination_point,mesh)

    q.put(src)
    came_from[src] = None
    #if the same box
    if src == dst and src != None:
        #maybe redundant
        boxes.append((src, dst))
        return boxes

    while not q.empty():
        current = q.get()
        
        if(current == dst):
            found = True
            break
       # print ("current: ",current)
        for next in mesh['adj'][current]:
            if next not in came_from:
                q.put(next)
                came_from[next] = current
    if not found:
        print ("No path Found")
    else:
        while current != src:
        # print("came_from: ", came_from[current])
            boxes.append(current)
            current = came_from[current]
        boxes.append(src)  
        boxes.reverse() 
        return boxes
    return None


# def a_star(source_point,destination_point,mesh):
#     src = None
#     dst = None
#     #find source box and destination box
#     src = find_box(source_point,mesh)
#     dst = find_box(destination_point,mesh)
#     print("src: ", src)
#     print("dst: ", dst)
#     #return lists
#     path = []
#     boxes = []
#     #if the same box
#     if src == dst and src != None:
#         #maybe redundant
#         path.append((source_point, destination_point))
#         boxes.append((src, dst))
#         return path, boxes
#     #detail points
#     detail_points = {}
#     #priority queue
#     queue = [(0,src)]
    
#     #distance
#     dist = {}
#     dist[src] = 0
#     # The dictionary that will store the backpointers
#     prev = {}
#     prev[src] = None
#     #detail points
#     detail_points[src] = source_point
#     detail_points[dst] = destination_point

#     while queue:
#         current_dist,current_box = heappop(queue)
#         # Check if current box is the destination
#         if current_box == dst:
#             print("Found, Nice!")
#             path.append((detail_points[dst],destination_point))
#             while current_box != src:
#                 boxes.append(current_box)
#                 path.append((detail_points[prev[current_box]],detail_points[current_box]))
#                 current_box = prev[current_box]
#             path.append((source_point,detail_points[current_box]))
#             boxes.append(src)
#             path.reverse()
#             boxes.reverse()
#             print("path: ",path)
#             return path,boxes
#         for neighbor in mesh['adj'][current_box]:
#             border = get_border(current_box, neighbor)
#             detail_points[neighbor] = get_entry_point(
#                 detail_points, current_box, border)
#             print("neighbor entry: ", detail_points[neighbor])
#             new_cost = dist[current_box] + elucidian_distance(
#                 detail_points[current_box], detail_points[neighbor]) 
#             if (neighbor not in dist) or (new_cost < dist[neighbor]):
#                 dist[neighbor] = new_cost
#                 current_dist = new_cost + elucidian_distance(
#                         detail_points[neighbor], destination_point)
#                 heappush(queue, (current_dist, neighbor))
#                 prev[neighbor] = current_box
                
        
#     return None,None
# bi-directional A*
def bi_a_star(source_point, destination_point, mesh):

    path = []
    boxes = []
    forward_queue = []
    backward_queue = []
    forward_dist = {}
    backward_dist = {}
    forward_prev = {}
    backward_prev = {}
    forward_detail_points = {}
    backward_detail_points = {}
    src = None
    dst = None
    found = False
    #get box
    src = find_box(source_point,mesh)
    dst = find_box(destination_point,mesh)
    #if the same box
    if src == dst and src != None:
       print("Same box!")
       return path, boxes
    #if click on somewhere that's not in a box
    if(src == None or dst == None):
        print("No path found")
        return path,boxes
    #initialize dictionaries
    heappush(forward_queue, (0,src))
    heappush(backward_queue, (0, dst))
    forward_dist[src] = 0
    backward_dist[dst] = 0
    forward_prev[src] = None
    backward_prev[dst] = None
    forward_detail_points[src] = source_point
    backward_detail_points[dst] = destination_point
    #start searching from both directions
    while(forward_queue and backward_queue):
        #get current box
        priority,current_box = heappop(forward_queue)
        #check if current hits backward search
        if current_box in backward_dist:
            found = True
            break
        for neighbor in mesh['adj'][current_box]:
            border = get_border(current_box,neighbor)
            #calculate entry point
            entry_point = get_entry_point(
                forward_detail_points, current_box, border)
            #entry to destination
            dist_dst = euclidian_distance(entry_point, destination_point)
            #update cost
            new_cost = forward_dist[current_box] + euclidian_distance(
                forward_detail_points[current_box], entry_point) + dist_dst
            # If the cost is new
            if neighbor not in forward_dist or new_cost < forward_dist[neighbor] + euclidian_distance(
                    forward_detail_points[neighbor], destination_point):
                forward_dist[neighbor] = new_cost - dist_dst
                forward_prev[neighbor] = current_box
                forward_detail_points[neighbor] = entry_point
                priority = new_cost
                heappush(forward_queue,(priority,neighbor))
        #Backward
        #get current box
        priority,current_box = heappop(backward_queue)
        #check if current hits forward search
        if current_box in forward_dist:
            found = True
            break
        for neighbor in mesh['adj'][current_box]:
            border = get_border(current_box,neighbor)
            #calculate entry point
            entry_point = get_entry_point(backward_detail_points,current_box,border)
            #entry to source
            dist_src = euclidian_distance(entry_point, source_point)
            #update cost
            new_cost = backward_dist[current_box] + euclidian_distance(
                backward_detail_points[current_box],entry_point) + dist_src
            # If the cost is new
            if neighbor not in backward_dist or new_cost < backward_dist[neighbor] + euclidian_distance(backward_detail_points[neighbor],source_point):
                backward_dist[neighbor] = new_cost - dist_src
                backward_prev[neighbor] = current_box
                backward_detail_points[neighbor] = entry_point
                priority = new_cost
                heappush(backward_queue,(priority,neighbor)) 


# append the result
    if found:
        #use to store current_box
        temp = current_box
        path.append((forward_detail_points[current_box], backward_detail_points[current_box]))
        #append to src
        while current_box != src:
            #(prev,current)
            path.append((forward_detail_points[forward_prev[current_box]],forward_detail_points[current_box]))
            boxes.append(current_box)
            current_box = forward_prev[current_box]
        path.append((source_point,forward_detail_points[current_box]))
        boxes.append(current_box) 
        #append to back
        current_box = temp
        while current_box != dst :
            #(prev,current)
            path.append((backward_detail_points[backward_prev[current_box]],backward_detail_points[current_box]))
            boxes.append(current_box)
            current_box = backward_prev[current_box]
        path.append((destination_point,backward_detail_points[current_box]))
        boxes.append(current_box)
    else:
        print("No path found")

    return path,boxes


def euclidian_distance(x,y):
    return sqrt((x[0]-y[0])**2 + (x[1]-y[1])**2)
# useless
# def find_mid_point(border):
#     return ((border[0] + border[1])/2 ,(border[2] + border[3])/2) 

def get_border(b1,b2):
    x_range = max(b1[0],b2[0]),min(b1[1],b2[1])
    y_range = max(b1[2],b2[2]),min(b1[3],b2[3])
    border = x_range + y_range
    return border


def find_box(point,mesh):
    target = None
    
   #find source box and destination box
    for box in mesh['boxes']:
        # print("point: ",point[0],point[1])
        # print("box: ",box[0],box[1])
        if point[0] in arange(box[0], box[1]):
            if point[1] in arange(box[2], box[3]):
                target = box
                return target
    return None
def get_entry_point(detail_points, current_box, border):
    # less than max, more than min, in_between
    if(detail_points[current_box][0] < border[0]):
        x = border[0]
    elif detail_points[current_box][0] > border[1]:
        x = border[1]
    else:
        x = detail_points[current_box][0]
    # same as x 
    if(detail_points[current_box][1] < border[2]):
        y = border[2]
    elif detail_points[current_box][1] > border[3]:
        y = border[3]
    else:
        y = detail_points[current_box][1]

    return (x,y)

  #  return None
