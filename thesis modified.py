

import sys
import arcpy
import numpy
import math
import timeit


NEIGHBOUR_CELL_NUMBER_MAX = 1

#Find the shortest path between start and end nodes in a graph
def shortestpath(graph,start,end, max_x, max_y, visited=[],distances={},predecessors={}):
    
    unvisited_nodes = create_nodes_list(graph)

    

    # we've found our end node, now find the path to it, and return
    while start != end:
        
        print start


        # detect if it's the first time through, set current distance to zero
        if not visited: distances[start]=0

        start_x, start_y = get_node_coordinates(start)

        if predecessors == {}:
            previous_node = start
        else: 
            previous_node = predecessors[start]
    
        
        xprev, yprev = get_node_coordinates(previous_node)

        x_relative = xprev - start_x
        y_relative = yprev - start_y
        
        # process neighbors as per algorithm, keep track of predecessors
        neighbours = find_neighbour_nodes(start_x, start_y, x_relative, y_relative, max_x, max_y)

        for neighbour in neighbours:
            if neighbour not in visited:

                neighbour_x, neighbour_y = get_node_coordinates(neighbour)
                neighbour_distance = calculate_distance(start_x, start_y, neighbour_x, neighbour_y)

                neighbour_slope = calculate_slope(neighbour_distance, graph[start_x][start_y], graph[neighbour_x][neighbour_y])
                neighbour_weight = calculate_weight(neighbour_slope, neighbour_distance)

                neighbourdist = distances.get(neighbour, sys.maxint)
                tentativedist = distances[start] + neighbour_weight
                if tentativedist < neighbourdist:

                    distances[neighbour] = tentativedist
                    predecessors[neighbour]=start

        # neighbors processed, now mark the current node as visited
        visited.append(start)
        unvisited_nodes.remove(start)

        # finds the closest unvisited node to the start
        unvisiteds = dict((k, distances.get(k,sys.maxint)) for k in unvisited_nodes)
        closestnode = min(unvisiteds, key=unvisiteds.get)

        start = closestnode

    
    path=[]
    while end != None:
        path.append(end)
        end=predecessors.get(end,None)
    return distances[start], path[::-1]

#creates a list of all nodes in the graph
def create_nodes_list(graph):
    node_list = []
    for x in range(0,len(graph[0])):
        for y in range(0,len(graph)):
            node_list.append(get_node_id(x,y))
    
    return node_list

# #create a list of all arcs in the graph
# def generate_arc_list(graph, list_of_nodes, neighbour_nodes):
#     arc_list = []


# #create dictionary af all arcs in the graph
# def dictionary_arcs_w_weight(graph, list_of_nodes, neighbour_nodes, weights):
#     arc_dictionary = {}



#calculates distance between current node and neighbour
def calculate_distance(x1, y1, x2, y2):
    return math.sqrt(((x1-x2) * cellSize)**2 + ((y1-y2) * cellSize)**2)

#calculates weight from distance and slope
def calculate_weight(slope, distance):
    return distance + (distance * slope)
    

#calculates slope as an absolute vale, applies piecewise scale for interpreting slope
def calculate_slope(distance, z1, z2):
    slope = abs((((z1) - (z2)) / distance) * 100)

    weighted_slope = 0

    if slope >= 0 and slope <= 5:
        weighted_slope = 0
    elif slope > 5 and slope <= 10:
        weighted_slope = 1
    elif slope > 10 and slope <= 15:
        weighted_slope = 2
    elif slope > 15:
        weighted_slope = 1000000

    return weighted_slope

# #finds all neuighbour nodes, based on the global NEIGHBOUR_CELL_NUMBER_MAX value
# def find_neighbour_nodes(x1, y1, max_x, max_y):
#     neighbours = []

#     for x_value in range(-1*NEIGHBOUR_CELL_NUMBER_MAX, 1*NEIGHBOUR_CELL_NUMBER_MAX+1):
#         for y_value in range(-1*NEIGHBOUR_CELL_NUMBER_MAX, 1*NEIGHBOUR_CELL_NUMBER_MAX+1):
#             neighbour_x = x1+x_value
#             neighbour_y = y1+y_value

#             if not (neighbour_x >= max_x or neighbour_x < 0 or neighbour_y >= max_y or neighbour_y < 0 or (neighbour_x == x1 and neighbour_y == y1)):
#                 neighbours.append(get_node_id(neighbour_x, neighbour_y))

#     return neighbours

    #finds all neuighbour nodes, based on the global NEIGHBOUR_CELL_NUMBER_MAX value
def find_neighbour_nodes(x1, y1, xprev, yprev, max_x, max_y):
    neighbours = []
    x_values = []
    y_values = []
        
    if xprev == 0 and yprev == 0:
        x_values = [-1, 0, 1]
        y_values = [-1, 0, 1]
    elif xprev == -1 and yprev == -1:
        x_values = [0, 1]
        y_values = [0, 1]
    elif xprev == 0 and yprev == -1:
        x_values = [-1, 0, 1]
        y_values = [1]
    elif xprev == 1 and yprev == -1:
        x_values = [-1, 0]
        y_values = [0, 1]
    elif xprev == -1 and yprev == 0:
        x_values = [1]
        y_values = [-1, 0, 1]
    elif xprev == 1 and yprev ==  0:
        x_values = [-1]
        y_values = [-1, 0, 1]
    elif xprev == -1 and yprev == 1:
        x_values = [0, 1]
        y_values = [-1, 0]
    elif xprev == 0 and yprev == 1:
        x_values = [-1, 0, 1]
        y_values = [-1]
    elif xprev == 1 and yprev == 1:
        x_values = [-1, 0]
        y_values = [-1, 0]

    for x_value in x_values:
        for y_value in y_values:
            neighbour_x = x1+x_value
            neighbour_y = y1+y_value

              

            if not (neighbour_x >= max_x or neighbour_x < 0 or neighbour_y >= max_y or neighbour_y < 0 or (neighbour_x == x1 and neighbour_y == y1)):
                neighbours.append(get_node_id(neighbour_x, neighbour_y))

    return neighbours


#interprets cell position to generate a string identifier for the cells
def get_node_id(x, y):
    return str(x) + "," + str(y)

#splits the coordinates into x or z and numeric value
def get_node_coordinates(id):
    split = id.split(',')
    return int(split[0]), int(split[1])

#takes list of coordinates and changes the z values in the array to 1
def selected_cells(path_coordinates, graph):
    for coordinate in path_coordinates:
        x, y = get_node_coordinates(coordinate)
        graph[x][y] = 1
    return graph

# makes all points not in the path 0
def zeroing_graph_values(graph):
    for x in range(0,len(graph[0])):
        for y in range(0,len(graph)):
            graph[x][y] = 0
    return graph

#creates a weight for 90 degree turning radius
def radius_of_curve(x1, y1, x2, y2, x3, y3):
    pass




if __name__ == "__main__":

    

    start_timer = timeit.default_timer()
    print start_timer

    raster = arcpy.Raster(r'C:\Users\Administrator\Documents\Lakehead\thesis\lid_rast\Run8\lid_8.tif')
    lowerLeft = arcpy.Point(raster.extent.XMin,raster.extent.YMin)
    cellSize = raster.meanCellWidth

    graph = arcpy.RasterToNumPyArray(raster, lowerLeft, nodata_to_value=100000)

    max_x = len(graph[0])
    max_y = len(graph)

    weight, shortest_path = shortestpath(graph, "0,0", "199,199", max_x, max_y)
    print weight, shortest_path

    zeroed_graph = zeroing_graph_values(graph)

    route = selected_cells(shortest_path, zeroed_graph)

    myraster = arcpy.NumPyArrayToRaster (route, lowerLeft, cellSize, cellSize, 0) 
    myraster.save (r'C:\Users\Administrator\Documents\Lakehead\thesis\lid_rast\Run8\lid_8_route.tif')


    stop_timer = timeit.default_timer()
    print stop_timer
    

    print ('Time: ', stop_timer - start_timer)
