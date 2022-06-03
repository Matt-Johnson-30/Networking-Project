# Matt Johnson, Katie Hay, Lauren Gilbert
# Final Project 11/13/20
# CSC 450 - Timofeyev

from sys import argv
from csv import *


class NODE():
    def __init__(self):
        self.name = ""
        self.vector = []
        # holds the master index of neighboring nodes
        self.neighbors = []
        self.min_neighbor = 9999
        self.neighbors_open = []


    def find_neighbors(self, node_list):
        for i in range(len(self.vector)):
                if self.vector[i] != 9999 and self.vector[i] != 0:
                    self.neighbors.append(i)

################################################# DISTANCE VECTOR #################################################
    def update(self, node_list):
        # keeps track if the nodes are actually updating
        changed = 0
        # update the path based on neighbors
        for neighbor in self.neighbors:
            temp_cost = 9999
            # grab the the distances to each link from the neighbor
            for i in range(len(node_list[neighbor].vector)):
                # add the cost to get to neighbor with the cost to get to any other node from the neighbor
                temp_cost = self.vector[neighbor] + node_list[neighbor].vector[i]
                # Bellman-Ford
                if (temp_cost < self.vector[i]):
                    self.vector[i] = temp_cost
                    changed = 1
        return changed      
        
 ###################################################################################################################       
    def Min_Neighbor(self, unvisited, node_list):
        min_neighbor = 9999
        min_index = 0

        # store indexes of nodes in unvisited list
        unvisited_index = []
        for j in range(len(node_list)):
            for i in unvisited:
                if i == j:
                    unvisited_index.append(j)

        # update the open neighbors for every node
        for node in node_list:
            node.neighbors_open = []
            # index of neighbors
            for neighbor in node.neighbors:
                # index of unvisited nodes
                for index in unvisited:
                    # if they are equal, then that neighbor is open
                    if index == neighbor:
                        node.neighbors_open.append(index)

        # use list of indexes in unvisited list to find minimum
        for k in unvisited_index:
            if (self.vector[k] < min_neighbor and self.vector[k] != 0):
                min_neighbor = self.vector[k]
                min_index = k
        self.min_neighbor = min_index

################################################### DIJKSTRA'S ###################################################
def dijkstra(node_list, source_node, names):
    path_tree = []
    path = source_node.name
    last_path = ""
    last_cost = "{}:0, ".format(source_node.name)
    costs = []
    Fcost = 0

    unvisited_nodes = []
    for i in range(len(node_list)):
        unvisited_nodes.append(i)
    # remove source node from list
    unvisited_nodes.remove(node_list.index(source_node))
    
    current_node = source_node
    # while the list of unvisited nodes is not empty
    while(len(unvisited_nodes) != 0):
        # initiate the least cost neighbor
        current_node.Min_Neighbor(unvisited_nodes, node_list)
        # take least cost path
        Fcost += current_node.vector[current_node.min_neighbor]
        path += node_list[current_node.min_neighbor].name
        current_node = node_list[current_node.min_neighbor]
        unvisited_nodes.remove(node_list.index(current_node))

        temp_cost = 0
        temp_path = source_node.name
        temp_costs = []
        temp_paths = []
        previous_node = source_node
        # takes an alternate path, along the least cost path then compares the two
        for node in path:
            node = node_list[names.index(node)]
            # if this is NOT the source node add the predetermind smallest cost
            if node != source_node:
                temp_cost += previous_node.vector[previous_node.min_neighbor]
                temp_path += node.name
                previous_node = node
            for neighbor in node.neighbors_open:
                # we have a rival!!
                if neighbor == node_list.index(current_node):
                    temp_costs.append(temp_cost + node.vector[neighbor])
                    temp_paths.append(temp_path + names[neighbor])

        Fpath = path
        cost = Fcost
        # smaller wins
        for i in range(len(temp_paths)):
            temp_cost = temp_costs[i]
            if temp_cost < cost:
                cost = temp_cost
                Fpath = temp_paths[i]

        path_tree.append(Fpath)
        last_path += "{}, ".format(Fpath)
        costs.append(cost)
        last_cost += "{}:{}, ".format(Fpath[-1:],cost)
                              
    print("\nShortest path tree for node {}:".format(source_node.name))
    print(last_path[:-2])
    print("Costs of the least-cost paths for node {}:".format(source_node.name))
    print(last_cost[:-2])
    print()


################################################# INITIALIZATION #################################################
NODES = []
names = []

try:
    filename = argv[1]
    SOURCE_NODE = input("Please, provide the source node: ")
except:
    print("To run this program:\npython3 Project.py network.csv")

# creates 2 lists
# 1 holds the name of the nodes
# 1 holds the values of the edges 
try:
    with open(filename, newline='') as network:
        contents = reader(network, delimiter=',')
        counter = 1
        for line in contents:
            # first line contains duplicate info
            if counter != 1:
                # adds each node to a list after initializing its distance vector and neighbors
                node = NODE()
                node.name = line[0]
                for i in range(len(line[1:])):
                    node.vector.append(int(line[i+1]))
                NODES.append(node)
            counter += 1
except:
    print("file not found\nTo run this program:\npython3 Project.py topology.csv")


############################################# WHERE THE MAGIC HAPPENS #############################################
# initializes the neighbor fields for each node
for node in NODES:
    node.find_neighbors(NODES)
    names.append(node.name)
    # update source node variable with node object
    if node.name == SOURCE_NODE:
        SOURCE_NODE = node

# call the function (prints in function)
dijkstra(NODES, SOURCE_NODE, names)

# updates the nodes until there are no more changes
changed = 1
while changed:
    for node in NODES:
        changed = node.update(NODES)

for node in NODES:
    print("Distance vector for the node {}:".format(node.name), node.vector)
