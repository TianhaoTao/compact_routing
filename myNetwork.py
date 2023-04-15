import networkx as nx
import matplotlib.pyplot as plt
import random
import node
import numpy as np
import packet

class Graph():

    def __init__(self,n,p,seed,table_type) -> None:
        self.n = n
        g = nx.fast_gnp_random_graph(n,p,seed=seed)
        while(not nx.is_connected(g)):
            print("it is not connected, try another one")
            g = nx.fast_gnp_random_graph(n,p)    

        self.g = g
        self.adj_matrix = self.create_weighted_adj_matrix(g.edges,self.n)
       
        self.nodes = []
        for i in range(n):
            self.nodes.append(node.Node(i))
            ports = []
            for j in range(n):
                if self.adj_matrix[i][j] !=0:
                    ports.append(j)
            self.nodes[i].initPort(ports)

        _,self.shortest_dist = self.naive_routing_table(table_type)

        print(g)
        print(f"edges in the graph are {g.edges}")
        print(f"nodes of graph are {g.nodes}")
        print(f"adj matrix is {self.adj_matrix}")

    def drawGraph(self):
        nx.draw_networkx(self.g)
        plt.draw() 
        plt.show() 

    def create_weighted_adj_matrix(self,edge_list,n):
        adj_matrix = [[0 for column in range(n)]
                        for row in range(n)]
        for edge in edge_list:
            weight = random.randint(1, 10)
            adj_matrix[edge[0]][edge[1]] = weight
            adj_matrix[edge[1]][edge[0]] = weight

        return adj_matrix
    
    def naive_routing_table(self,table_type):
        naive_routing_table = []
        shortest_dists = []
        for i in range(self.n):
            dist, lastNodes, sptSet = self.dijkstra(i,self.n)
            naive_routing_table.append(lastNodes)
            shortest_dists.append(dist)

        naive_routing_table = np.array(naive_routing_table).T.tolist()
        if table_type == "naive":
            for i in range(self.n):
                self.nodes[i].initTable(naive_routing_table[i])  
        return naive_routing_table, shortest_dists
    
    def send_packet(self,init,dest):
        newPacket = packet.Packet(init,dest)
        dist = 0
        newPacket.extend_path(init)

        while(newPacket.current != newPacket.dest):
            current_node = self.nodes[newPacket.current]
            next_node_index = current_node.table[dest]
            dist += self.adj_matrix[newPacket.current][next_node_index]
            newPacket.current = next_node_index
            newPacket.extend_path(next_node_index)
        stretch = dist/self.shortest_dist[init][dest]
        return dist, stretch, newPacket.path

    # Function that implements Dijkstra's single source
    # shortest path algorithm for a graph represented
    # using adjacency matrix representation
    def dijkstra(self, src, k):## k is the closest number
    
        def minDistance(dist, sptSet):
    
            # Initialize minimum distance for next node
            min = float('inf')
    
            # Search not nearest vertex not in the
            # shortest path tree
            for v in range(self.n):
                if dist[v] < min and sptSet[v] == False:
                    min = dist[v]
                    min_index = v
    
            return min_index
 
        dist = [float('inf')] * k
        dist[src] = 0
        sptSet = [False] * k
        lastNodes = [src] * k
 
        for cout in range(k):
 
            # Pick the minimum distance vertex from
            # the set of vertices not yet processed.
            # u is always equal to src in first iteration
            u = minDistance(dist, sptSet)
 
            # Put the minimum distance vertex in the
            # shortest path tree
            sptSet[u] = True
 
            # Update dist value of the adjacent vertices
            # of the picked vertex only if the current
            # distance is greater than new distance and
            # the vertex in not in the shortest path tree
            for v in range(self.n): ## for all nodes, instead of k nearest neighbours
                if (self.adj_matrix[u][v] > 0 and
                   sptSet[v] == False and
                   dist[v] > dist[u] + self.adj_matrix[u][v]):
                    dist[v] = dist[u] + self.adj_matrix[u][v]
                    lastNodes[v] = u
            
 
        # self.printSolution(dist)
        return dist, lastNodes, sptSet