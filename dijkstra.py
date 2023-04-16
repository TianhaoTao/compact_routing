# Python program for Dijkstra's single
# source shortest path algorithm. The program is
# for adjacency matrix representation of the graph
import random
import math

class Graph():
 
    def __init__(self, vertices,bandwidth_range):
        self.V = vertices
        self.edge_list = []
        self.edge_list_bandwidth = []
        self.graph = [[0 for column in range(vertices)]
                      for row in range(vertices)]
        self.bandwidth_range = bandwidth_range
 
    def printSolution(self, dist):
        print("Vertex \t Distance from Source")
        for node in range(self.V):
            print(node, "\t\t", dist[node])
 
    # A utility function to find the vertex with
    # minimum distance value, from the set of vertices
    # not yet included in shortest path tree
    def minDistance(self, dist, sptSet):
 
        # Initialize minimum distance for next node
        min = 1e7
 
        # Search not nearest vertex not in the
        # shortest path tree
        for v in range(self.V):
            if dist[v] < min and sptSet[v] == False:
                min = dist[v]
                min_index = v
 
        return min_index
 
    # Function that implements Dijkstra's single source
    # shortest path algorithm for a graph represented
    # using adjacency matrix representation
    def dijkstra(self, src):
 
        dist = [1e7] * self.V
        dist[src] = 0
        sptSet = [False] * self.V
 
        for cout in range(self.V):
 
            # Pick the minimum distance vertex from
            # the set of vertices not yet processed.
            # u is always equal to src in first iteration
            u = self.minDistance(dist, sptSet)
 
            # Put the minimum distance vertex in the
            # shortest path tree
            sptSet[u] = True
 
            # Update dist value of the adjacent vertices
            # of the picked vertex only if the current
            # distance is greater than new distance and
            # the vertex in not in the shortest path tree
            for v in range(self.V):
                if (self.graph[u][v] > 0 and
                   sptSet[v] == False and
                   dist[v] > dist[u] + self.graph[u][v]):
                    dist[v] = dist[u] + self.graph[u][v]
 
        # self.printSolution(dist)
        return dist
    
    def shortest_path_table(self):
        n = self.V
        shortest_path = []
        for i in range(n):
            shortest_path.append(self.dijkstra(i))
        return shortest_path
    
    def bandwidth_edge_list(self):
        edge_list_bandwidth = []
        for edge in self.edge_list:
            edge_list_bandwidth.append(1/self.graph[edge[0]][edge[1]])
        return edge_list_bandwidth
    
    def update_bandwidth(self):
        for edge in self.edge_list:
            if self.graph[edge[0]][edge[1]] == 0:
                bandwidth = random.uniform(self.bandwidth_range[0], self.bandwidth_range[1])
                # bandwidth = 10
                self.graph[edge[0]][edge[1]] = 1/bandwidth
                self.graph[edge[1]][edge[0]] = 1/bandwidth


    def edge_list_to_adj_matrix(self,e_list,n):
        self.edge_list = e_list
        adj_matrix = [[0 for j in range(n)] for i in range(n)]
        for edge in e_list:
            if adj_matrix[edge[0]][edge[1]] == 0:
                bandwidth = random.uniform(self.bandwidth_range[0], self.bandwidth_range[1])
                # bandwidth = 10
                adj_matrix[edge[0]][edge[1]] = 1/bandwidth
                adj_matrix[edge[1]][edge[0]] = 1/bandwidth
        self.graph = adj_matrix
        return adj_matrix

def generate_ring(n):
    edge_list = []
    for i in range(n-1):
        edge_list.append([i,i+1])
        edge_list.append([i+1,i])
    edge_list.append([n-1,0])
    edge_list.append([0,n-1])
    return edge_list

def generate_star(n):
    edge_list = []
    for i in range(1,n):
        edge_list.append([0,i])
        edge_list.append([i,0])
    return edge_list


def generate_tree(n):
    edge_list = []
    for i in range(n):
        if i*2+1<n:
            edge_list.append([i,i*2+1])
            edge_list.append([i*2+1,i])
        if i*2+2<n:
            edge_list.append([i,i*2+2])
            edge_list.append([i*2+2,i])
    return edge_list


# Driver program
# g = Graph(9)
# g.graph = [[0, 4, 0, 0, 0, 0, 0, 8, 0],
#            [4, 0, 8, 0, 0, 0, 0, 11, 0],
#            [0, 8, 0, 7, 0, 4, 0, 0, 2],
#            [0, 0, 7, 0, 9, 14, 0, 0, 0],
#            [0, 0, 0, 9, 0, 10, 0, 0, 0],
#            [0, 0, 4, 14, 10, 0, 2, 0, 0],
#            [0, 0, 0, 0, 0, 2, 0, 1, 6],
#            [8, 11, 0, 0, 0, 0, 1, 0, 7],
#            [0, 0, 2, 0, 0, 0, 6, 7, 0]
#            ]

# n = 7
# edge_list = generate_ring(n)
# # edge_list = generate_star(n)
# print(edge_list)
# g = Graph(n)
# adj_m = g.edge_list_to_adj_matrix(edge_list,n)

# print(f"adj_m is {adj_m}")

# g.graph = adj_m

# print(f"shortest path is {g.shortest_path_table()}")
# print(f"bandwidth is {g.bandwidth_edge_list()}")
 
# # This code is contributed by Divyanshu Mehta



    def cluster(self,A,v):
        C_v = []
        for u in range(self.n):
            if self.adj_matrix[v][u]<self.delta_A_u(A,v):
                C_v.append(u)
        return C_v