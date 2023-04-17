import networkx as nx
import matplotlib.pyplot as plt
import random
import node
import numpy as np
import packet
import math
import util

class Graph():

    def __init__(self,n,p,seed) -> None:
        self.n = n
        g = nx.fast_gnp_random_graph(n,p,seed=seed)
        while(not nx.is_connected(g)):
            print("it is not connected, try another one")
            g = nx.fast_gnp_random_graph(n,p)    

        self.g = g
        print(g)
        self.adj_matrix = self.create_weighted_adj_matrix(g.edges,self.n)
        self.nodes = {}
       
        self.s_bound,self.num_iter_bound,self.A_bound,self.C_bound = util.get_bounds(n)

        # self.generate_routing_table(table_type)
        # print(f"edges in the graph are {g.edges}")
        # print(f"nodes of graph are {g.nodes}")
        # print(f"adj matrix is {self.adj_matrix}")

    # def copy_net(self,net):
    #     self.n = net.n
    #     self.g = net.g
    #     self.adj_matrix = net.adj_matrix
    #     self.s_bound,self.num_iter_bound,self.A_bound,self.C_bound = util.get_bounds(self.n)
    #     self.nodes[self.table_type] = []
    #     for i in range(self.n):
    #         self.nodes[self.table_type].append(node.Node(i))
    #         ports = []
    #         for j in range(self.n):
    #             if self.adj_matrix[i][j] !=0:
    #                 ports.append(j)
    #         self.nodes[self.table_type][i].initPort(ports)
        

    def drawGraph(self):
        nx.draw_networkx(self.g)
        plt.draw() 
        plt.show() 


    def create_weighted_adj_matrix(self,edge_list,n):
        adj_matrix = [[0 for column in range(n)]
                        for row in range(n)]
        for edge in edge_list:
            weight = random.randint(1, 100)
            adj_matrix[edge[0]][edge[1]] = weight
            adj_matrix[edge[1]][edge[0]] = weight

        return adj_matrix
    
    def generate_shortest_dist(self):
        naive_routing_table = []
        shortest_dists = []
        for i in range(self.n):
            dist, lastNodes, _ = self.dijkstra(i,self.n)
            naive_routing_table.append(lastNodes)
            shortest_dists.append(dist)

        naive_routing_table = np.array(naive_routing_table).T.tolist()
        self.shortest_dist = shortest_dists
        return naive_routing_table,shortest_dists
        
    
    def generate_routing_table(self,table_type):
        self.nodes[table_type] = []
        for i in range(self.n):
            self.nodes[table_type].append(node.Node(i))
            ports = []
            for j in range(self.n):
                if self.adj_matrix[i][j] !=0:
                    ports.append(j)
            self.nodes[table_type][i].initPort(ports)
        naive_routing_table,shortest_dists = self.generate_shortest_dist()
        if table_type == "thorup":
            self.thorup_zwick_table(naive_routing_table)
        elif table_type == "naive":
            print(f"size of naive routing table is {len(naive_routing_table[0])}")
            for i in range(self.n):
                self.nodes[table_type][i].initTable(naive_routing_table[i]) 
        else:
            raise Exception("Please input the correct routing scheme")
        return naive_routing_table, shortest_dists
    
    def thorup_zwick_table(self,naive_routing):
        s = math.ceil((self.n/math.log2(self.n))**(1/2)) ## s = (n/logn)^(1/2) to make size of routing table to be O((nlogn)^(1/2))
        # print(f"s is {s}")
        A,centers,C_A= self.center(s)
        # print(f"A includes {A}")
        # print(f"CA is {C_A}")
        max_table_size = 0
        for w in range(self.n):
            table = {}
            # print(f"C_A(w) is {C_A[w]}")
            ## 2-level hash table TAB_w for each v in A U C(w)
            for v in A:
                table[v] = naive_routing[w][v] ##(v,port(w,v))
            for v in C_A[w]:
                assert v not in table
                table[v] = naive_routing[w][v]
            
            self.nodes["thorup"][w].initTable(table)
            if len(table)>max_table_size:
                max_table_size = len(table)
            label_w = [w,centers[w],naive_routing[centers[w]][w]]
            # print(f"label_w is {label_w}")
            self.nodes["thorup"][w].initLabel(label_w)

        print(f"max size of a thorup table is {max_table_size}")
    
    def delta_A_u(self,A,v):
        min = float('inf')
        cent = -1
        for u in A:
            if self.shortest_dist[u][v]<min:
                min = self.shortest_dist[u][v]
                cent = u
        # print(f"min is {min}")
        assert cent!=-1
        return min,cent

    def clusters(self,A):
        C = [[] for i in range(self.n)]
        centers = [0 for i in range(self.n)] 
        max_nodes_in_C = 0
        for v in range(self.n):
            min,cent = self.delta_A_u(A,v)
            centers[v] = cent
            for w in range(self.n):
                if self.shortest_dist[w][v]<min:
                    C[w].append(v)
                if len(C[w])>max_nodes_in_C:
                    max_nodes_in_C = len(C[w])
        
        assert len(centers)== self.n
        return C,centers,max_nodes_in_C

    def center(self,s):
        
        def sample(W,s):
            temp = []
            p = s/len(W)
            # print(f"p is {p}, W is {W}")
            if len(W)<s:
                return W
            else:
                for i in W:
                    if random.uniform(0, 1)<=p:
                        temp.append(i)
            # print(f"samples from w is {temp}")            
            return temp
        
        A = []
        W = [i for i in range(self.n)]
        num_iter = 0
        while(W):
            num_iter +=1
            ### A<-A Union sample(W,s)
            A = A + sample(W,s)
            # print(f"A is {A}")
            
            ### C(w) for every w in V
            C,centers,max_nodes_in_C = self.clusters(A)
            # print(f"C is {C}")

            ### update W
            W = []
            for w in range(self.n):
                if len(C[w])>4*self.n/s:
                    W.append(w)
                     
        print(f"s is {self.s_bound}, we use the ceil {s}")        
        print(f"the centers/A are {A}, size is {len(A)}, upper bound is {self.A_bound}")
        print(f"number of iterations to sample is {num_iter}, upper bound is {self.num_iter_bound}")
        print(f"max number of nodes in a cluster is {max_nodes_in_C}, upper bound is {self.C_bound}")
        # print(f"closets centers are {centers},length is {len(centers)}")
        return A,centers,C          
    
    def send_packet(self,init,dest,table_type):
        newPacket = packet.Packet(init,dest)
        dist = 0
        dist_seg = []
        newPacket.extend_path(init)

        if table_type == "naive":
            while(newPacket.current != newPacket.dest):
                current_node = self.nodes[table_type][newPacket.current]
                # print(f"current node table is {current_node.table}")
                next_node_index = current_node.table[dest]
                path_seg_dist = self.adj_matrix[newPacket.current][next_node_index]
                dist += path_seg_dist
                dist_seg.append(path_seg_dist)
                newPacket.current = next_node_index
                newPacket.extend_path(next_node_index)
        elif table_type == "thorup":
            newPacket.header = self.nodes[table_type][newPacket.dest].label ## header takes the label of the destination
            while(newPacket.current != newPacket.dest):
                # print(f"newPacket.current is {newPacket.current}")
                current_node = self.nodes[table_type][newPacket.current]
                # print(f"routing table of current node is {current_node.table}, label is {current_node.label}")
                # print(f"self.nodes[table_type] are {self.nodes[table_type]}")
                if dest in current_node.table:
                    # print(f"heading to destination directedly")
                    next_node_index = current_node.table[dest]
                elif newPacket.header[1] == current_node.label[0]: ## w = cent(v)
                    # print(f"currently at center, heading to destination from a center")
                    port = newPacket.header[2] ## port(cent(v),v)
                    # print(f"port in the header is {port}")
                    assert self.adj_matrix[newPacket.current][port] != 0 ## be sure that this port exits
                    next_node_index = port
                else:
                    # print(f"heading to a center")
                    cent_v = newPacket.header[1]
                    # print(f"closest center to the destination is {cent_v}")
                    next_node_index = current_node.table[cent_v]
                
                path_seg_dist = self.adj_matrix[newPacket.current][next_node_index]
                dist += path_seg_dist
                dist_seg.append(path_seg_dist)
                newPacket.current = next_node_index
                newPacket.extend_path(next_node_index)
        if init!=dest:
            stretch = dist/self.shortest_dist[init][dest] 
        else:
            stretch = 0
            
        return dist, stretch, newPacket.path, dist_seg

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