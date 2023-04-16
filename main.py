import myNetwork
import random

n = 250 ## number of nodes
p = 0.1 ## the probability of edges to be generated in the Erdős-Rényi model
initial_node_index = 150
destination_node_index = 26
seed = 42
routing_scheme = "thorup" ### "naive" and "thorup"
# routing_scheme = "naive" ### "naive" and "thorup"

random.seed(seed)

net = myNetwork.Graph(n,p,seed,routing_scheme)

nodes = net.nodes
# for i in range(len(nodes)):
#     print(f"routing table of node {i} is {nodes[i].table}")
#     print(f"label of node {i} is {nodes[i].label}")

# print(f"net.shortest_dis is {net.shortest_dist}")

stretch_set_all = []
for j in range(n):
    initial_node_index = j
    stretch_set = []
    for i in range(n):
        destination_node_index = i
        dist, stretch, path, dis_seg  = net.send_packet(initial_node_index,destination_node_index,routing_scheme)
        stretch_set.append(stretch)
        # print(f"from {initial_node_index} to {destination_node_index} the distance of the routing path is {dist}, stretch is {stretch}. Path of the packet is {path} and segement distances are {dis_seg}")
    # print(f"max of stretch when initiator is {j} is {max(stretch_set)}")
    stretch_set_all.append(max(stretch_set))

print(f"max of stretch in the graph is {max(stretch_set_all)}")
    
# A = net.thorup_zwick_table()

# print(f"A is {A}, size of A is {len(A)}")

# net.drawGraph()
