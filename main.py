import myNetwork
import random

n = 10 ## number of nodes
p = 0.2 ## the probability of edges to be generated in the Erdős-Rényi model
seed = 42

random.seed(seed)

net = myNetwork.Graph(n,p,seed,table_type="naive")


nodes = net.nodes
for i in range(len(nodes)):
    print(f"routing table of node {i} is {nodes[i].table}")

print(f"net.shortest_dis is {net.shortest_dist}")

dist, stretch, path  = net.send_packet(0,3)
print(f"the distance of the routing path is {dist}, stretch is {stretch}")
print(f"the path of the packe is {path}")

net.drawGraph()
