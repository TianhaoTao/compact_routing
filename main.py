import myNetwork
import random
import argparse

def initialization(all_stretch):
    net = myNetwork.Graph(n,p,seed)
    net.generate_routing_table("naive")
    net.generate_routing_table("thorup")
    # net_thorup = myNetwork.Graph(n,p,seed)
    # net_thorup.copy_net(net)
    # net_thorup.generate_routing_table("thorup")

    # nodes = net.nodes
    # for i in range(len(nodes)):
    #     print(f"routing table of node {i} is {nodes[i].table}")
    #     print(f"label of node {i} is {nodes[i].label}")

    # print(f"net.shortest_dis is {net.shortest_dist}")
    
    # print(f"net nodes are: {len(net.nodes)}")

    if all_stretch:
        stretch_set_all_naive = []
        stretch_set_all_thorup = []
        for j in range(n):
            stretch_set_naive = []
            stretch_set_thorup = []
            for i in range(n):
                dist_naive, stretch_naive, path_naive, dis_seg_naive  = net.send_packet(j,i,"naive")
                dist_thorup, stretch_thorup, path_thorup, dis_thorup  = net.send_packet(j,i,"thorup")
                stretch_set_all_naive.append(stretch_naive)
                stretch_set_all_thorup.append(stretch_thorup)
                # print(f"from {initial_node_index} to {destination_node_index} the distance of the routing path is {dist}, stretch is {stretch}. Path of the packet is {path} and segement distances are {dis_seg}")
            stretch_set_all_naive.append(max(stretch_set_all_naive))
            stretch_set_all_thorup.append(max(stretch_set_all_thorup))

        print(f"max of stretch by naive routing is {max(stretch_set_all_naive)}")
        print(f"max of stretch by thorup is {max(stretch_set_all_thorup)}")

    # net.drawGraph()
    return net

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('n',type=int,nargs='?',default=256)
    parser.add_argument('init',type=int,nargs='?',default=33)
    parser.add_argument('dest',type=int,nargs='?',default=111)
    parser.add_argument('--max_stretch',action="store_true")
    
    args = parser.parse_args()    
    
    n = args.n ## number of nodes
    p = 0.1 ## the probability of edges to be generated in the Erdős-Rényi model
    initial_node_index = args.init
    destination_node_index = args.dest
    seed = 42
    all_stretch = args.max_stretch

    random.seed(seed)

    net = initialization(all_stretch)


    dist_naive, stretch_naive, path_naive, dis_seg_naive  = net.send_packet(initial_node_index,destination_node_index,"naive")
    dist_thorup, stretch_thorup, path_thorup, dis_thorup  = net.send_packet(initial_node_index,destination_node_index,"thorup")
    print(f"from {initial_node_index} to {destination_node_index} the distance of the routing path is {dist_naive}, stretch is {stretch_naive}. Path of the packet is {path_naive} and segement distances are {dis_seg_naive}")
    print(f"from {initial_node_index} to {destination_node_index} the distance of the routing path is {dist_thorup}, stretch is {stretch_thorup}. Path of the packet is {path_thorup} and segement distances are {dis_thorup}")
