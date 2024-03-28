# compact_routing
This is an implementation of Stretch3 in section 3 of paper Thorup, Mikkel, and Uri Zwick. "Compact routing schemes." Proceedings of the thirteenth annual ACM symposium on Parallel algorithms and architectures. 2001.

python main.py [n] [init] [dest] --max_stretch

eg. "pyhthon main.py 250 0 1 --max_stretch" will create a connected graph with 250 nodes. A packet is sent from node 0 to node 1 with two different methods. The max stretch of two routing scheme are also printed by trying all possible routing path.

required packages: numpy, matplotlib, networkx
