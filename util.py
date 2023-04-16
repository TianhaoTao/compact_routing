import math

def get_bounds(n):
    s = (n/math.log2(n))**(1/2)
    num_iter = 2*math.log2(n)
    upper_bound_of_A = 2*(n*math.log2(n))**(1/2)
    upper_bound_of_C = 4* (n*math.log2(n))**(1/2)
    return s,num_iter,upper_bound_of_A,upper_bound_of_C

# n = 160

# s,num_iter,upper_bound_of_A,upper_bound_of_C = get_bounds(n)
# print(f"n is {n}")
# print(f"s is {s}")
# print(f"number of iterations is {num_iter}")
# print(f"upper_bound_of_A is {upper_bound_of_A}")
# print(f"upper_bound_of_C is {upper_bound_of_C}")