import math as m
"""
The struct in the C file will be 
typedef struct = {
    int x;
    int y;
} Point;

and the print from this file must be

Point lookup_table[173] = {
    {x0,y0}, {x1,y1}, {x2,y2}, ..
}

This will make it so I can paste it straight into the file
"""



lookup_table = [] # needs to store (x,y) tuples

degree_step = 1.04
degree_step_in_rad = degree_step * m.pi/180

def calculate_x(step_count, step_degree):
    x2 = 64 - (62*m.cos(step_degree*step_count))
    return round(x2,2)

def calculate_y(step_count, step_degree):
    y2 = 64 - (62*m.sin(step_degree*step_count))
    return round(y2,2)

for i in range(174): # want to start at 0 and then end at 173 since 173*1.04 = about 180
    x = calculate_x(i, degree_step_in_rad)
    y = calculate_y(i, degree_step_in_rad)
    lookup_table.append((x,y))

for count, x in enumerate(lookup_table):
    print(f"{{{x[0]},{x[1]}}}, ", end="")
    if count == len(lookup_table)-1:
        print(count)