ZONE_MAP = [
    [40, 41, 42, 43, 44, 45, 46, 47],
    [32, 33, -1, 35, -1, 37, 38, -1],
    [24, 25, -1, 27, 28, 29, 30, -1],
    [16, -1, -1, 19, 20, 21, 22, -1],
    [ 8,  9, 10, 11, 12, 13, 14, 15],
    [ 0,  1, -1,  3, -1, -1,  6,  7]
]

PDDL_FOLDER = "/home/etdisc/Bureau/catkin_ws/src/ai-robotics-lab/pddl/"

OOIs = {'o1': 46, 'o2': 35}

def generate_problem(problem, zone_map, objects, robot_position):
    pb = "(define (problem {}) (:domain navigation)\n".format(problem)
    # Objects
    pb += "(:objects\n"
    pb += "\tr - robot\n"
    for row in zone_map:
        for cell in row:
            if cell > -1:
                pb += "\twp_{} - waypoint\n".format(cell)
    for o in objects:
        pb += "\t{} - object_of_interest\n".format(o)
    pb += ")\n"
    # Init
    pb += "(:init\n"
    pb +="\t(at r wp_{})\n".format(robot_position)
    for o, w in objects.items():
        pb += "\t(location {} wp_{})\n".format(o, w)
    for i in range(len(zone_map)):
        for j in range(len(zone_map[i])):
            w = zone_map[i][j]
            if w < 0: 
                # obstacle
                continue
            if i > 0:
                up = zone_map[i-1][j]
                if up > -1:
                    pb += "\t(adjacent wp_{} wp_{})\n".format(w, up)
            if i < len(zone_map) - 1:
                down = zone_map[i+1][j]
                if down > -1:
                    pb += "\t(adjacent wp_{} wp_{})\n".format(w, down)
            if j > 0:
                left = zone_map[i][j-1]
                if left > -1:
                    pb += "\t(adjacent wp_{} wp_{})\n".format(w, left)
            if j < len(zone_map[i]) - 1:
                right = zone_map[i][j+1]
                if right > -1:
                    pb += "\t(adjacent wp_{} wp_{})\n".format(w, right)
    pb += ")\n"
    # Goal
    pb += "(:goal\n\t(and\n"
    for o in objects:
        pb += "\t(observed {})\n".format(o)
    pb += "\t)\n)\n)"
    return pb

def write_problem(filename, problem):
    f = open(filename, "w")
    f.write(problem)
    f.close()

my_problem = generate_problem('navpb1', ZONE_MAP, OOIs, 11)

write_problem(PDDL_FOLDER + 'my_nav_pb.pddl', my_problem)
