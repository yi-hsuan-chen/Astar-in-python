from tkinter import messagebox, Tk
import networkx as nx
import numpy as np
import pygame

MAX_WEIGHT = 1000000

# User defined parameters
n               = 20        # nxn grid
window_width    = 600
window_height   = 600

# Pygame initialization
pygame.init()
window          = pygame.display.set_mode((window_width,window_height))
clock           = pygame.time.Clock()
box_width       = window_width // n
box_height      = window_height // n
G               = nx.grid_2d_graph(n,n)

# set start and goal nodes position
start_pos       = (1, 1)
start_idx       = start_pos[0]*n+start_pos[1]+1

# Set Node index from 1
idx     = 1
for node in G.nodes:
    G.nodes[node]['idx']    = idx
    G.nodes[node]['g']      = 0
    G.nodes[node]['h']      = 0
    G.nodes[node]['f']      = 0
    G.nodes[node]['parent'] = None
    G.nodes[node]['visited']= False
    if idx == start_idx:
        start_node = node
    idx = idx+1

# Set all weights to 1
for edge in G.edges:
    G.edges[edge]['weight'] = 1
# Add diagonal edges to make 8-connected grid
G.add_edges_from([
    ((x, y), (x+1, y+1))
    for x in range(n-1)
    for y in range(n-1)
] + [
    ((x+1, y), (x, y+1))
    for x in range(n-1)
    for y in range(n-1)
], weight=1.414)


def backTrace(G, closed):
    start_node      = closed[0]
    goal_node       = closed[-1]
    path            = [goal_node]
    while path[-1] != start_node:
        next_node       = G.nodes[path[-1]]['parent']
        path.append(next_node)
    path.reverse()
    return path


def draw(node,win,color):
    y,x = node[0],node[1]
    pygame.draw.rect(win,color,(x*box_width, y*box_height, box_width-2, box_height-2))


## ============================== Main function ==============================
def main():
    begin_search    = False
    reachGoal       = False
    set_goal        = False
    goal_node       = None
    obs_list        = []
    # Initialize priority queue and visited list
    Q       = []
    VISITED = []            # visited list
    path    = []
    Q.append(start_node)
    

    while 1:
        for event in pygame.event.get():
            # Quit Windows
            if event.type == pygame.QUIT:
                pygame.quit() 
            elif event.type == pygame.MOUSEMOTION:
                y   = pygame.mouse.get_pos()[0]
                x   = pygame.mouse.get_pos()[1]
                i   = x // box_width
                j   = y // box_height
                # Define obstacles using the left mouse button
                if event.buttons[0]:
                    obs_idx     = i*n+j+1
                    if obs_idx not in obs_list:
                        obs_node = [n for n,v in G.nodes(data=True) if v['idx'] == obs_idx]
                        obs_list.append(*obs_node)
                # Set and draw Goal
                if event.buttons[2] and not set_goal:
                    goal_pos    = (x, y)
                    goal_idx    = i*n+j+1
                    goal_node   = [n for n,v in G.nodes(data=True) if v['idx'] == goal_idx]
                    goal_node   = goal_node[0]
                    set_goal    = True

            # Start Search
            if event.type == pygame.KEYDOWN and set_goal:
                begin_search = True
                print('Searching...')

        ## A* path finding starts here...
        if begin_search and not reachGoal:
            if len(Q) > 0:
                # print('Popping...')
                current_node    = Q.pop(0)
                G.nodes[current_node]['visited'] = True
                VISITED.append(current_node)
                if current_node == goal_node:
                    print('Goal!')
                    path        = backTrace(G, VISITED)
                    reachGoal   = True
                neighbors       = G.neighbors(current_node)
                for neighbor in neighbors:
                    if neighbor not in obs_list:
                        current_g   = G.nodes[neighbor]['g']
                        new_g       = G.nodes[current_node]['g']+ G.get_edge_data(current_node,neighbor)['weight']
                        if not G.nodes[neighbor]['visited'] or current_g > new_g:
                            g           = new_g
                            x_dis2goal  = int(neighbor[0])-int(goal_pos[0])
                            y_dis2goal  = int(neighbor[1])-int(goal_pos[1])
                            h           = np.linalg.norm((x_dis2goal,y_dis2goal), 2)                            
                            f           = h+g

                            G.nodes[neighbor]['visited'] = True
                            G.nodes[neighbor]['parent'] = current_node
                            G.nodes[neighbor]['g'] = g
                            G.nodes[neighbor]['f'] = f
                            Q.append(neighbor)
                            Q   = sorted(Q, key=lambda n: G.nodes[n]['f'])
            else:
                if not reachGoal:
                    Tk().wm_withdraw() # to hide the main window
                    messagebox.showinfo("NO solution")

        window.fill((0,0,0))

        # Draw grids
        for node in G:
            draw(node,window,'lightgreen')
            # if G.nodes[node]['queued']:
            #     draw(node,window,'green')
            if node in VISITED:
                draw(node,window,'lightpink')
            if node in path[1:-1]:
                draw(node,window,'blue')
            if node == start_node:
                draw(node,window,'cyan')
            if node == goal_node:
                draw(node,window,'yellow')
            if node in obs_list:
                draw(node,window,'black')
        pygame.display.update()

main()
