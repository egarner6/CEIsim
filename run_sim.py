"""
Simulation of active brownian shape-shifting agents in hallway navigation
experiment

Developed by Alli Nilles, started from pymunk bouncing balls example

"""

import random
import sys, os

import csv      # import library to write to a CSV


# For writng video games, allows you to create fully featured games and multimedia programs
import pygame
import pygame.camera

# 2d physics library that can be used whenever you need 2d rigid body physics from Python
import pymunk
import pymunk.pygame_util
from pymunk import Vec2d
import numpy as np

from utils import *

# Number of 'bodies' in the simulation
N_agents = 20


# timing for sim
FPS = 30. # FPS of animation
delay = 10 # number of steps to run between frames
dt = 1.0 / (FPS*delay) # 1/300 = 0.003333
total_time = 10. # seconds
morph_time = 5./dt
char_T = int(1./dt) # characteristic time between noise events

# environment parameters
rect_size = 30 # dim of shorter rectangle side, pixels
esize = 20*rect_size
pixpermeter = rect_size/0.1 # approximate rectangle width as 5cm
f = 1.4 # ratio of hallway to rect_size
theta = 135 # angle of room into hallway

# friction constants
muF = 0.1 # coefficient of friction of floor
muFwall = 0.5 # coefficient of friction of walls
muFhub = 0.2 # coefficient of friction of hubs

# bias force from gravity
tilt = 3.4 # degrees
mass = 0.09 # kg
g = pixpermeter*9.81 # meters / s**2
acc = (g*np.sin(np.radians(tilt)) - muF)
F = mass*acc


# parameters for active brownian model
D = 0.05 # noise intensity

# open a file to write to, create writer, write to it
simdata =  open('C:/Users/lizga/Documents/CEIsim/CEIsim.csv', 'w')
writer = csv.writer(simdata)
writer.writerow(['Testing that this writes', '2nd square'])

class WildBodies:
    def __init__(self, ID):
        pygame.sprite.Sprite.__init__(self)
        self.ID = ID
        self.x = random.randint(int(0.2*esize), int(0.5*esize))
        self.y = random.randint(200, int(1.3*esize)-200)
        self.shape, self.body, self.inertia = genHex(mass)
        vel = gen_rand_vec()
        self.body.velocity = vel[0], vel[1]
        self.shape.friction = muFhub
        # self.shape.add(self.body, self.shape)       # UNSURE OF THIS LINE
        # bots[ID] = self.shape                       # UNSURE OF THIS LINE

# called when two shapes start touching for the first time
# TODO place actual "sensor" objects
def start_collision(arbiter, space, data):
    (s1, s2) = arbiter.shapes
    foundOne = False       # Change out these to be in start collision (robot vs wall vs movable object)
    foundBoth = False

    log_collision(s1, s2, data, 1, foundOne, foundBoth)
    print("Elapsed Time =", pygame.time.get_ticks(), " millisec")
    
    return True


def log_collision(s1, s2, data, val, foundOne, foundBoth):
    # use for short circuit in case we scale up
    
    # 9.2 Python documnetation, make a class where we create an instance of the object class (polygon) with type or ID?
    # Or have a global list/dictionary with type of an environment or robot, with ID if robot
    if isinstance(s1, pymunk.shapes.Poly) and isinstance(s2, pymunk.shapes.Poly):
        for id, s in data["bots"].items():          # looping through all the bots with IDs and shapes
            if foundBoth:
                return True
            if s1 == s:     # Checking 
                data["contacts"][id] += val
                if foundOne == False:
                    foundOne = True
                else:
                    foundBoth = True
            if s2 == s:
                data["contacts"][id] += val
                if foundOne == False:
                    foundOne = True
                else:
                    foundBoth = True
    writer.writerow([s1, s2, data])         # print a timestamp, need global time, s1 and s2 are not meaningful to wrte or data
    #next, work on logging x and y and then creating global time, make two different csvs probably
    # writer.writerow([body.position, pygame.time.get_ticks()])
    return True

# draw circle proportional to collision overlap
def draw_collision(arbiter, space, data):
    for c in arbiter.contact_point_set.points:
        r = max(3, abs(c.distance * 5))
        r = int(r)

        p = pymunk.pygame_util.to_pygame(c.point_a, data["surface"])
        pygame.draw.circle(data["surface"], pygame.Color("black"), p, r, 1)

def stop_collision(arbiter, space, data):
    (s1, s2) = arbiter.shapes

    log_collision(s1, s2, data, -1)



def genHex(mass):

    # define regular hexagon with unit side length
    h = np.sin(np.radians(60))
    vs = [(0., 0.), (1., 0.), (1.5, h), (1., 2*h), (0., 2*h), (-0.5, h)]
    poly1 = pymunk.Poly(None, vs)

    # shift to origin, scale up
    COM = poly1.center_of_gravity
    x, y = -COM.x, -COM.y
    t = pymunk.Transform.identity().translated(x, y).scaling(rect_size)
    poly2 = pymunk.Poly(None, vs, transform=t)

    # add inertia and body
    inertia = pymunk.moment_for_poly(mass, poly2.get_vertices())
    body = pymunk.Body(mass, inertia)
    poly_good = pymunk.Poly(body, poly2.get_vertices(), radius = 0.002*rect_size)

    return poly_good, body, inertia

def genAsymmetricHexVerts(COM, angle, s):
    assert((s<=1.) and (s>=0.))
    phi = np.radians(120.+s*60.)
    theta = np.pi-0.5*phi
    xp = np.cos(np.pi-theta)
    yp = np.sin(np.pi-theta)
    vs = [(0., 0.), (1., 0.), (1+xp, yp), (1., 2*yp), (0., 2*yp), (0.-xp, yp)]
    poly1 = pymunk.Poly(None, vs)

    # shift to origin, scale up
    COM = poly1.center_of_gravity
    x, y = -COM.x, -COM.y
    t = pymunk.Transform.identity().translated(x, y).scaling(rect_size)
    poly2 = pymunk.Poly(None, vs, transform=t)
    inertia_new = pymunk.moment_for_poly(mass, poly2.get_vertices())
    body_new = pymunk.Body(mass, inertia_new)
    poly_new = pymunk.Poly(body_new, poly2.get_vertices(), radius = 0.002*rect_size)

    return poly_new, body_new, inertia_new

def morph(space, shape, body, s):

    angle = body.angle
    mass = body.mass

    shape_new, body_new, inertia_new = genAsymmetricHexVerts(shape.center_of_gravity, angle, s)
    #print("Old:", shape.get_vertices())
    #print("New:", shape_new.get_vertices())


    body_new.position = body.position
    body_new.velocity = body.velocity
    body_new.angle = body.angle
    body_new.force = body.force
    shape_new.friction = muFhub

    return shape_new

# generate random vector on unit circle
def gen_rand_vec():
    x = np.random.standard_normal(2)
    return x / np.linalg.norm(x)


def main():

    steps = total_time/dt
    vdot = [0.,0.]
    print(f"external force is {acc*mass} N")

    pygame.init()
    pygame.camera.init()
    height = int(1.3*esize)
    width = esize
    screen = pygame.display.set_mode((width, height))
    clock = pygame.time.Clock()
    running = True

    ### Physics stuff
    space = pymunk.Space()
    draw_options = pymunk.pygame_util.DrawOptions(screen)
    # disable the build in debug draw of collision point since we use our own code.
    draw_options.flags = (
        draw_options.flags ^ pymunk.pygame_util.DrawOptions.DRAW_COLLISION_POINTS
    )
    ## Balls
    bots = {i:None for i in range(N_agents)}

    ### Environment parameters
    #
    space.gravity = (acc, 0.0)
    static_lines = genEnv(space, theta, f, rect_size, height, width)
    for l in static_lines:
        l.friction = muFwall

    space.add(*static_lines)


    # spawn bots
    for i in range(N_agents):
        # x = random.randint(int(0.2*width), int(0.5*width))
        # y = random.randint(200, height-200)
        # shape, body, inertia = genHex(mass)
        # body.position = x, y
        # vel = gen_rand_vec()
        # body.velocity = vel[0], vel[1]
        # shape.friction = muFhub
        # space.add(body, shape)
        # bots[i] = shape

        # have the class hold all the agents, not an array with all of them which is currently bots[]
        # eventually have a class for the simulation, doesnt matter right now where initialization is
        # call WildBodies once and send it the number of bots you want
        # print current time and number of robots, unmber of walls youre in contact with in either class or somewhere else
        WildBodies(i)
        # bots[i].shape.add(bots[i].body, bots[i].shape)       # UNSURE OF THIS LINE
        # bots[i] = bots[i].shape                              # UNSURE OF THIS LINE
        outputStr = "bot spawned at (" + str(bots[i].x) + ", " + str(bots[i].y) + ")"
        writer.writerow(outputStr)

    # make dictionary for data logging
    contacts = {i:0 for i,s in bots.items()}
    # 0 = hex, 1 = rect
    bot_states = {i:0 for i,s in bots.items()}
    bot_s = {i:0. for i,s in bots.items()}
    forces = {i:list(gen_rand_vec()) for i,s in bots.items()}

    # collision handler
    ch = space.add_collision_handler(0, 0)
    ch.data["surface"] = screen
    ch.data["contacts"] = contacts
    ch.data["bots"] = bots
    ch.data["forces"] = forces
    ch.begin = start_collision
    ch.post_solve = draw_collision
    ch.separate = stop_collision


    framenum = 0
    try:
        os.makedirs("frames")
    except OSError:
        pass

    while steps > 0:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                running = False
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_p:
                pygame.image.save(screen, "contact_with_friction.png")

        fname = "frames/%04d.png" % framenum
        pygame.image.save(screen, fname)
        framenum += 1
        ### Clear screen
        screen.fill(pygame.Color("white"))

        ### Draw stuff
        space.debug_draw(draw_options)

        bots_to_remove = []
        for i,bot in ch.data["bots"].items():
            if bot.body.position.x > width:
                bots_to_remove.append((i,bot))
        for i,bot in bots_to_remove:
            space.remove(bot, bot.body)
            del ch.data["bots"][i]

        ### Update physics
        # this extra loop allows us to run physics at small dt and
        # draw and check other events more infrequently
        for x in range(delay):

            morph_list = []
            for id, bot in ch.data["bots"].items():
                # reset force DIRECTION every char_T timesteps
                # keep applying force at every timestep
                differential_force = list(dt*np.array(forces[id]))
                bot.body.apply_force_at_local_point(differential_force)
                if random.random() < 1./char_T:

                    # Brownian motion, random step on unit circle
                    xi = pixpermeter*gen_rand_vec()
                    vdot = list(D*xi)
                    #print(f"Actual applied force is {bot.body.force}")
                    #print(f"Random force for bot {id} is {vdot}")
                    forces[id] = vdot
                    bot.body.force = [0,0]
                    bot.body.apply_impulse_at_local_point(forces[id])

                if ch.data["contacts"][id] > 1 and bot_states[id] == 0:
                    bot_states[id] = 1
                # do not morph every time step to allow for motion as well
                if bot_states[id] == 1. and (steps % delay == 0) and bot_s[id] != 1.:
                    s = min(1.0, bot_s[id] + (1.0*delay)/morph_time)
                    bot_s[id] = s

                    bot_new = morph(space, bot, bot.body, s)
                    displacement = abs(bot_new.body.position - bot.body.position)
                    if displacement > rect_size:
                        raise(ValueError, "too much displacement")
                        steps = 0
                        print("too much displacement!")
                    else:
                        morph_list.append((id, bot, bot_new))


            for id, bot, bot_new in morph_list:
                space.remove(bot, bot.body)
                space.add(bot_new, bot_new.body)
                ch.data["bots"][id] = bot_new

            space.step(dt)
            steps -= 1


        ### Flip screen
        pygame.display.flip()
        clock.tick(FPS)
        pygame.display.set_caption("fps: " + str(clock.get_fps()))

    # render video
    cmd = str(f"ffmpeg -r {FPS} -f image2 -i frames/%04d.png -y -qscale 0 -s {width}x{height} result.avi")
    os.system(cmd)
    


if __name__ == "__main__":
    sys.exit(main())
   
