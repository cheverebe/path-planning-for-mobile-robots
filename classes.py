# -*- coding: utf-8 -*-
import random
import numpy as np
import matplotlib
from matplotlib import collections  as mc
import matplotlib.pyplot as plt
import pylab as pl

distance_factor = .03

def primitives():
    return [(lambda pos: Position([pos.coordinates[0] + distance_factor, pos.coordinates[1]])),
            (lambda pos: Position([pos.coordinates[0] - distance_factor, pos.coordinates[1]])),
            (lambda pos: Position([pos.coordinates[0], pos.coordinates[1] + distance_factor])),
            (lambda pos: Position([pos.coordinates[0], pos.coordinates[1] - distance_factor])),
            (lambda pos: Position([pos.coordinates[0] + distance_factor, pos.coordinates[1] + distance_factor])),
            (lambda pos: Position([pos.coordinates[0] - distance_factor, pos.coordinates[1] - distance_factor])),
            (lambda pos: Position([pos.coordinates[0] - distance_factor, pos.coordinates[1] + distance_factor])),
            (lambda pos: Position([pos.coordinates[0] + distance_factor, pos.coordinates[1] - distance_factor]))]


class Target(object):
    def __init__(self, center, radius):
        c = center.as_tuple()
        self.lx = c[0]-radius
        self.hx = c[0]+radius
        self.ly = c[1]-radius
        self.hy = c[1]+radius
        
    def check(self, pos):        
        return self.lx <= pos.as_tuple()[0] and \
                pos.as_tuple()[0] <= self.hy and \
                self.hy >= pos.as_tuple()[1] and \
                pos.as_tuple()[1] >= self.ly
        
    def plot(self, ax):
        target_lines = [((self.lx, self.hy), (self.lx, self.ly)), ((self.lx, self.hy), (self.hx, self.hy)), ((self.hx, self.hy), (self.hx, self.ly)),
                        ((self.lx, self.ly), (self.hx, self.ly))]
        for line in target_lines:
            n_pos = line[0]
            p_pos = line[1]

            DATA = (n_pos, p_pos)
            (x, y) = zip(*DATA)
            color = 'y'
            ax.plot(x, y, marker='o', color=color)

class RRT(object):
    def __init__(self, initial_state, target, primitives, map):
        self.initialState = initial_state
        self.droneCount = initial_state.count()
        self.states = [initial_state]
        self.target = target
        self.primitives = primitives
        self.random_points = []
        self.map = map

    def statesInTarget(self):
        statesRes = []
        for state in self.states:
            # print(state)
            in_target = True
            # print(state.positions)
            for position in state.positions:
                in_target = in_target and self.target.check(position)
            # print(in_target)
            if in_target:
                statesRes.append(state)
        return statesRes

    def grow(self, max_iter):
        for iteration in range(max_iter):
            q_rand = self.sample_free()
            q_nearest = self.nearest(q_rand)
            q_new = self.steer(q_nearest, q_rand)
            if self.collisionFree(q_nearest, q_new) and self.formationMaintained(q_new):
                for point in q_rand:
                    self.random_points.append(point)
                self.states.append(q_new)

    def collisionFree(self, q_nearest, q_new):
        for i in range(self.droneCount):
            old_pos = q_nearest.positions[i]
            new_pos = q_new.positions[i]
            if self.map.colides(old_pos,new_pos):
                return False
        return True

    def sample_free(self):
        sample = [[random.randint(-200, 200) / 100.0, -random.randint(-200, 200) / 100.0] for i in range(self.droneCount)]
        return [Position(pos) for pos in sample]

    def nearest(self, q_rand):
        # default
        nearest = self.states[0]
        min_dist = 0
        for i in range(self.droneCount):
            position = nearest.positions[i]
            rand_position = q_rand[0]
            min_dist += position.distance(rand_position)
        # loop
        for state in self.states:
            total_dist = 0
            for i in range(self.droneCount):
                position = state.positions[i]
                rand_position = q_rand[i]
                total_dist += position.distance(rand_position)
            if total_dist < min_dist:
                min_dist = total_dist
                nearest = state
        return nearest

    def steer(self, q_nearest, q_rand):
        new_positions = []
        for i in range(self.droneCount):
            position = q_nearest.positions[i]
            best_pos = self.primitives[0](position)
            best_dist = position.distance(q_rand[i])
            for primitive in self.primitives:
                new_pos = primitive(position)
                new_dist = new_pos.distance(q_rand[i])
                if new_dist < best_dist:
                    best_dist = new_dist
                    best_pos = new_pos
            new_positions.append(best_pos)
        return State(new_positions, q_nearest)

    def plot(self):
        color_samples = ['r', 'g', 'b', 'c', 'm', 'y']
        lines = []
        colors = []
        pointsx = []
        pointsy = []

        fig, ax = plt.subplots()

        sit = self.statesInTarget()
        for state in self.states:
            parent = state.parent
            if parent:
                for i in range(self.droneCount):
                    n_pos = state.positions[i].as_tuple()
                    p_pos = parent.positions[i].as_tuple()

                    lines.append([n_pos, p_pos])
                    DATA = (n_pos, p_pos)
                    (x, y) = zip(*DATA)
                    color = 'm' if state in sit else color_samples[i % len(color_samples)]
                    ax.plot(x, y, marker='o', color=color)

        random_points_X = [point.as_tuple()[0] for point in self.random_points]
        random_points_Y = [point.as_tuple()[1] for point in self.random_points]
        random_points_color = ['k'] * len(random_points_X)
        ax.scatter(random_points_X, random_points_Y, c=random_points_color)
        self.target.plot(ax)
        self.map.plot(ax)
        plt.show()

    def get_solution(self):
        solution = []
        sit = self.statesInTarget()
        if len(sit)>0:
            state = sit[0]
            while state:
                solution.append(state)
                state = state.parent
            solution = reversed(solution)
        return solution

    def formationMaintained(self, q_new):
        distance_threshhold = .5

        pos_leader = q_new.positions[0]
        for position in q_new.positions:
            if pos_leader.distance(position) > distance_threshhold:
                return False
        return True


class State(object):
    def __init__(self, positions, parent=None):
        self.positions = positions
        self.parent = parent

    def count(self):
        return len(self.positions)

    def __str__(self):
        return ', '.join([position.__str__() for position in self.positions])


class Position(object):
    def __init__(self, coordinates):
        self.coordinates = coordinates

    def as_tuple(self):
        return (self.coordinates[0], self.coordinates[1])

    def distance(self, p2):
        a = np.array(self.as_tuple())
        b = np.array(p2.as_tuple())
        return np.linalg.norm(a - b)

    def __str__(self):
        return str(self.coordinates)

class Map(object):
    def __init__(self, obstacles):
        self.obstacles = obstacles

    def colides(self, pos_orig, pos_dest, safety_radius=.1):
        for obstacle in self.obstacles:
            if obstacle.position.distance(pos_dest) < safety_radius + obstacle.radius:
                return True
        return False

    def plot(self, ax):
        for obstacle in self.obstacles:
            obstacle.plot(ax)


class Obstacle(object):
    def __init__(self, position, radius=.3):
        self.position = position
        self.radius = radius

    def plot(self, ax):
        c = self.position.as_tuple()
        lx = c[0]-self.radius
        hx = c[0]+self.radius
        ly = c[1]-self.radius
        hy = c[1]+self.radius
        target_lines = [((lx, hy), (lx, ly)), ((lx, hy), (hx, hy)), ((hx, hy), (hx, ly)),
                        ((lx, ly), (hx, ly))]
        for line in target_lines:
            n_pos = line[0]
            p_pos = line[1]

            DATA = (n_pos, p_pos)
            (x, y) = zip(*DATA)
            color = 'k'
            ax.plot(x, y, marker='+', color=color)