# -*- coding: utf-8 -*-
import random
import numpy as np
import matplotlib.pyplot as plt

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
        self.target = target
        self.primitives = primitives
        self.random_points = []
        self.map = map
        self.states = [(initial_state, self.coverage(initial_state))]

    def statesInTarget(self):
        statesRes = []
        for state in self.states:
            if state[1] > 0: #has coverage
                statesRes.append(state)
        return statesRes

    def coverage(self, state):
        grid_precision = 10
        coverage_radius = .3
        total = 0
        lx = self.target.lx
        ly = self.target.ly
        hx = self.target.hx
        hy = self.target.hy
        for i in range(grid_precision):
            x = ((hx - lx) / grid_precision) * i + lx
            for j in range(grid_precision):
                y = ((hy - ly) / grid_precision) * i + ly
                point = Position([x,y])
                for position in state.positions:
                    if position.distance(point) < coverage_radius:
                        total += 1
                        break
        return float(total) / (grid_precision * grid_precision)

    def grow(self, max_iter):
        for iteration in range(max_iter):
            q_rand = self.sample_free()
            q_nearest = self.nearest(q_rand)
            q_new = self.steer(q_nearest, q_rand)
            if self.collisionFree(q_nearest, q_new) and self.formationMaintained(q_new):
                for point in q_rand:
                    self.random_points.append(point)
                self.states.append((q_new, self.coverage(q_new)))

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
        nearest = self.states[0][0]
        min_dist = 0
        for i in range(self.droneCount):
            position = nearest.positions[i]
            rand_position = q_rand[0]
            min_dist += position.distance(rand_position)
        # loop
        for state in self.states:
            state = state[0]
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

        fig, ax = plt.subplots()

        sit = [s[0] for s in self.statesInTarget()]
        for state in self.states:
            state = state[0]
            parent = state.parent
            if parent:
                for i in range(self.droneCount):
                    n_pos = state.positions[i].as_tuple()
                    p_pos = parent.positions[i].as_tuple()

                    lines.append([n_pos, p_pos])
                    DATA = (n_pos, p_pos)
                    (x, y) = zip(*DATA)
                    if state in sit:
                        color = 'm'
                    else:
                        color = color_samples[i % len(color_samples)]
                    ax.plot(x, y, marker='o', color=color)

        random_points_X = [point.as_tuple()[0] for point in self.random_points]
        random_points_Y = [point.as_tuple()[1] for point in self.random_points]
        random_points_color = ['k'] * len(random_points_X)
        ax.scatter(random_points_X, random_points_Y, c=random_points_color)
        self.target.plot(ax)
        self.map.plot(ax)
        plt.show()

    def solution(self):
        solution = []
        sit = self.statesInTarget()
        max_coverage = 0
        for state in sit:
            coverage = state[1]
            if coverage > max_coverage:
                max_coverage = coverage
                state = state[0]
                solution = []
                while state:
                    solution.append(state)
                    state = state.parent
        solution = list(reversed(solution))
        return solution

    def formationMaintained(self, q_new):
        distance_threshhold = .7
        quad_radius = .3
        if len(q_new.positions) <= 1:
            return True

        for i in range(len(q_new.positions)-1):
            for j in range(i+1,len(q_new.positions)):
                pos = q_new.positions[i]
                pos2 = q_new.positions[j]
                if pos2.distance(pos) > distance_threshhold or pos2.distance(pos) < quad_radius :
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

    def as_vrep_path_point(self):
        return "%d,%d,0.000000,0.000000,90.000000,90.000000,1.000000,15,0.500000,0.500000,0.000000,0,0.000000,0.000000,0.000000,0.000000\n" % (self.as_tuple()[0],self.as_tuple()[1])

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
    def __init__(self, position, radius=.2):
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