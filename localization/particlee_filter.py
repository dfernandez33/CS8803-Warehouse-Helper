import numpy as np

class Particle:
    def __init__(self, coord, prob):
        self.coord = coord
        self.prob = prob
    
    def update_prob(self, obs_prob):
        self.prob *= obs_prob


def particleFilter(num_particles=1000):
    particles = []

    for _ in range(num_particles):
        rand_x = np.randint(0, 2921)
        rand_y = np.randint(0, 2057)
        particles.append(Particle(np.array([rand_x, rand_y]), 1.0))
    
