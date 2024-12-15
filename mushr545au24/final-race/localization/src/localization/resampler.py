#!/usr/bin/env python
from __future__ import division

from threading import Lock

import numpy as np


class LowVarianceSampler:
    """Low-variance particle sampler."""

    def __init__(self, particles, weights, state_lock=None):
        """Initialize the particle sampler.

        Args:
            particles: the particles to update
            weights: the weights to update
            state_lock: guarding access to the particles and weights during update,
                since both are shared variables with other processes
        """
        self.particles = particles
        self.weights = weights
        self.state_lock = state_lock or Lock()
        self.n_particles = particles.shape[0]

        # You may want to cache some intermediate variables here for efficiency

    def resample(self):
        """Resample particles using the low-variance sampling scheme.

        Both self.particles and self.weights should be modified in-place.
        """
        # Acquire the lock that synchronizes access to the particles. This is
        # necessary because self.particles is shared by the other particle
        # filter classes.
        #
        # The with statement automatically acquires and releases the lock.
        # See the Python documentation for more information:
        # https://docs.python.org/3/library/threading.html#using-locks-conditions-and-semaphores-in-the-with-statement
        with self.state_lock:
            # BEGIN QUESTION 3.2
            "*** REPLACE THIS LINE ***"
            resampled_particles = np.zeros_like(self.particles)
            r = np.random.uniform(0, 1 / self.n_particles)
            w = self.weights[0]
            i = 0

            for m in range(self.n_particles):
                U = r + m * (1 / self.n_particles)
                while U > w:
                    i = i + 1
                    w = w + self.weights[i]
                resampled_particles[m, :] = self.particles[i, :]
            self.particles[:] = resampled_particles[:]
            self.weights[:] = np.ones(self.n_particles) / self.n_particles
            # END QUESTION 3.2
