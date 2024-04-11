import random
import numpy as np
import math
from environment_stage_1 import Env

env = Env()

class ReplayBufferEnergy:
    def __init__(self, capacity):
        self.capacity = capacity
        self.trajectory_buffer = []
        self.position = 0
        self.weights = np.ones(len(self.trajectory_buffer))

    def __len__(self):
        return len(self.trajectory_buffer)

    def push(self, trajectory):
        if len(trajectory) == 0:
            return
        if len(self.trajectory_buffer) < self.capacity:
            self.trajectory_buffer.append(None)
        self.trajectory_buffer[self.position] = trajectory
        self.position = (self.position + 1) % self.capacity

    def sample(self, batch_size):
        probs = [t[1] for t in self.trajectory_buffer]
        # total_probs = sum(probs)
        # probs = [p / total_probs for p in probs]

        sample_index = random.choices(range(len(self.trajectory_buffer)), weights=probs, k=1)[0]
        #print("sample_traj:", sample_index)
        sample_traj = self.trajectory_buffer[sample_index][0]

        batch = random.sample(sample_traj, batch_size if len(sample_traj) > batch_size else len(sample_traj))
        self.trajectory_buffer[0][1] -= 0.001  # decrease the priority

        state, action, reward, next_state, done = map(np.stack, zip(*batch))
        return state, action, reward, next_state, done  # normalized_weights


    def calculate_traj_energy(self, episode):
        episode_length = len(episode)
        m, inertia = 1, 1  # inertia
        episode_energy_list = np.zeros((episode_length))

        for t in range(episode_length):
            #print("shape", len(episode))
            velocity = episode[t][1][0]
            #print("velocity:", velocity)
            angular_velocity = episode[t][1][1]
            #print("angular_velocity:", angular_velocity)
            # kinetic_energy
            kinetic_energy = 0.5 * m * np.power(velocity, 2)

            # rotational_energy
            rotational_energy = 0.5 * inertia * np.power(angular_velocity, 2)

            transition_energy = kinetic_energy + rotational_energy
            episode_energy_list[t] = transition_energy

        energy_diff = np.diff(episode_energy_list, axis=0)  # energy difference
        # clip
        for i in range(len(energy_diff)):
            energy_diff[i] = energy_diff[i] if energy_diff[i] > 0 else 0

        # print("energy_ori:", np.sum(energy_diff, axis=0))

        energy = np.sum(energy_diff, axis=0) / 50
        trajectory_energy = energy
        # print("energy:", energy)
        return trajectory_energy

    # def self_paced_prioritized_curriculum(self, trajectory_energy):
    #     delta = trajectory_energy
    #     priority = 1.0
    #     lam = 0.5
    #     if 0 < delta <= lam:
    #         priority = 1.0
    #     elif lam < delta and delta < 1.0:
    #         priority = 0
    #     return priority

    # def self_paced_prioritized_curriculum(self, trajectory_energy):
    #     delta = trajectory_energy
    #     priority = lam1 = 1.0
    #     lam2 = 0.5
    #     psi = lam1 * lam2 / (lam1 - lam2)
    #     if 0 < delta <= lam2:
    #         priority = 1.0
    #     elif delta >= lam1:
    #         priority = 0
    #     else:
    #         priority = psi * (1/delta - 1/lam1)
    #     return priority

    def self_paced_prioritized_curriculum(self, trajectory_energy, lamada):

        delta = trajectory_energy
        priority = 1.0
        if 0 < delta <= lamada:
            priority = math.exp(delta - lamada)
        elif lamada < delta and delta < 2 * lamada:
            priority = (math.log(delta - 2 * lamada + 1)) / (math.log(1 - lamada))
        elif delta >= 2 * lamada and delta < 0.05:
            priority = 0.
        #print ("priority:", priority)
        return priority