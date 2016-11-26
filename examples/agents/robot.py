import random
import math
import numpy as np
import gym
import sys

""" range of observations
[1.8303681833507139, 3.3895342884303021, 0.20879056797820722, 2.7872154423606679]
[-0.34487815869867666, -2.4511489979372088, -0.20892458535656977, -2.5936806785031399]
"""


# Set-up
env = gym.make('CartPole-v0')
outdir = sys.argv[1]

basis = 3
# initial_epsilon = 0  # probability of choosing a random action (changed from original value of 0.0)
alpha = 0.001  # learning rate
lambda_ = 0.9  # trace decay rate
gamma = 1.0  # discount rate

M = env.action_space.n
N = 2 * (3 + 1)**5

# TODO: Normalize the state variables from -1 to 1


def get_c(goal, choices, current=0, storage=[]):
    """
    :param goal: the desired length of vectors
    :param choices: the number possible choices for each entry in the vector
    :param current: the current length of vectors
    :param storage: the list of currently developed vectors

    :return: a list of distinct vectors of size "goal" with values chosen from the
    range [0, choices).
    """
    if current == goal:
        return storage
    elif current == 0:
        new_storage = [[x] for x in xrange(choices)]
    else:
        new_storage = [x+[y] for y in xrange(choices) for x in storage]
    return get_c(goal, choices, current+1, new_storage)


def q_value(s, a, c, theta):
    """
    :param s: list of state variables
    :param a: variable defining action (0 or 1 for L or R)
    :param c: list of N/2 vectors of dimension 5
    :param theta: vector of dimension N

    :return: q value of action
    """
    x = np.append(s, [a])
    val = 0
    for i in xrange(N/2):
        cos = theta[2*i] * math.cos(math.pi * np.dot(c[i], x))
        sin = theta[2*i+1] * math.sin(math.pi * np.dot(c[i], x))
        val += cos + sin
    return val

def normalize(s):
    s[0] /= 3.
    s[1] /= 4.
    s[2] /= 0.21
    s[3] /= 3.

    return s

def best_action(s, c, theta):
    """
    :param s: list of state variables
    :param c: list of N/2 vectors of dimension 5
    :param theta: vector of dimension N

    :return: the action (either 0 or 1) with the highest q score.
    """
    left_val = q_value(s, 0, c, theta)
    right_val = q_value(s, 1, c, theta)
    if left_val > right_val:
        return 0
    elif left_val == right_val and random.random() < 0.5:
        return 0
    else:
        return 1


def best_q(s, c, theta):
    """
    :param s: list of state variables
    :param c: list of N/2 vectors of dimension 5
    :param theta: vector of dimension N

    :return: the highest q score possible given the possible actions.
    """
    left_val = q_value(s, 0, c, theta)
    right_val = q_value(s, 1, c, theta)
    return max(left_val, right_val)


def update_coef(new_s, new_a, s, a, c, theta, r):
    """
    :param new_s: list of new state variables
    :param new_a: best action available from new_s
    :param s: list of old state variables
    :param a: variable defining action taken to get from s to new_s
    :param c: list of N/2 vectors of dimension 5
    :param theta: vector of dimension N
    :param r: reward given

    :return: the updated theta vector
    """
    x = np.append(s, [a])
    theta_new = np.zeros(N)

    temp1 = q_value(new_s, new_a, c, theta)
    temp2 = q_value(s, a, c, theta)
    for i in xrange(N):
        if i % 2 == 0:
            deriv = -theta[i] * math.sin(math.pi * np.dot(c[i/2], x))
        else:
            deriv = theta[i] * math.cos(math.pi * np.dot(c[i/2], x))

        theta_new[i] = theta[i] + alpha * (r + gamma * temp1 - temp2) * deriv
        if theta_new[i] > 1024:
            theta_new[i] = 1024

    return theta_new


def main():
    env.monitor.start(outdir, force=True)
    print "start environment"

    # epsilon = initial_epsilon
    theta = [random.random() for _ in xrange(N)]
    c = get_c(5, basis+1)
    print "setup done"

    # Run 2000 episodes
    for episode_num in xrange(2000):
        print "begin episode:", episode_num
        # print episode_num, episode(epsilon, theta, env.spec.timestep_limit)
        s = normalize(env.reset())
        action = best_action(s, c, theta)
        total_reward = 0
        done = False
        while not done:
            new_s, reward, done, _ = env.step(action)
            new_s = normalize(new_s)
            new_a = best_action(new_s, c, theta)
            theta = update_coef(new_s, new_a, s, action, c, theta, reward)
            total_reward += reward
            s = new_s
            action = new_a

        print "end episode:", episode_num, total_reward

    env.monitor.close()

main()
