__author__ = 'nishaswarup'

import gym
env = gym.make('CartPole-v0')

maxi = [-1000 for _ in xrange(4)]
mini = [1000 for _ in xrange(4)]

for i_episode in range(200):
    observation = env.reset()
    for t in range(100):
        env.render()
        for i in xrange(4):
            mini[i] = min(mini[i], observation[i])
            maxi[i] = max(maxi[i], observation[i])
        action = env.action_space.sample()
        observation, reward, done, info = env.step(action)
        if done:
            print("Episode finished after {} timesteps".format(t+1))
            break

print maxi
print mini