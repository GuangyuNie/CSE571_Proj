import gym
import gym_sokoban
import time
import random
import numpy as np
import time

env_name = 'Sokoban-v0'
env = gym.make(env_name)

# 3 boxs, 3 destination 10*10 grid -> 900 states? 9 actions so (900,9)
q_table = np.zeros([900, 9]) # not sure
ACTION_LOOKUP = env.unwrapped.get_action_lookup()
print("Created environment: {}".format(env_name))


# Hyperparameters
alpha = 0.1
gamma = 0.6
epsilon = 0.1

# For plotting metrics
all_epochs = []
all_penalties = []

for i in range(1, 100001):
    start = time.time()
    state = env.reset()


    epochs, penalties, reward, = 0, 0, 0
    done = False
    
    while not done:
        if random.uniform(0, 1) < epsilon:
            action = env.action_space.sample() # Explore action space
        else:
            action = np.argmax(q_table[state]) # Exploit learned values
        #print(state) ##Something wrong with the state
        next_state, reward, done, info = env.step(action) 
        
        old_value = q_table[state, action]
        next_max = np.max(q_table[next_state])
        
        new_value = (1 - alpha) * old_value + alpha * (reward + gamma * next_max)
        q_table[state, action] = new_value

        if reward == -10:
            penalties += 1

        state = next_state
        epochs += 1
    end = time.time()
    print("Training time per eps: ", end - start)
    # Validation
    if i % 5 == 0:
        print(f"Episode: {i}")
        print(q_table) # Something wrong here
        total_epochs, total_penalties = 0, 0
        episodes = 1
        state = env.reset()
        for _ in range(episodes):
            state = env.reset()
            epochs, penalties, reward = 0, 0, 0
    
            done = False
    
            while not done:
                action = np.argmax(q_table[state])
                state, reward, done, info = env.step(action)
                if reward == -10:
                    penalties += 1

                epochs += 1

            total_penalties += penalties
            total_epochs += epochs
        state = env.reset()
        print(f"Results after {episodes} episodes:")
        print(f"Average timesteps per episode: {total_epochs / episodes}")
        print(f"Average penalties per episode: {total_penalties / episodes}")
        for t in range(100):#100
            env.render(mode='human')
            if random.uniform(0, 1) < epsilon:
                action = env.action_space.sample() # Explore action space
            else:
                action = np.argmax(q_table[state]) # Exploit learned values

            # Sleep makes the actions visible for users
            time.sleep(0.5)
            state, reward, done, info = env.step(action)
            #print(state)
            print(ACTION_LOOKUP[action], reward, done, info)
            if done:
                print("Episode finished after {} timesteps".format(t+1))
                env.render()
                break        

print("Training finished.\n")


