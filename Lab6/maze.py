import sys
import numpy as np
import cv2

n_image = 1000
def save_Q(Q):
    global n_image
#   print(Q)
    image = np.zeros((500,500))
    for idx in range(25):
        i = int(idx / 5)
        j = idx % 5
        v = max(Q[idx])
        z = (v + 1000) / 1500
        if z < 0:
            z = 0
        if z > 1:
            z = 1
        for k in range(100):
            for l in range(100):
                image[i*100+k][j*100+l] = z
    image = 255 * image 
    img = image.astype(np.uint8)
    cv2.imshow("image", img)
    cv2.waitKey(10)
#   uncomment for output
#   cv2.imwrite(f"pic_{n_image}.jpg", img)
    n_image = n_image + 1


def execute_action(maze, s, a):
    sp = [0,0]
    sp[0] = s[0] + a[0]
    sp[1] = s[1] + a[1]

    isp = sp[0] * 5 + sp[1]
    idxs = s[0] * 5 + s[1]

    if (sp[0] < 0) or (sp[1]<0) or (sp[0] >= 5) or (sp[1] >= 5):
        return idxs, -100, False
    if maze[sp[0]][sp[1]] == 0:
        return isp, -1, False
    elif maze[sp[0]][sp[1]] == 1:
        return idxs, -100, False
    elif maze[sp[0]][sp[1]] == 10:
        return isp, 500, True
    print(f"Should not happen {s} {sp} {a} {maze}")
    sys.exit(1)

#
#  Q[state, action]  state is row major version of the 5x5 map, actions are [0,1], [1,0], [0,-1], [-1, 0] 
#  in row col
#
def init_Q():
    Q = []
    for i in range(25):
        x = [0, 0, 0, 0]
        Q.append(x)
    return Q

def reset():
     return 0

def state_to_row_col(s):
     return [int(s / 5), s % 5]

def q_learning(maze, actions, n_episodes=300, max_episode_length=50, epsilon=0.1, alpha=0.2, gamma=0.7):
    rewards = []

    Q = init_Q()
    for e in range(n_episodes):
        s = reset()
        reward = 0
    
        for t in range(max_episode_length): 
            if np.random.uniform(0,1) < epsilon:
                a = np.random.randint(4)
            else:
                a = np.argmax(Q[s])

            sp, R, done = execute_action(maze, state_to_row_col(s), actions[a])
            Q[s][a] = (1 - alpha) * Q[s][a] + alpha * (R + gamma * max(Q[sp]))
            reward = reward + R
            if done:
                break
            s = sp
        rewards.append(reward)
        save_Q(Q)
    return rewards, Q

    
#
# define the maze 0 = unoocuped, 1 = wall, 10 = goal
#   

def main():
    maze = np.array([
        [0, 0, 0, 0, 0],
        [1, 1, 1, 1, 0],
        [1, 0, 0, 0, 0],
        [1, 1, 1, 1, 0],
        [1, 10, 0, 0, 0]])

    actions = [[0,1], [1,0], [0,-1], [-1, 0]]

    rewards, q = q_learning(maze, actions)
    for i,r in enumerate(rewards):
        print(f"{i}, {r} \n")
    print(q)

    import matplotlib.pyplot as plt

    # Sample data for rewards and episode steps
    # This data can be replaced with the actual rewards data from the q_learning function.
    episode_indices = list(range(300))  # Assuming 300 episodes
    rewards = [0, -1, 1, 2, 5, -2, 10, 0, -1, 3] * 30  # Placeholder for actual rewards data

    # Plotting the rewards as a function of episode index
    plt.figure(figsize=(12, 6))
    plt.plot(episode_indices, rewards, marker='o', linestyle='-', color='b')
    plt.title('Total Reward per Episode in Q-Learning')
    plt.xlabel('Episode Index')
    plt.ylabel('Total Reward')
    plt.grid()
    plt.show()


    cv2.waitKey(0)


if __name__ == "__main__":
    main()