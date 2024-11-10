import sys
import numpy as np
import cv2
import matplotlib.pyplot as plt

n_image = 1000

def save_Q(Q):
    global n_image
    image = np.zeros((700, 700))  # Updated to 700x700 for a 7x7 maze
    for idx in range(49):  # 7x7 = 49 states
        i = int(idx / 7)
        j = idx % 7
        v = max(Q[idx])
        z = (v + 1000) / 1500
        z = max(0, min(1, z))  # Normalize value to [0, 1]
        for k in range(100):
            for l in range(100):
                image[i * 100 + k][j * 100 + l] = z
    image = 255 * image
    img = image.astype(np.uint8)
    cv2.imshow("image", img)
    cv2.waitKey(10)
    # Uncomment for output
    # cv2.imwrite(f"pic_{n_image}.jpg", img)
    n_image += 1

def execute_action(maze, s, a):
    sp = [s[0] + a[0], s[1] + a[1]]  # Update position based on action
    isp = sp[0] * 7 + sp[1]  # Successor index
    idxs = s[0] * 7 + s[1]  # Current index

    # Check for out of bounds
    if (sp[0] < 0) or (sp[1] < 0) or (sp[0] >= 7) or (sp[1] >= 7):
        return idxs, -100, False
    if maze[sp[0]][sp[1]] == 0:
        return isp, -1, False  # Regular movement
    elif maze[sp[0]][sp[1]] == 1:
        return idxs, -100, False  # Hit a wall
    elif maze[sp[0]][sp[1]] == 10:
        return isp, 500, True  # Goal reached
    print(f"Should not happen {s} {sp} {a} {maze}")
    sys.exit(1)

def init_Q():
    Q = []
    for i in range(49):  # 7x7 = 49 states
        x = [0, 0, 0, 0]
        Q.append(x)
    return Q

def reset():
    return 0  # Starting state

def state_to_row_col(s):
    return [int(s / 7), s % 7]  # Convert index to row and column

def q_learning(maze, actions, n_episodes=300, max_episode_length=50, epsilon=0.1, alpha=0.2, gamma=0.7):
    rewards = []
    Q = init_Q()
    for e in range(n_episodes):
        s = reset()
        reward = 0
    
        for t in range(max_episode_length): 
            if np.random.uniform(0, 1) < epsilon:
                a = np.random.randint(4)  # Random action (exploration)
            else:
                a = np.argmax(Q[s])  # Best action (exploitation)

            sp, R, done = execute_action(maze, state_to_row_col(s), actions[a])
            Q[s][a] = (1 - alpha) * Q[s][a] + alpha * (R + gamma * max(Q[sp]))  # Q-value update
            reward += R
            if done:
                break
            s = sp  # Move to the new state
        rewards.append(reward)
        save_Q(Q)
    return rewards, Q

def main():
    # Define the 7x7 maze
    maze = np.array([
        [0, 0, 0, 0, 0, 0, 0],
        [1, 1, 1, 1, 1, 0, 1],
        [0, 0, 0, 0, 1, 0, 1],
        [1, 1, 1, 0, 1, 0, 1],
        [1, 0, 0, 0, 0, 0, 1],
        [1, 1, 1, 1, 0, 1, 1],
        [0, 10, 0, 0, 0, 0, 0]
    ])

    actions = [[0, 1], [1, 0], [0, -1], [-1, 0]]  # Right, Down, Left, Up

    rewards, q = q_learning(maze, actions)

    # Print rewards for each episode
    for i, r in enumerate(rewards):
        print(f"{i}, {r} \n")
    print(q)

    # Plotting the rewards as a function of episode index
    episode_indices = list(range(300))  # Assuming 300 episodes
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
