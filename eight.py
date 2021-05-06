from search import Environment, Problem, SearchAgent
import numpy as np

class EightProblem()

class EightEnvironment(Environment):
    def next_state(action, state):
        state = state.copy()
        idx = int(np.where(np.isclose(state, 0))[0])
        if action == "right":
            if idx % 3 < 2:
                tmp = state[idx+1] #[1, 0, 2, 8, 7, 6, 5, 4, 3]
                state[idx+1] = 0
                state[idx] = tmp
        elif action == "left":
            if idx % 3 > 0:
                tmp = state[idx-1] #[1, 0, 2, 8, 7, 6, 5, 4, 3]
                state[idx-1] = 0
                state[idx] = tmp
        elif action == "up":
            if idx < 3:
                tmp = state[idx-3] #[1, 0, 2, 8, 7, 6, 5, 4, 3]
                state[idx-3] = 0
                state[idx] = tmp
        elif action == "down":
            if idx > 5:
                tmp = state[idx+3] #[1, 0, 2, 8, 7, 6, 5, 4, 3]
                state[idx+3] = 0
                state[idx] = tmp
        else:
            assert False, "Action is invalid: %s"%(action)        
        return state, 1

    def __init__ (self, agent, state=None):
        super().__init__(agent)
        if state is None: #state = [1, 2, 3, 4, 5, 0, 7, 8, 6]
            self.state = np.random.choice([0, 1, 2, 3, 4, 5, 6, 7, 8], 9, False)
        else:
            self.state = state

    def next_percetion(self, action):
        if action is not None:
            self.state, _ = EightEnvironment.next_state(action, self.state)
        return self.state        
    
def h1(state): # state = [0, 1, 2, 3, 4, 5, 6, 7, 8]
    counter = 0
    for v, idx in enumerate(state):
        if v != idx:
            counter += 1
    return counter

def h2(state): # distância manhattan
    return 0

if __name__ == "__main__":
    problem = EightProblem

    print(env.state)
    state, _ = EightEnvironment.next_state("right", [0, 1, 2, 3, 4, 5, 6, 7, 8])
    env = EightEnvironment(agent, problem.initialState)
    env.run()
    print("Number of generated nodes: ", get_nb_nodes_gen())