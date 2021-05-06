import inspect
import bisect

nb_nodes_gen = 0

def get_nb_nodes_gen():
    global nb_nodes_gen
    return nb_nodes_gen

class Node:
    def __init__(self, state, parent, action, step_cost):
        self.state = state
        self.parent = parent
        self.action = action
        self.fvalue = 0 #estimative to goal
        if parent is not None:
            self.path_cost = parent.path_cost + step_cost
        else:
            self.path_cost = step_cost
    
    def __lt__(self, other):
        return self.fvalue < other.fvalue

class Problem:
    def __init__(self, initial_state, actions, goal_test):
        self.initialState = initial_state
        self.actions = actions
        self.goalTest = goal_test
    
    def successorFunction(self, action, state):
        pass

    def expand(self, node):
        children = []
        for action in self.actions:
            state, step_cost = self.successorFunction(action, node.state)
            if state is not None:
                children.append(Node(state, node, action, step_cost))
        return children

class JarProblem(Problem):
    def __init__(self, initial_state, actions, goalTest):
        super().__init__(initial_state, actions, goalTest)

    def successorFunction(self, action, state):
        return JarEnvironment.next_state(action, state)

def checkIn(state, nodeList):
    for node in nodeList:
        if state == node.state:
            return True
    return False

def getSolutionPath(node):
    path = []
    while node.parent != None:
        path.append(node.action)
        node = node.parent
    path.reverse()
    return path

def breadth_first_search(problem):
    global nb_nodes_gen
    nb_nodes_gen = 1
    no = Node(problem.initialState, None, None, 0)
    if problem.goalTest(no.state):
        return getSolutionPath(no)
    frontier = [no]
    explored = {}
    while len(frontier) != 0:
        no = frontier.pop(0)
        children = problem.expand(no)
        nb_nodes_gen += len(children)
        explored[tuple(no.state)] = True
        for child in children:
            if problem.goalTest(child.state):
                return getSolutionPath(child)
            if not ( checkIn(child.state, frontier) or tuple(child.state) in explored ):
                frontier.append(child)
    return None #search fail


def a_star_search(problem, heuristic):
    global nb_nodes_gen
    nb_nodes_gen = 1
    if heuristic is None:
        heuristic = lambda state: 0
    no = Node(problem.initialState, None, None, 0)
    no.fvalue = heuristic(no.state)
    if problem.goalTest(no.state):
        return getSolutionPath(no)
    frontier = [no]
    explored = {}
    while len(frontier) != 0:
        no = frontier.pop(0)
        children = problem.expand(no)
        nb_nodes_gen += len(children)
        explored[tuple(no.state)] = True
        for child in children:
            if problem.goalTest(child.state):
                return getSolutionPath(child)
            if not ( checkIn(child.state, frontier) or tuple(child.state) in explored ):
                child.fvalue = child.path_cost + heuristic(child.state)
                bisect.insort(frontier, child)
    return None #search fail

class SearchAgent:
    def __init__(self, goal=None, formulateProblem=None, search=None, heuristic=None):
        self.seq = []
        self.goal = goal
        self.formulateProblem = formulateProblem
        self.problem = None
        self.heuristic = heuristic
        if search is None:
            self.search = breadth_first_search
        else:
            self.search = search

    def state_update(self, perception):
        return perception

    def formulateGoal(self, state):
        if inspect.isfunction(self.goal):
            if self.goal(state):
                return None
            else:
                return self.goal
        else:
            if state == self.goal:
                return None
            else:
                return self.goal

    def act(self, perception):
        state = self.state_update(perception)
        if len(self.seq) == 0:
            goal = self.formulateGoal(state)
            if goal is None:
                return None
            self.problem = self.formulateProblem(state, goal)
            if self.heuristic is None:
                self.seq = self.search(self.problem)
            else:
                self.seq = self.search(self.problem, self.heuristic)
            if self.seq is None:
                self.seq = []
        if not len(self.seq):
            return None
        return self.seq.pop(0)

class Environment:
    def __init__(self, agent):
        self.agent = agent
        self.last_action = None

    def render(self, perception):
        print("Action: ", self.last_action)
        if perception is not None:
            print("Perception: ", perception)
    
    def step(self, perception):
        self.last_action = self.agent.act(perception)
        if self.last_action is None:
            return False
        return True
    
    def next_percetion(self, action):
        return None

    def run(self):
        perception = self.next_perception(None)
        self.render(perception)
        while self.step(perception):
            perception = self.next_perception(self.last_action)
            self.render(perception)

class JarEnvironment(Environment):
    def __init__(self, agent):
        super().__init__(agent)
        self.state = [0, 0]

    def next_state(action, state):
        if action == "EA":
            return (0, state[1]), 1
        elif action == "EB":
            return (state[0], 0), 1
        elif action == "ENA":
            return (3, state[1]), 1
        elif action == "ENB":
            return (state[0], 4), 1
        elif action == "TAB":
            c = 4 - state[1]
            r = max(0, state[0] - c)
            t = min(4, state[1] + state[0])
            return (r, t), 1
        elif action == "TBA":
            c = 3-state[0]
            r = max(0, state[1] - c)
            t = min(3, state[0] + state[1])
            return (r, t), 1
        else:
            assert False, "Invalid Action: {}".format(action)
        return None, 0
    
    def next_perception(self, action):
        if action is not None:
            self.state, cost = JarEnvironment.next_state(action, self.state)
        return self.state

if __name__ == '__main__':
    goal = lambda state: state[0] == 2 or state[1] == 2
    formulateProblem = lambda fromState, toGoal: JarProblem(fromState, 
        ['EA', 'EB', 'ENA', 'ENB', 'TAB', 'TBA'], toGoal)
    agent = SearchAgent(goal, formulateProblem)
    environment = JarEnvironment(agent)
    environment.run()
