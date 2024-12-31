import util
from util import Queue, PriorityQueue

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).
    """
    def getStartState(self):
        """Returns the start state for the search problem."""
        util.raiseNotDefined()

    def isGoalState(self, state):
        """Returns True if and only if the state is a valid goal state."""
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """Returns a list of successors (state, action, stepCost)."""
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """Returns the total cost of a sequence of actions."""
        util.raiseNotDefined()

def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze. For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return [s, s, w, s, w, w, s, w]

def generalGraphSearch(problem, frontier):
    """
    Generalized graph search algorithm to reduce redundancy in DFS, BFS, UCS, and A*.
    """
    startState = problem.getStartState()
    if problem.isGoalState(startState):
        return []

    frontier.push((startState, [], 0))  # (state, path, cost)
    explored = set()

    while not frontier.isEmpty():
        state, path, cost = frontier.pop()

        if state in explored:
            continue

        explored.add(state)

        if problem.isGoalState(state):
            return path

        for successor, action, stepCost in problem.getSuccessors(state):
            if successor not in explored:
                newPath = path + [action]
                totalCost = cost + stepCost
                frontier.push((successor, newPath, totalCost))

    return []

def depthFirstSearch(problem):
    """Search the deepest nodes in the search tree first."""
    from util import Stack
    return generalGraphSearch(problem, Stack())

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    from util import Queue
    return generalGraphSearch(problem, Queue())

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    from util import PriorityQueue

    def priorityFunction(item):
        state, path, cost = item
        return cost

    frontier = PriorityQueue(priorityFunction)
    return generalGraphSearch(problem, frontier)

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem. This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    from util import PriorityQueue

    def priorityFunction(item):
        state, path, cost = item
        return cost + heuristic(state, problem)

    frontier = PriorityQueue(priorityFunction)
    return generalGraphSearch(problem, frontier)

# Example heuristic function for testing:
def manhattanHeuristic(state, problem):
    """
    Example heuristic: Returns the Manhattan distance from the current state to the goal.
    Assumes the state and goal are tuples of (x, y).
    """
    goal = problem.goal
    return abs(state[0] - goal[0]) + abs(state[1] - goal[1])

# Abbreviations for easy use
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
