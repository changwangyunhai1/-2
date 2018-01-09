# search.py
# ---------
# Licensing Information: Please do not distribute or publish solutions to this
# project. You are free to use and extend these projects for educational
# purposes. The Pacman AI projects were developed at UC Berkeley, primarily by
# John DeNero (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# For more info, see http://inst.eecs.berkeley.edu/~cs188/sp09/pacman.html

"""
In search.py, you will implement generic search algorithms which are called
by Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples,
        (successor, action, stepCost), where 'successor' is a
        successor to the current state, 'action' is the action
        required to get there, and 'stepCost' is the incremental
        cost of expanding to that successor
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.  The sequence must
        be composed of legal moves
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other
    maze, the sequence of moves will be incorrect, so only use this for tinyMaze
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s,s,w,s,w,w,s,w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first
    [2nd Edition: p 75, 3rd Edition: p 87]

    Your search algorithm needs to return a list of actions that reaches
    the goal.  Make sure to implement a graph search algorithm
    [2nd Edition: Fig. 3.18, 3rd Edition: Fig 3.7].

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """

    from util import Stack
    from game import Directions

    open_nodes = Stack()
    closed_nodes = []
    iteration = {}
    path = []
    
    open_nodes.push(problem.getStartState())

    while( not open_nodes.isEmpty() ):  
        cur_node = open_nodes.pop()
        
        if problem.isGoalState(cur_node):
            break;
            
        if cur_node in closed_nodes:
           continue
        
        expand_nodes = []        
        for item in problem.getSuccessors(cur_node):
            if (item[0] not in closed_nodes):
                expand_nodes.append(item[0])
        
        if (len(expand_nodes) > 0):
            for item in expand_nodes:
                open_nodes.push(item)
                iteration[item] = cur_node
        closed_nodes.append(cur_node)       
    
    while( cur_node != problem.getStartState()):
        parent_node = iteration[cur_node]
        if cur_node[0] == parent_node[0] + 1:
            path.append(Directions.EAST)
        elif cur_node[0] == parent_node[0] - 1:
            path.append(Directions.WEST)
        elif cur_node[1] == parent_node[1] - 1:
            path.append(Directions.SOUTH) 
        elif cur_node[1] == parent_node[1] + 1:
            path.append(Directions.NORTH)
        cur_node = parent_node

    path.reverse()
    return path


def breadthFirstSearch(problem):
    """
    Search the shallowest nodes in the search tree first.
    [2nd Edition: p 73, 3rd Edition: p 82]
    """
    from util import Queue
    from game import Directions

    open_nodes = Queue()
    closed_nodes = []
    iteration = {}
    path = []
    
    open_nodes.push(problem.getStartState())

    while( not open_nodes.isEmpty() ):  
        cur_node = open_nodes.pop()
        
        if problem.isGoalState(cur_node):
            break;
            
        if cur_node in closed_nodes:
           continue
        
        expand_nodes = []
        for item in problem.getSuccessors(cur_node):
            if (item[0] not in closed_nodes):
                expand_nodes.append(item[0])
        
        if (len(expand_nodes) > 0):
            for item in expand_nodes:
                open_nodes.push(item)
                iteration[item] = cur_node
        closed_nodes.append(cur_node)   
    
    while( cur_node != problem.getStartState()):
        parent_node = iteration[cur_node]
        if cur_node[0] == parent_node[0] + 1:
            path.append(Directions.EAST)
        elif cur_node[0] == parent_node[0] - 1:
            path.append(Directions.WEST)
        elif cur_node[1] == parent_node[1] - 1:
            path.append(Directions.SOUTH) 
        elif cur_node[1] == parent_node[1] + 1:
            path.append(Directions.NORTH)
        cur_node = parent_node

    path.reverse()
    return path

def uniformCostSearch(problem):
    "Search the node of least total cost first. "
    "*** YOUR CODE HERE ***"
    from util import PriorityQueue
    from game import Directions
    import heapq

    open_nodes = PriorityQueue()
    closed_nodes = []
    iteration = {}
    path = []

    open_nodes.push(problem.getStartState(), 0)

    while( not open_nodes.isEmpty() ):
        (cur_cost,cur_node) = heapq.heappop(open_nodes.heap)

        if problem.isGoalState(cur_node):
            break;

        if cur_node in closed_nodes:
            continue

        expand_nodes = []
        for item in problem.getSuccessors(cur_node):
            if (item[0] not in closed_nodes):
                expand_nodes.append((item[0],item[2]))

        if (len(expand_nodes) > 0):
            for item in expand_nodes:
                if not iteration.has_key(item[0]):
                    open_nodes.push(item[0], item[1] + cur_cost)
                    iteration[item[0]] = cur_node
                else:
                    for i in open_nodes.heap:
                        if i[1] == item[0]:
                            if item[1] + cur_cost < i[0]:
                                iteration[item[0]] = cur_node
                                open_nodes.heap.remove((i[0],i[1]))
                                open_nodes.push(item[0], item[1] + cur_cost)
                            break

        closed_nodes.append(cur_node)
    
    while( cur_node != problem.getStartState()):
        parent_node = iteration[cur_node]
        if cur_node[0] == parent_node[0]+ 1:
            path.append(Directions.EAST)
        elif cur_node[1] == parent_node[1]- 1:
            path.append(Directions.SOUTH)
        elif cur_node[0] == parent_node[0]- 1:
            path.append(Directions.WEST)
        elif cur_node[1] == parent_node[1]+ 1:
            path.append(Directions.NORTH)
        cur_node = parent_node

    path.reverse()
    return path

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    "Search the node that has the lowest combined cost and heuristic first."
    "*** YOUR CODE HERE ***"
    from util import PriorityQueue
    from game import Directions
    import heapq

    open_nodes = PriorityQueue()
    closed_nodes = []
    iteration = {}
    path = []
    
    open_nodes.push(problem.getStartState(), 0+heuristic(problem.getStartState(),problem))

    while( not open_nodes.isEmpty() ):
        (cur_cost, cur_node) = heapq.heappop(open_nodes.heap)
        if problem.isGoalState(cur_node):
            break;   

        if cur_node in closed_nodes:
            continue

        expand_nodes = []
        for item in problem.getSuccessors(cur_node):
            if (item[0] not in closed_nodes):
                expand_nodes.append((item[0],item[2]))

        if (len(expand_nodes) > 0):
            for item in expand_nodes:
                if not iteration.has_key(item[0]):
                    open_nodes.push(item[0], item[1] + cur_cost-heuristic(cur_node,problem)+heuristic(item[0],problem))
                    iteration[item[0]] = cur_node
                else:
                    for i in open_nodes.heap:
                        if i[1] == item[0]:
                            if item[1] + cur_cost-heuristic(cur_node,problem)+heuristic(item[0],problem) < i[0]:
                                iteration[item[0]] = cur_node
                                open_nodes.heap.remove((i[0],i[1]))
                                open_nodes.push(item[0], item[1] + cur_cost-heuristic(cur_node,problem)+heuristic(item[0],problem))
                            break

        closed_nodes.append(cur_node)
        
    while( cur_node != problem.getStartState()):
        parent_node = iteration[cur_node]
        if cur_node[0] == parent_node[0]+ 1:
            path.append(Directions.EAST)
        elif cur_node[1] == parent_node[1]- 1:
            path.append(Directions.SOUTH)
        elif cur_node[0] == parent_node[0]- 1:
            path.append(Directions.WEST)
        elif cur_node[1] == parent_node[1]+ 1:
            path.append(Directions.NORTH)
        cur_node = parent_node

    path.reverse()
    return path

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
