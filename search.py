# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
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
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()

class SearchGenerator:

    def __init__(self, problem: SearchProblem, fringe, algorithm, heuristic=None) -> None:
        self.problem = problem
        self.algorith = algorithm
        self.fringe = fringe
        self.heuristic = heuristic


    def search(self):
        start_state = self.problem.getStartState()
        directions = []
        
        # Push the start state onto the stack
        if isinstance(self.fringe, util.PriorityQueue):
            self.fringe.push((start_state, directions), 0) 
        else:
            self.fringe.push((start_state, directions))

        # Mark the start state as visited
        visited = set()

    
        while not self.fringe.isEmpty():

            # Pop the current state from the stack (ucs will get the lowest cost node)
            current_state, directions = self.fringe.pop()

            if self.problem.isGoalState(current_state):
                return directions

            if not current_state in visited:
                visited.add(current_state)

                # Get successors of the current state
                successors = self.problem.getSuccessors(current_state)
        
                for successor in successors:
                    next_state = successor[0]
                    action = successor[1]
                    new_directions = directions + [action]


                    if self.algorith == "dfs" or self.algorith == "bfs":

                        if not next_state in visited:
                                self.fringe.push((next_state, new_directions))
                    
                    elif self.algorith == "ucs":

                        if next_state not in visited: 
                            # calculate the cumulative cost to the next state
                            cost_to_next_state = self.problem.getCostOfActions(new_directions) 

                            # If item already in priority queue with higher priority, update its priority and rebuild the heap.

                            # If item already in priority queue with equal or lower priority, do nothing.
                            
                            # If item not in priority queue, do the same thing as self.push.
                            self.fringe.update((next_state, new_directions), cost_to_next_state)   


                    elif self.algorith == "a*":
                        
                        if next_state not in visited:

                            # calculate the cumulative cost to the next state: g(n)
                            cost_to_next_state = self.problem.getCostOfActions(new_directions)

                            # get the heuristic value to the next state: h(n)
                            heuristic= self.heuristic(next_state, self.problem) 

                            # update the fringe with the priority of g(n) + h(n)
                            self.fringe.update((next_state, new_directions), heuristic + cost_to_next_state)

def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem: SearchProblem):
    """Search the deepest nodes in the search tree first."""
    return SearchGenerator(problem, algorithm="dfs", fringe=util.Stack()).search()
    
def breadthFirstSearch(problem: SearchProblem):
    """Search the shallowest nodes in the search tree first."""
    return SearchGenerator(problem, algorithm="bfs", fringe=util.Queue()).search()

def uniformCostSearch(problem: SearchProblem):
    """Search the node of least total cost first."""
    return SearchGenerator(problem, algorithm="ucs", fringe=util.PriorityQueue()).search()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem: SearchProblem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    return SearchGenerator(problem, algorithm="a*", fringe=util.PriorityQueue(), heuristic=heuristic).search()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
