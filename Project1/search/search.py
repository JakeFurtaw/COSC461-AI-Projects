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


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

#-------------------------Start Q1------------------------------------------------------------------------------------------------------
def depthFirstSearch(problem):
    "*** YOUR CODE HERE ***"
#-------------Variables, Arrays, and Dictionaries-------------
    fringeList = util.Stack()#creating the fringe list using stack because its DFS
    visitedNodes = {}#giving a place for the visited nodes to live, Contains popped nodes and their directions
    solution = []#holds the sequence of directions to the goal
    parents = {}#holds the parents and their child nodes
    startState = problem.getStartState()#setting the start state
    fringeList.push((startState, 'Undefined', 0))#adding it to the fringe list
#-----------Main While Loop--------
    while not fringeList.isEmpty():#while loop that runs while the fringe list is not empty
        currentNode= fringeList.pop()#pop from the top of fringe list and set it to current node
        visitedNodes[currentNode[0]] = currentNode[1]#storing current node and its direction
        if problem.isGoalState(currentNode[0]):#checking to see if we have reached the goal state yet
            nodeSolution = currentNode[0]
            break  
        for suc in problem.getSuccessors(currentNode[0]):
            if suc[0] not in visitedNodes.keys():#checking that our current child is not already in visited nodes
                parents[suc[0]] = currentNode[0]#filling parents w/ child and its parent
                fringeList.push(suc)#adding to fringe
#-----------Solution Path-----------
    while(nodeSolution in parents.keys()):#creating solution path
        nodeSolutionLast = parents[nodeSolution]#getting parent list and assigning it to the var node by node
        solution.insert(0, visitedNodes[nodeSolution])#inserting the direction to the solution list
        nodeSolution = nodeSolutionLast #setting next node to previous node
    return solution #self explanatory
    util.raiseNotDefined()
#-------------------------Start Q2------------------------------------------------------------------------------------------------------------
def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
#-------------Variables, Arrays, and Dictionaries-------------
    fringeList = util.Queue()#creating the fringe list using stack because its BFS
    visitedNodes = {}
    solution = []
    parents = {}
    startState = problem.getStartState()
    fringeList.push((startState, 'Undefined', 0))
#-----------Main While Loop--------
    while not fringeList.isEmpty():
        currentNode= fringeList.pop()
        visitedNodes[currentNode[0]] = currentNode[1]
        if problem.isGoalState(currentNode[0]):
            nodeSolution = currentNode[0]
            break
        for suc in problem.getSuccessors(currentNode[0]):
            if suc[0] not in visitedNodes.keys() and suc[0] not in parents.keys():#if successor hasnt been visited or isnt a child of another node
                parents[suc[0]] = currentNode[0]
                fringeList.push(suc)
#-----------Solution Path-----------
    while(nodeSolution in parents.keys()):
        nodeSolutionLast = parents[nodeSolution]
        solution.insert(0, visitedNodes[nodeSolution])
        nodeSolution = nodeSolutionLast
    return solution
    util.raiseNotDefined()
#-------------------------Start Q3-----------------------------------------------------------------------------------------------------
def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
#-------------Variables, Arrays, and Dictionaries-------------
    fringeList = util.PriorityQueue()#Using Priority Queue
    visitedNodes = {}
    cost = {}
    solution = []
    parents = {}
    startState = problem.getStartState()
    fringeList.push((startState, 'Undefined', 0), 0)
#-----------Main While Loop--------
    while not fringeList.isEmpty():
        currentNode= fringeList.pop()
        visitedNodes[currentNode[0]] = currentNode[1]
        if problem.isGoalState(currentNode[0]):
            nodeSolution = currentNode[0]
            break
        for suc in problem.getSuccessors(currentNode[0]):
            if suc[0] not in visitedNodes.keys():
                priority = currentNode[2] + suc[2]#calculating the cost if node hasnt been visited
                if suc[0] in cost.keys():#checking cost of current node
                    if cost[suc[0]] <= priority:#if new cost is greater than current cost, continue
                        continue
                fringeList.push((suc[0], suc[1], priority), priority)#pushing node back into fringe list and changing cost and parent, if new cost less than current cost
                cost[suc[0]] = priority#changing cost
                parents[suc[0]] = currentNode[0]#store child and its parent
#-----------Solution Path-----------
    while(nodeSolution in parents.keys()):
        nodeSolutionLast = parents[nodeSolution]
        solution.insert(0, visitedNodes[nodeSolution])
        nodeSolution = nodeSolutionLast 
    return solution
    util.raiseNotDefined()
#-------------------------Start Q4-----------------------------------------------------------------------------------------------------
def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0
def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    fringeList = util.PriorityQueue()#Using Priority Queue again
    visitedNodes = {}
    cost = {}
    solution = []
    parents = {}
    startState = problem.getStartState()
    fringeList.push((startState, 'Undefined', 0), 0)

    while not fringeList.isEmpty():
        currentNode= fringeList.pop()
        visitedNodes[currentNode[0]] = currentNode[1]

        if problem.isGoalState(currentNode[0]):
            nodeSolution = currentNode[0]
            break

        for suc in problem.getSuccessors(currentNode[0]):
            if suc[0] not in visitedNodes.keys():
                priority = currentNode[2] + suc[2] + heuristic(suc[0], problem)#calculating the cost if node hasnt been visited, accounting for heuristic

                if suc[0] in cost.keys():
                    if cost[suc[0]] <= priority:
                        continue
                fringeList.push((suc[0], suc[1], currentNode[2] + suc[2]), priority)
                cost[suc[0]] = priority 
                parents[suc[0]] = currentNode[0]
    
    while(nodeSolution in parents.keys()):
        nodeSolutionLast = parents[nodeSolution]
        solution.insert(0, visitedNodes[nodeSolution])
        nodeSolution = nodeSolutionLast
    return solution
    util.raiseNotDefined()



# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
