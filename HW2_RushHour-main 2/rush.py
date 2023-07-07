from heapq import heappush, heappop

class Car:
    def __init__(self, i, j, L, horiz):
        """
        Parameters
        i: int
            Row of the car
        j: int
            Column of the car
        L: int
            Length of the car
        horiz: boolean
            True if the car is horizontal, false
            if the car is vertical
        """
        self.i = i
        self.j = j
        self.L = L
        self.horiz = horiz

class State:
    def __init__(self):
        self.N = 0 # Our cars are on an NxN grid
        self.cars = [] # The first car is the red car
        self.goal = [0, 0] # The state that our red car needs to reach
        self.prev = None # Pointers to previous states (use later)

    def clone(self):
        """
        Make a deep copy of this state

        Return
        ------
        State: Deep copy of this state
        """
        s = State()
        s.N = self.N
        for c in self.cars:
            s.cars.append(Car(c.i, c.j, c.L, c.horiz))
        s.goal = self.goal.copy()
        return s

    def load_puzzle(self, filename):
        """
        Load in a puzzle from a text file
        
        Parameters
        ----------
        filename: string
            Path to puzzle
        """
        fin = open(filename)
        lines = fin.readlines()
        fin.close()
        self.N = int(lines[0])
        self.goal = [int(k) for k in lines[1].split()]
        for line in lines[2::]:
            fields = line.rstrip().split()
            i, j, L = int(fields[0]), int(fields[1]), int(fields[3])
            horiz = True
            if "v" in fields[2]:
                horiz = False
            self.cars.append(Car(i, j, L, horiz))

    def get_state_grid(self):
        """
        Return an NxN 2D list corresponding to this state.  Each
        element has a number corresponding to the car that occupies 
        that cell, or is a -1 if the cell is empty

        Returns
        -------
        list of list: The grid of numbers for the state
        """
        grid = [[-1]*self.N for i in range(self.N)]
        for idx, c in enumerate(self.cars):
            di = 0
            dj = 0
            if c.horiz:
                dj = 1
            else:
                di = 1
            i, j = c.i, c.j
            for k in range(c.L):
                grid[i][j] = idx
                i += di
                j += dj
        return grid
    
    def __str__(self):
        """
        Get a string representing the state

        Returns
        -------
        string: A string representation of this state
        """
        s = ""
        grid = self.get_state_grid()
        for i in range(self.N):
            for j in range(self.N):
                s += "%5s"%grid[i][j]
            s += "\n"
        return s
    
    def __lt__(self, other):
        """
        Overload the less than operator so that ties can
        be broken automatically in a heap without crashing

        Parameters
        ----------
        other: State
            Another state
        
        Returns
        -------
        Result of < on string comparison of __str__ from self
        and other
        """
        return str(self) < str(other)
    
    def get_state_hashable(self):
        """
        Return a shorter string without line breaks that can be
        used to hash the state

        Returns
        -------
        string: A string representation of this state
        """
        s = ""
        grid = self.get_state_grid()
        for i in range(self.N):
            for j in range(self.N):
                s += "{}".format(grid[i][j])
        return s
    
    def is_goal(self):
        """
        Determine whether or not the goal has been reached by comparing
        the current state to the goal state

        Returns
        -------
        isGoal : a boolean that is true if the current state is the goal state

        """
        isGoal = False
        startCar = self.cars[0]
        
        if [startCar.j + startCar.L - 1] == [self.goal[1]]:
            isGoal = True
        else:
            isGoal = False

        return isGoal
    
    def get_neighbors(self):
        """
        Return a list of all possible moves that can be made by checking
        which cars can move where. Each move is appended to a list as a state.

        Returns
        -------
        neighbs : a list of neighboring states

        """
        
        grid = self.get_state_grid()
        neighbs = []
          
        for i in range(len(self.cars)):
            state = self.clone()
            c = state.cars[i]
                       
            if c.horiz:
                end_cord = c.j + (c.L - 1)
                
                if 0 <= end_cord < len(grid) - 1 and grid[c.i][end_cord + 1] == -1:
                    s = self.clone()       
                    s.cars[i].j = s.cars[i].j + 1
                    neighbs.append(s)
                if c.j >= 1 and grid[c.i][c.j - 1] == -1:
                    s = self.clone()       
                    s.cars[i].j = s.cars[i].j - 1
                    neighbs.append(s)
            else:
                end_cord = c.i + (c.L - 1)
                
                if 0 <= end_cord < len(grid) - 1 and grid[end_cord + 1][c.j] == -1:
                    s = self.clone()       
                    s.cars[i].i = s.cars[i].i + 1
                    neighbs.append(s)
                if c.i >= 1 and grid[c.i - 1][c.j] == -1:
                    s = self.clone()       
                    s.cars[i].i = s.cars[i].i - 1
                    neighbs.append(s)

                
                
        return neighbs
    
    def get_Heuristic(self):
        """
        Calculates the heuristic value based on whether or not the red car
        is blocked by another car/whether the goal has been reached

        Returns
        -------
        h : the heuristic(estimate of how long there is left to complete the puzzle)

        """
        h = 0
             
        if self.is_goal():
            h = 0
        else: 
            
            grid = self.get_state_grid()
            end_cord = self.cars[0].j + (self.cars[0].L - 1)
            nextSpace = end_cord
            h = 1
                
            for i in range (end_cord, len(grid) - 1):               
                nextSpace = nextSpace + 1
                if grid[self.cars[0].i][nextSpace] != -1:
                    h = 2
                    
                        
        return h
    
    def get_myheuristic(self):
        """
        Calculates the heuristic value based on how many cars are blocking
        the red car

        Returns
        -------
        h : the heuristic(estimate of how long there is left to complete the puzzle)

        """
        
        cars_blocking = 0
                                                
        grid = self.get_state_grid()
        end_cord = self.cars[0].j + (self.cars[0].L - 1)
        nextSpace = end_cord
               
        for i in range (end_cord, len(grid) - 1):               
            nextSpace = nextSpace + 1
            if grid[self.cars[0].i][nextSpace] != -1:
                cars_blocking += 1
                
    
                
        return cars_blocking
        
        

    def plot(self):
        """
        Create a new figure and plot the state of this puzzle,
        coloring the cars by different colors
        """
        import numpy as np
        import matplotlib.pyplot as plt
        from matplotlib import cm
        from matplotlib.colors import ListedColormap
        c = cm.get_cmap("Paired", len(self.cars))
        colors = [[1, 1, 1, 1], [1, 0, 0, 1]]
        colors = colors + c.colors.tolist()
        cmap = ListedColormap(colors)
        grid = self.get_state_grid()
        grid = np.array(grid)
        plt.imshow(grid, interpolation='none', cmap=cmap)
                  
        
    def solve(self):
        """
        Use breadth first tree search to solve the puzzle

        Returns
        -------
        states : The solution states (the steps to complete the puzzle)

        """
        frontier = [self]                  
        reachedGoal = False
        end = None
        
        while not reachedGoal and len(frontier) > 0:
            state = frontier.pop(0)
                                           
            if state.is_goal():
                reachedGoal = True
                end = state
            else:
                neighbs = state.get_neighbors()
                for move in neighbs:
                    move.prev = state
                    frontier.append(move)
                    
        states = [end]
        state = end
        while state.prev:
            state = state.prev
            states.append(state)  
            
        states.reverse()
        
        return states
        pass
        
        

    def solve_graph(self):
        """
        Use breadth first graph search to solve the puzzle

        Returns
        -------
        states : The solution states (the steps to complete the puzzle)

        """
        
        frontier = [self]
        visited = set([])                     
        reachedGoal = False
        end = None
        node_count = 0
        
        while not reachedGoal and len(frontier) > 0:
            state = frontier.pop(0)
                                           
            if state.is_goal():
                reachedGoal = True
                end = state
            else:
                neighbs = state.get_neighbors()
                for move in neighbs:
                    if not move.get_state_hashable() in visited:  
                        move.prev = state
                        frontier.append(move)
                        visited.add(move.get_state_hashable())
                        node_count += 1
        print(node_count)
        states = [end]
        state = end
        while state.prev:
            state = state.prev
            states.append(state)  
            
        states.reverse()
        
        return states
        pass
    
    def solve_astar(self):
        """
        Use astar search to solve the puzzle

        Returns
        -------
        states : The solution states (the steps to complete the puzzle)

        """
        
        frontier = []
        cumucost = 0
        distance = 1
        costn = cumucost + self.get_Heuristic()
        heappush(frontier, (costn, self, None, cumucost + distance))
        visited = set([])                     
        reachedGoal = False
        end = None
        node_count = 0
        
        
        while not reachedGoal and len(frontier) > 0:
            (est, state, prev, cumucost) = heappop(frontier)
                                           
            if state.is_goal():
                reachedGoal = True
                end = state
                
            else:
                neighbs = state.get_neighbors()
                              
                for move in neighbs:
                    if not move.get_state_hashable() in visited:  
                        move.prev = state
                        costn = cumucost + distance + move.get_Heuristic()
                        heappush(frontier, (costn, move, state, cumucost + distance))
                        visited.add(move.get_state_hashable())    
                        node_count += 1                                       
                        
        print(node_count)
        
        states = [end]
        state = end
        while state.prev:
            state = state.prev
            states.append(state)  
            
        states.reverse()
        
        return states
        pass
        
    def solve_myastar(self):
        """
        Use astar search and my custom heuristic search to solve the puzzle

        Returns
        -------
        states : The solution states (the steps to complete the puzzle)

        """
        
        frontier = []
        cumucost = 0
        distance = 1
        costn = cumucost + self.get_Heuristic()
        heappush(frontier, (costn, self, None, cumucost + distance))
        visited = set([])                     
        reachedGoal = False
        end = None
        node_count = 0
        
        
        while not reachedGoal and len(frontier) > 0:
            (est, state, prev, cumucost) = heappop(frontier)
                                           
            if state.is_goal():
                reachedGoal = True
                end = state
                
            else:
                neighbs = state.get_neighbors()
                
                
                for move in neighbs:
                    if not move.get_state_hashable() in visited:  
                        visited.add(move.get_state_hashable())    
                        move.prev = state
                        costn = cumucost + distance + move.get_myheuristic()
                        heappush(frontier, (costn, move, state, cumucost + distance))                   
                        node_count += 1                                       
                        
        print(node_count)
        
        states = [end]
        state = end
        while state.prev:
            state = state.prev
            states.append(state)  
            
        states.reverse()
        
        return states
        pass
                
            
                
        
        
        
    
        
        
