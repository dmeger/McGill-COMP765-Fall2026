# Planning Introduction

In order for a robot to act intelligently, it needs to deliberate about the possible motions it can attempt and to select promising or optimal behaviors. The first way that we'll consider this deliberation is an algorithmic approach called **graph-based planning**. This is our starting point because we hope it feels familiar for folks who have the COMP 251 pre-req where we hope you saw many ways to build trees and plan things like shortest paths in graphs.

## Graph-based Planning Formulation

In order to use our tools from graph algorithms, we want the robot to be able to consider its world, geometry and potential motions in graph form. So, we need to interface the robot's physical world in graph form. Here are the elements that we'll use:
- A vertex, $v$, in our planning graph corresponds to a robot state, $x$, and possibly some additional book-keeping information depending on the planning algorithm that we're using. 
- We will start planning at a start state, typically the one the robot is currently occupying $x_t$, which allows us to execute any plan we find immediately.
- The robot's set of goals, $g \in G$, which is one or more elements of the state space, corresponds to terminal vertices in the planning graph.
- Edges, $e(v,v')$ are present between vertices when the robot can make an elementary motion to reach from $v$ to $v'$ in one time-step. That is, there exists $u \in U$, with $U$ the space of all possible actions, such that applying $u$ accomplishes moves the robot from $v$ to $v'$. The specific $u$ associated with each edge is normally recorded in the graph or plan, but it can sometimes be recovered implicitly, such as by using a call to inverse Kinematics or Dynamics.
- Costs $c(v,v')$ come from some problem-dependent measure of what we want to achieve with our plan, but they are almost always going to be spatial or temporal quantities, like the distance between states or the time it takes to transition. Occassionally we might see the energy use, or safety factors. In any case, the function $c$ is external to our planning algorithm and must be available to call for every potential.

Our goal in planning is to find the a path, $P=[v_0, ..., v_k]$ (along with accompanying commands $[u_0,...,u_{k-1}]$), that satisfies starting at the start and ending in the goal set. Properties of planners include:
- Correct: if the planner always outputs a **feasible** path. That is one that begins at the start, ends in a goal and makes valid transitions at every step (respects kinematics and has no collisions).
- Complete: if the planner always returns a path when one is possible.
- Optimal: if the planner only returns shortest/best paths.
- Terminating: if the planner always stops (it cannot run forever).

## Algorithm 1: Brute-force Search

The intuition of brute-force search is:
- Begin at the start, with an empty tree
- Loop forever:
   - Pick a leaf node. If none exist, return fail.
   - Try all possible actions and add the resulting next states to the tree as children.
   - Check the children to see if any are goals. If so, return the path corresponding to this branch, from root to new goal leaf.

I hope you can verify that Brute-force Search for robotic planning is not optimal and it can be non-terminating when state spaces are unbounded or actions allow looping back to previously visited states. This one is only here for us to make fun of (and to just have a gentle start into the topic - we'll draw it on the board in lecture). There isn't much more to say about this method's analysis, and please don't code it on any real robots!

## Algorithm 2: Dynamic Programming 

The problem with Brute-force search was that it's too disorginized in the order it uses to explore nodes and actions, and that it repeats work, checking the same branches and loops over and over. I hope that you already have ideas for how to better organize our search from your pre-reqs. The first one will be to use Dynamic Programming to find a planning sub-problem definition that allows us to divide-and-conquor. The useful one in planning is based on $D(v)$, defined as the cost-to-go from vertex $v$ to the nearest goal. It divides the problem like this:

$$ D(v) = min[ d(v,u) + D(u)]\\
 \forall u \in N(v).$$

In words, the best cost to go from $v$ to a goal is the lowest over avaiable actions of the immediate transition cost to each neighbor, plus the cost to go from that neighbor to the goal. There is nothing special about writing out this recurrence. It is simple intuition from the additive nature of the problem formulation.

However, it allows us to make a smarter algorithm. If we can "finish" computing D(v) for any nodes, then they can be re-used in the search process, effectively truncating computational threads whenever new paths need to be considered through $v$. The first way to compute a finished $D(v)$ is to look at nodes $v$ in the goal set. They have zero cost to go, so we can immediately fill in their values.

Now we work backwards, opening up subsequent neighbors that can reach each finished node and computing the DP recurrence for them. A problem in robotics is that our knowledge of the graph connectivity does not come for free. We may need many calls to inverse and forward kinematics to execute the needed loops here, and therefore we make a small tweak to produce the algorithm really used.

## Algorithm 3: Dijkstra's Greedy Forward DP

Dijkstra's algorithm states that the best next node to explore is the one with the lowest cost-to-reach (note the difference from cost-to-go above), within an open frontier reachable from the set of completed nodes. We will see that this selection choice, starting from the origin, a node with zero cost-to-reach, gives a very efficient, optimal planner.

Dijkstra's re-uses the DP sub-problem symbol $D(v)$, but here means the smallest possible sum of costs along any path from $v_0$ to $v$. We maintain several data structures:
- A list $D(v)$ of the cost-to-reach for each node we have computed so far. Initialized with 0 at the origin and $\infty$ elsewhere. 
- A priority queue, $Q$, with active vertices, for which we have at least one way to reach, but are not yet certain of their shortest route.
- A list of finalized nodes, $F$, for which we write back-pointers to other nodes in $F$ that reach the origin. 

Dijkstra's uses the greedy rule: select the open node $v_{nearest}$ from $Q$ which has the lowest cost-to-reach. The key to the algorithm's efficientcy is to note:
- $v_{nearest}$ can never be updated further. Since it has the lowest cost-to-reach in $Q$ and $c(v,v')$ is positive, all paths explored through all other nodes in $Q$, even if they loop back to hit $v_{nearest}$ will always have higher costs. When we "pop" $v_{nearest}$, we are done with its computation.
- Other nodes, $v$ in $Q$ may be neighbors of $v_{nearest}$ where we have already proposed one or mnore path(s) (and computed the best $D(v)$ over that set). We need to check if the new possible path that goes $[v_0, ..., v_{nearest}, v]$ could imporove the value of $D(v)$. If so, we need to udpate it, and appropriately re-order our priority queue $Q$.
- Using an efficient priority queue, $Q$, allows us to make these updates and select the next node to finish quickly. This data structure is the key to running Dijkstra's fast and a *Fibonacci Heap* is a good option.

## Algorithm 4: A\* Search

In robotics, we often have a rough idea of the high level properties that will make paths interesting. For example, with a single goal, nodes that are closer to the goal make sense to search before nodes that are farther. This doesn't mean we expect every path to be a straight line to the goal, but often some segments of best paths will be in the direction of the goal. 

Our way to formalize this intuition is by creating a **planning heuristic**, $h(v)$. The function $h(v)$ is an input to our planning algorithm which must be coded by the roboticist to give a positive value for every vertex. 

The **ideal** heurisitic, with special name $h^{\*}(v)$, is the true cost-to-go to the goal from $v$. With this heuristic, planning is easy, we could always greedily explore the next unexplored node with the lowest value of $f(v) = g(v) + h^{\*}(v)$. The sum of cost-to-reach plus cost-to-goal is the full/true cost of the path in this case, because both inputs are exact. As the job of optimal planning is to select the lowest cost paths, this greedy selection does the full job here. 

Do we expect to have access to $h^{\*}(v)$, which is essentially the solution to planning as an input? How did we receive that function? It would require running something like a full planner **before we plan**. It's not going to be available with any reasonable computation, so rather, this unit is about planning with $h(v)$ functions that are approximations to the cost-to-goal.

### Heuristic Properties

Real heuristics are estimates of the cost-to-goal. Therefore, by construction (and for things to make sense), they have the special property that $h(v)=0$ whenever $v \in G$, the node is a member of our goal set.

A heuristic $h(v)$ can be:
- **Admissible** if it underestimates, that is $\forall v \in V, h(v) \leq h^{\*}(v).$ 
- **Monotonic** if the computed $f(v)$ increases along all paths. Here we note that $\forall (v,v') \in V$, $f(v) \leq f(v')$ implies that $h(v) \leq c(v,v') + h(v')$ because $f=g+h$ and the difference in the $g(v)$ and $g(v')$ components of $f$ is $c(v,v')$.

### The A\* Algorithm

A\* planning means running a loop identical to Dijkstra's, but instead of using the cost-to-reach as the priority value, replacing this with the full path-cost estimate $f(v)=g(v)+h(v)$. Otherwise, the algorithm is unmodified.

### A\* Analysis

Dijsktra's algorithm was optimal and complete. Does A\* preserve these properties?

**Thm: $A\*$ Planning is Optimal if $h(v)$ is Admissible**

**Proof:**
Consider two paths to the goal, the optimal one $P^{\*}$ and a longer alternative $P_2$. Note that if our planner reaches the goal from either one of these paths, it will immediately return that path (check the Dijkstra's pseudo-code). An optimal algorithm must always return $P^{\*}$. The question of optimality is then, "can A\* visit all nodes in $P_2$ before it completes $P^{\*}$? We will show this is impossible.

Consider the $f(v)$ value that would be present in the queue if $v=goal$ and it has been inserted into the queue along path $P_2$. Recall that for goals, $h(v)=0$, so $f(v)$ must hold the real path cost of $P_2$.

Compare this with the values present in the queue for any $v \in P^{\*}$ that we happen to be considering at the same time. For those, $f(v) = g(v) + h(v)$, and here we use our assumption. $h(v) \leq h^{\*}(v)$, and so the computed $f(v)$ is less than the path cost of $P^{\*}$. We have the ordering $f(v) \leq c(P^{\*}) \leq c(P_2)$. 

Putting this together, it means that for all nodes $v \in P^{\*}$, they will have some under-estimated path cost and it will surely be lower than the value of the goal along the longer path, $P_2$. This means we will surely pop all nodes along $P^{\*}$, including the goal along that path, whose f-value and back-pointer we'll update when we reach it along $P^{\*}$, before we pop that goal along $P_2$. With this invariant, we are done. A\* planning with an admissible heuristic is optimal.

### A\* Exercises:
1) How do Admissible and Monotonic heuristics compare? Does one imply the other and in what ordering?
2) Analyze A\* with an always-zero heuristic, $\forall v \in V, f(v)=0$. What are it's properties and how does it compare with other algorithms we know?





