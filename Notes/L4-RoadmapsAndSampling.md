# Roadmaps and Sampling Planners

So far we've applied standard CS algorithms that work on any graph planning problem, from playing board games to scheduling jobs on a CPU. That is, they were not very "robotic". This week we'll deviate and mention methods that take specific advantage of the continuous, geometric nature of the robot planning problem. Things to notice:
- In Euclidean spaces, the shortest path through free space is easy to find and is optimal. We might always start planning by checking a straight line start-goal, and many planning instances can be solved immediately.
- Of course, obstacles will fall along this path in all interesting cases. However, the same reasoning extends to sub-problems when navigating obstacles:
    - The best way to pass from vertex $v_1$ to $v_2$ is also a straight line. In 2D, the only way to go "around" an obstacle is by touching the vertices that form its **convex hull**. 
    - In higher dimensions, the shortest path might pass over the edges or vertices of obstacles, but regardless, these entities are our key inputs to planning. The description of obstacles and their boundaries can be viewed as at least equally important to planning as their dual, the free space.

A problem with extending the discrete, anonymous graph algorithms of the previous sections to robotics problems is that we must discretize the naturally continuous, and potentially high dimensional state and action spaces of our robot. Discretization simply means dividing continuous space up into finitely many, usually regularly spaced locations and connecting them up based on the kinematics and obstacle information of our planning problem. This discrete graph grows exponentially in the planning dimensions because there are more and more factors to divide evenly. A unit 
line in 1D can be broken into K even line segments with length $1/K$. A unit plane takes $K^{2}$ small squares of equal side length and a unit cube $K^{3}$ small cubes. We very quickly want to avoid this data structure and almost never use a 6-D even grid to consider the motions of our Kinova arms, for example.

Instead, planners that are aware of being run in metric spaces can use various elements of geometry: simplifications, known invariant properties, useful data structures and more in order to find more efficient representations.

## Algorithm 1: Voronoi Graph

A Voronoi Diagram/Graph (VG) is the data structure formed by edges and curves that are equidistant from at least two elements of the free space boundary. Examples are shown on the slides and easy to obtain. The VG can be thought of as a skeleton, lying at the centre of the free space. It's an important data structure for many elements of geometry processing. It is used in robotic navigation, especially when we want robots to be maximally safe. The idea is to build the VG, then connect every element of free space up to its network. We move from the start to a VG vertex or curve (vertices are easier to identify in a simple algorith, but curves are usually closer). Then along the VG, until we apply the inverse operation to reach the goal.

## Algorithm 2: Visibility Graph

A visibility graph extends the concept of "the shortest path is a straight line". When obstacles are present in between start and goal, our path must deviate from the direct one. However, we can show that, for polygonal obstacles made up of straight line segments, and in 2D, a string that we'd stretch as tightly as possible (shortest), ends up only passing through free space in straight segments and laying along the straight edges of the obstacles. How can we form this line? As a Visibility Graph (VisG):

- Init, add the start as an active vertex in the VisG 
- Loop until no new vertices are added:
    - For all active vertices v:
        - For all other vertices of obstacles v':
            - Check visibility by forming the line (v,v') and collision-checking with all edges.
            - If it is "visible", that is there are no collisions, add v' as an active vertex in VisG with edge (v,v').
        - Remove v from the active list

To make this into a planning algorithm, add the goal by considering it's visibility to all veritices $v$ in VisG and adding edges (v,goal) whenever visible. Then, run graph shortest-path in VisG and return the shortest path. 

VisG planning is optimal in 2D but only finds approximate paths in higher dimensions, since the shortest path there may pass over edges. Forming the VisG has a polynomial cost in the number of vertices present, without significant computational contribution of the planning dimension (it is roughly linear for the extra cost of collision-checking against higher dimensional surfaces). When the obstacle shapes are simple enough (low vertex-count), VisG is a great choice. However, what about problems in high dimensions where the obstacles may also be complex? It's time for the star of our planning unit!

## Rapidly Exploring Random Trees (RRT)

A problem with both VG and VisG methods is that they process obstacle descriptions directly and thus scale in complexity with the description length of these. Of course there will be no way to avoid the fact that navigating around complex shapes makes paths more complex, but we want our interactions with those shapes to be as limited as possible. Here is a (very surprisingly) simple algorithm that only needs to collision check short line segments against the obstacles and still builds a global roadmap through the planning space:

### Algorithm RRT:
- Init $V={x_init}$ and $E=\emptyset$ 
- For n=1...N
    - Sample $x_{rand}$ uniformly from the free space
    - Find $x_{nearest} \in V$ using an efficient method
    - Generate candidate $x_{new} = Steer(x_{nearest},x_{new})$
    - If not $CheckCollision(V,E,x_{nearest},x_{new})$
        - Add to the tree. $V \leftarrow V \cup x_{new}$, $E \leftarrow E \cup (x_{nearest},x_{new})$.

This algorithm builds a tree through the planning space with surprisingly good scaling in terms of planning dimension and obstacle complexity, but only when all the "core" steps are done well. We need efficient sampling, nearest neighbors, and collision checking. Luckily, all of these are well-researched and excellent sub-methods are available.

The punch-line about RRTs, to come, is that they **explore rapidly**. This means that they achieve efficient coverage of the free space. Consider the potential alternative, they could have spent a lot of time densifying the tree near the start. When planning start-to-goal, with a potentially adversarial goal location, the key for good complexity is getting everywhere fast. We will continue to see how RRTs do this, and just how fast they are.

First, a quick run-down of how to make the RRT-building loop run efficiently:

### Sampling Free Space

### Nearest Neighbor Queries

For each sample, we must find the closest point in the existing tree. This is an important part of achieving the rapoidly-exploring property, but if we aren't careful, it will require us to process all of the existing tree each iteration. This adds an overall $O(n^2)$ complexity and might encourage us not growing the tree too large. However, rather than brute force comparison, we can use a spatially-indexed data structure

**Option 1: KD-tree**

A KD-Tree is a data structure that splits space into axis-aligned half regions, ideally with half of the points in each region. We can rapidly (in $O(lg(n))$ time) find the bin closest to our query point. Unfortunately, as bins are rectangular and "distance" has (hyper)spherical geometry, it can be that the closest point exists in a neighboring bin. The retrieval algorithm thus proceeds upwards for some levels to confirm a match. Note that we must perform an insertion for each new point and balancing kd-trees is not a trivial operation. In very large planning dimensions, the "curse of dimensionality" causes more bins to be neighbors and the upwards recursion can end up covering nearly the whole tree.

To summarize, although the complexity of kd-trees depends on the point distribution and dimensionality. In practice, it is effective to pair with RRTs almost always, but we should know what concerns to have. 

**Option 2: Locality-Sensitive Hashing**

Here, we form a hash table, as you are hopefully all familiar with. However, instead of hoping for every data point to be in a separate bucket, we choose a hash function that brings spatially nearby points together into a common bucket. When retrieving a nearest neighbor, we can use the query's hash value to pull up these likely neighbors and then only search that smaller subset. Design choices on how to control the number of neighbors in a bucket and how to handle the search over neighbors (it may still require a local kd-tree), impact the efficiency of this method. It is often run in an "approximately nearest" fashion where the best match may lay in another hash bin but we ignore this for speed.

## RRT Properties

**Claim: Building RRTs is efficient in high dimensions and with complex obstaces**

**Disclaimer:** Worst-case analysis (in fact any precise analysis) is very complex!

1. RRT will eventually "cover" the free space.
2. RRT will not compute the optimal path asymptotically.
3. New nodes will be distributed with a Voronoi-bias. That is, connected to each vertex $v$ in proportion to the volume of the Voronoi cell that is nearest to $v$.
4. The probability that RRT finds a path to $g$ increases exponentially in the number of iterations.

### Claim 1: Analysis

Coverage in this case is probabilistic. For any point $p$ in the free space, and any small distance $\epsilon$, a vertex in the tree $v$ will eventually enter the hyper-sphere around $p$, $||p-v||_{2}^{2} < \epsilon$. 

First, we are assuming free space is connected. The tree can obviously never reach through a wall or obstacle to a pocket of unreachable nodes. 

Next, the connection must posses finite, non-zero volume. Intuitively, there must be a "tube" which has reasonable radius at all points. Otherwise, there can be only a single point, or series of points, with no volume around them. None of these points $p$ will ever be sampled precisely in a continuous space. That is $p(x_{rand} == p)=0$, a consequence of probability in continuous spaces. When there is non-zero volume however, we can just wait until the sample $x_{rand}$ is in the "direction" along the tube. It will happen eventually. We make progress and repeat the same logic until we reach the end of the tube (and all tubes in the space).

With these conditions, eventually we will always make progress along the unknowable, but existing path from $x_{init}$ to $p$, which has finite, non-zero volume. The argument is basically that uniform coverage samples every place infinitely often, including whatever place we need to sample next for the tree to reach $p$. 

Claim 1 already tells us RRT is "probabilistically complete". If a path exists, the RRT will find a path with probability increasing as we continue to sample. But, this process could maybe be very slow and expensive. The next claims are about speed and details of the path found.

### Claim 2: RRT is Probabilistically Sub-Optimal

RRT returns a path which is the branch of the tree, $x_{init}$ to the first leaf that enters an ${\epsilon}-ball$ of the goal, $g$. 

a) There may be many sub-optimal ways to pass obstacles. These are all competing with the optimal "route". Although the fact that the optimal path is shortest and therefore requires the least progress to be made, the alternatives can be just a bit longer and there can be countlessly many. Which one is selected is up to random sampling chance.

b) The assumptions of Claim 1 mean we can expect a non-zero volume tube to exist around the optimal path. So, even if RRT finds a path within this "homotopy class" (technical definition you don't need to know precisely... just think of the things close to optimal), the RRT-path will still very likely be less smooth than the optimal one. 

RRT will pick points within the non-zero volume in all radial directions as likely as the optimal central point. Since the central, optimal path has vanishingly small chance of being sampled, and there are many steps along the path, overall, there is essentially zero chance to sample optimally.

### Claim 3: New Nodes are Voronoi-distributed

This simply accounts for our uniform sampling of free space in a convenient way to book-keep which RRT vertex will be $x_{nearest}$ at each step. Not much more analysis here.

### Claim 4: RRT finds $g$ with probability increasing exponentially in iteration

This is a geometry argument about the speed with which Voronoi cells shrink as the tree grows. By Claim 3 we shrink the largest ones more often, and we shrink them by something related to the Steer() distance every time. For an ${\epsilon}-ball$ to remain around $g$ without a sample, it must necessarily have some set of containing Voronoi regions. Eventually, as the overall volume of the tree grows, the remaining potential volume in the space shrinks and the "Voronoi-bias" property increases the probability for us to either have already sampled within $\epsilon$ of the goal, or to do it in this immediate next iteration.

## RRT Modifications of Interest

One of the worst parts of RRT is its sub-optimality, and the jerky nature of the paths it produces. A famous improvement is ${RRT}^{\*}$. This algorithm differs in the way the tree is extended to include the newly sampled $x_{new}$. Instead of only considering connections to $x_{near}$, ${RRT}^{\*}$ considers a nearby set of nodes in the tree and looks for the one that actually gives the shortest path to $x_{new}$. It also does a small amount of "re-wiring" of the existing tree if $x_{new}$ may now allow shorter paths to some existing nodes. With this combination, a somewhat complex set of analysis allows us to show convergence of ${RRT}^{\*}$ to nearly optimal paths.



