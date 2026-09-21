# Bug Planners

Bug Algorithms fall in the family of behavior-based methods which are a contrast in basic ideas from deliberative planners such as graph methods. They
- Do not require a full map of the world.
- Do no computation before they start moving.
- Simply react to the immediate sensor reading or perception algorithm's output.
- Can still reach goals often/always.

So, they are really attractive options for cheap robots. They have famously been championed by folks like Rodney Brooks, the founder of iRobot and designer of the first generation Roomba vacuums. Robot vacuums (and lawn-mowers, pool cleaners, etc), typically have all day or night to get their cleaning done, so may not need to move along the most optimal path. But, they are price-competitive, so having a simple robot that "works" is a key goal here.

## Bug 1:

The Bug 1 Method is simply:
- Forever:
    - End when at goal.
    - Check the direction to the goal. If it is clear:
        - Move towards the goal.
    - Else:
        - Turn to the left until the forward direction is clear and walk "with your hand on the obstacle".

The point of a behavior-based algorithm is that the right simple choice may still give good properties. Most of the work here lies in analyzing candidate algorithms. Since there is little to no computation involved, we are just interested in the properties of the plans returned, not big-oh issues:

- Is the Bug 1 Algorithm Terminating?
- Is the Bug 1 Algorithm Complete?
- Is the Bug 1 Algorithm Optimal?

The slides show several successful cases where we'd be very happy to have picked Bug 1. However, there is a counter-example that can make it loop forever.

## Bug 2 Variant

A small fix improves the situation:
- Forever:
    - End when at goal.
    - Check the direction to the goal. If it is clear:
        - Move towards the goal.
    - Else:
        - Turn to the left **and remember the line $L$ between our point of contact with the obstacle and the goal, plus distance $d_{init}=d(x,g)$.**
        - Walk "with your hand on the obstacle" until you reach $L$, the path along $L$ to the goal is clear and $d(x,g) \le d_{init}$.

This fixes the previous counter-example. We claim this algorithm is now Complete. It will reach the goal if a path exists. Can you find any counter-example?

No Bug Algorithm is optimal because the choice of turning left vs right is baked into the code and doesn't adapt to the environment (and complicated envs require us to sometimes turn left and other times turn right). 

### Exercise:

1) The Bug algorithm written here is non-terminating. In case of being closed in a region that separates the robot from the goal, it will just orbit that region. Can you think of a fix to make the method Terminating?