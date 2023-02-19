# My Voronoi Planner
## Done:
- made goal a region & connected its vertices
- made start a region & connected its vertices
- exponential scaling to hausdorff distance metric in a* planner to encourage the paths to start out more similar (to solve path switching)
- added ellipse arc behind robot
- added gap size heuristic to a*

## To Try:
- make the goal not hardcoded in vor_node
- spline interpolation is buggy sometimes
- verify lidar scans (compare mean and median)
- maybe can do incremental voronoi
- list of lidars, compare oldest to most recent, throw away old scans if mean/median doesn't agree
    - figure out the heuristic for this

## Tunable Params:
- spline coefficient
- priority of similar path (to solve path switching)
- priority of large gaps
- gap hard limit size

## Order of priority (Opinion)
- stable path (no switching)
- gap size - priority should increase as possible gap size in environment decreases
- distance of path - least important