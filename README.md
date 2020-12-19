# qphase v1.0

This is the solving algorithm powering [Cuboth](), the (as of December 2020) world's fastest robot to solve an **unmodified** Rubik's Cube. In general, it is a heavily enhanced version of my own [`rob-twophase`](https://github.com/efrantar/rob-twophase) solver designed to fully account for Cuboth's complex quad-arm mechanics while searching for solutions. This leads to extremely efficient solutions of just 19 robot moves (already counting all tilts and non-parallel regrips) within milliseconds and is one of the cornerstone improvements over the previous unmodified cube robot record.

**NOTE:** *The primary purpose of this repository is to document the various new techniques that were necessary to adapt Herbert Kociemba's two-phase algorithm for finding solutions particularly efficient with respect to advanced quad-arm robot mechanics (the code here is however of course fully executable). If you are just looking for a very efficient off-the-shelf solver to use in your own projects, you should probably be looking at [`rob-twophase`](https://github.com/efrantar/rob-twophase) (unless you are working on a robot very similar to Cuboth).*

Now follows a rather detailed description of main ideas behind the `qphase` algorithm. Note that the following text assumes good familarity with Kociemba's two-phase algorithm (see for example [Kociemba's website](http://kociemba.org/cube.htm) or Tomas Rokicki's extensively documented implementation in [`cube20src`](http://kociemba.org/cube.htm)).

## General Considerations

A simple technique for finding a solution that is not just short in terms of the number of moves but also efficient to physically execute with a robot is to generate several/many solutions (rather than just a single one) to then select the fastest one. While this sometimes already gives reasonably good results, one can generally do quite a bit better by considering the robot mechanics directly in the search. However, depending on the type of mechanics to consider, this can become very challenging to do.

My previous solving algorithm `rob-twophase` could for example search directly in the *axial metric (AX)* where two moves on opposite faces count as one (because they can be executed in parallel by an axial robot) or in the *axial quarter metric (AXQT)* where additonally half-turns count as 2 moves. While doing this was certainly already quite tricky to implement, the corresponding extensions of the standard two-phase algorithms are conceptually rather straight-forward. Basically, one just has to modify the moveset available to the solver. Efficiently considering proper quad-arm robot mechanics is a lot more challenging and requires quite a few new techniques.

Still, the basis of the `qphase` algorithm is `rob-twophase` and since a quad-arm robot can also do parallel turns, we want to be searching in either AX or AXQT. While my previous robot [SquidCuber](https://www.youtube.com/watch?v=wLzn1w8vgM4) used the AXQT mode (since a half-turn took almost twice as long as a quarter-turn), we use AX here. This allows us to avoid any complications caused by the quarter-turn metric and Kociemba's two-phase algorithm is also generally more efficient in half-turn metrics. Further, half-turns are actually not that much slower to execute for quad-arm robots because they generally do not require an immediate ungrip action (which of course costs extra time and is almost always needed after quarter-turns).

## Tilts

The biggest mechanical restriction of a quad-arm robot is that it cannot directly turn the top and bottom face of the cube meaning that it generally has to perform several cube rotations, we call those *tilts*, throughout a solve. Since standard solutions may require a lot of tilts we want to code this information directly into the pruning tables. 

There are 24 different ways to hold a cube. Naively augmenting the standard phase1/phase2 pruning tables with this `TILT` coordinate would result in a 24x memory increase. This is problematic as the original tables already require > 1 GB of RAM (with an axial version of Rokicki's extended phase 1 table). Fortunately, `TILT` is highly symmetric. As long as the axis that cannot currently be turned is the same, we can always symmetry transform (with respect to *robot symmetry*) a solution with `tilt1` into one for the same cube in `tilt2`. Therefore, we can symmetry reduce `TILT` to only 3 different states and consequently achieve an 8x memory reduction. Actually implementing this is however quite tricky because the standard Kociemba algorithm already utilizes *cube symmetry* which is not independent of the cube's `TILT` as a symmetry transformation can change the axis that cannot currently be turned. Thus, to perform a *double symmetry reduction* we first have to reduce the current state with respect to cube symmetry, then adjust the `TILT` if necessary (via conjugation) to get an actually equivalent state and finally symmetry reduce the `TILT` with respect to robot symmetry. Overall this makes for a very efficient (both in memory and in runtime) method for fully considering cube rotations during the search.

## Gripper States

## Search Details

## Optimal Regrips
