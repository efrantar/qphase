# qphase v1.0

This is the solving algorithm powering [Cuboth](), the (as of December 2020) world's fastest robot to solve an **unmodified** Rubik's Cube. In general, it is a heavily enhanced version of my own [`rob-twophase`](https://github.com/efrantar/rob-twophase) solver designed to fully account for Cuboth's complex quad-arm mechanics while searching for solutions. This leads to extremely efficient solutions of just 19 robot moves (already counting all tilts and non-parallel regrips) within milliseconds and is one of the cornerstone improvements over the previous unmodified cube robot record.

**NOTE:** *The primary purpose of this repository is to document the various new techniques that were necessary to adapt Herbert Kociemba's two-phase algorithm for finding solutions particularly efficient with respect to advanced quad-arm robot mechanics (the code here is however of course fully executable). If you are just looking for a very efficient off-the-shelf solver to use in your own projects, you should probably be looking at [`rob-twophase`](https://github.com/efrantar/rob-twophase) (unless you are working on a robot very similar to Cuboth).*

Now follows a rather detailed description of the `qphase` algorithm. Note that the following text assumes good familarity with Kociemba's two-phase algorithm (see for example [Kociemba's website](http://kociemba.org/cube.htm) or Tomas Rokicki's extensively documented implementation in [`cube20src`](http://kociemba.org/cube.htm)).

# General Considerations

# Tilts

# Gripper States

# Search Details

# Optimal Regrips
