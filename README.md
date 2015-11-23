# Machine Learning 6.867 Homework

## Q-Learning Project

### Dependencies

https://github.com/RobotLocomotion/director

### Recommended setup

Add to ~/.bash_profile or ~/.bash_rc:

`alias ddConsoleApp=$HOME/path-to/build/bin/ddConsoleApp`

### How to run with iPython notebook

Find full path to `ipython`

```
which ipython
```

Run ddConsoleApp with full path to `ipython`, with `notebook` as argument

(for example:)

```
ddConsoleApp /usr/local/bin/ipython notebook
```

Run the cells to set properties of a simulation, and then `sim.run()`

Currently need to restart the kernel after each time starting a new simulation.



### How to run simple demo

```
cd project/code/
ddConsoleApp movingIntersection.py
```

Alternatively, can press play in `runcar.ipynb`

Once visualizer opens:

- F8 to open the console
- `timer.start()`






