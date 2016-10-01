# mut8

This is a sandbox, based around the physics engine [Box2D](https://github.com/erincatto/Box2D),
in which creatures evolve to run as fast as possible - by defualt.

The project can be ran [here.](https://rawgit.com/tobq/mut8/master/index.html)

### Controls:
* right arrow (→): Speed up simulation by 1s/s
* left arrow (←): Slow down simulation by 1s/s
* up arrow (↑): Speed up by a factor of 2
* down arrow (↓): Slow down by a factor of 2
* W : Toggle Drawing of World
* Q : Toggle Drawing of overlays (graph, max distance etc.)
* ESC : Reset simulation

###### Notes:
* speed → 0 = pause.
* The graph's Yellow and Cyan lines represent Average and Max speeds over time - respectively.
* You can change how the program runs by altering the `config` object.