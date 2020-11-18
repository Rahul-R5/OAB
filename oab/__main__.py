# This the main file which will run the simulation.

from oab.test.ShapeFactoryTest import run
from oab.Map import Map

if __name__ == "__main__":
    maps = Map(0.5, (5, 5), 8)
    run()
    