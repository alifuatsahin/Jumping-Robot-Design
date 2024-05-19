import Model #import the simulation and the model
import Algorithm #import data-driven algorithms 
import numpy as np
import matplotlib.pyplot as plt

nb_generation = 500
BS = Algorithm.genetic_algorith(nb_generation)

parameters = Model.Parameters(BS[0])

robot = Model.model(parameters)

robot.simulate(True)
robot.plot()