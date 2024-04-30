import numpy as np
import Model #import the simulation and the model
import pygad

#genetic algorithms

#https://pygad.readthedocs.io/en/latest/

# Define the objective function 
def obj(ga_instance,parameters,solution_idx):
    print("new model")
    robot = Model.model(Model.Parameters(parameters))
    length = robot.simulate()
    return length #/energy 

# Define the boundaries

boundaries = np.array([
    [2/100, 15/100], #link
    [2/100, 15/100], #link
    [1.5/1000, 14/1000], #spring
    [10*np.pi/180, 80*np.pi/180], #rest
    [5*np.pi/180, 30*np.pi/180]])

# genetic algorithm

def genetic_algorith():
    print("algo")
    ga_instance = pygad.GA(num_generations=5,
                           num_parents_mating=5,
                           fitness_func=obj,
                           sol_per_pop=10,
                           num_genes=5,
                           gene_space=boundaries,
                           parent_selection_type="sss",
                           keep_parents=2,
                           crossover_type="single_point",
                           mutation_type="random",
                           mutation_percent_genes=10)

    # Run the genetic algorithm
    ga_instance.run()

    # Get the best solution found by the genetic algorithm
    best_solution = ga_instance.best_solution()
    print("Best solution:", best_solution)
    return best_solution