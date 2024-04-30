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
    [20, 150], #link
    [20, 150], 
    [20, 150],
    [20, 150],
    [20, 150],
    [5, 30], #compression
    [10, 80], #rest
    [1.5, 14] #spring
    ])

# genetic algorithm

def genetic_algorith():
    print("algo")
    ga_instance = pygad.GA(num_generations=5,
                           num_parents_mating=5,
                           fitness_func=obj,
                           sol_per_pop=10,
                           num_genes=8,
                           gene_space=boundaries,
                           parent_selection_type="sss",
                           keep_parents=2,
                           crossover_type="single_point",
                           mutation_type="random",
                           mutation_percent_genes=None,
                           mutation_num_genes=1)

    # Run the genetic algorithm
    ga_instance.run()

    # Get the best solution found by the genetic algorithm
    best_solution = ga_instance.best_solution()
    print("Best solution:", best_solution)
    return best_solution


def pca():
    print("Create pool")
    param = Model.Parameters()
    robot = Model.model()
    length = robot.simulate()
    
    print("apply PCA")

    # Get the best solution found by the genetic algorithm
    best_solution = ga_instance.best_solution()
    print("Best solution:", best_solution)
    return best_solution