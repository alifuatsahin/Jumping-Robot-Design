

import numpy as np
import Model #import the simulation and the model
import pandas as pd
import matplotlib.pyplot as plt

#PCA 
from sklearn.decomposition import PCA
from sklearn.preprocessing import StandardScaler
from sklearn.preprocessing import MinMaxScaler
from scipy.stats import norm
  
# importing required libraries 
from mpl_toolkits.mplot3d import Axes3D  

#genetic algorithms

import pygad
#https://pygad.readthedocs.io/en/latest/

#bayesion algorithms

from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF
from sklearn.pipeline import make_pipeline
from itertools import product

#https://medium.com/@okanyenigun/step-by-step-guide-to-bayesian-optimization-a-python-based-approach-3558985c6818

point = 20
# Define the objective function 
def obj(ga_instance,parameters,solution_idx): # spe declaration of GA
    return function_obj(parameters)

def function_obj(parameters):
    robot = Model.model(Model.Parameters(parameters))
    jump_distance, energy = robot.simulate()
    max_length = max(robot.parameters.l2, robot.parameters.l3_c*robot.parameters.l2, robot.parameters.l5_c*robot.parameters.l2)
    
    if abs(jump_distance) > 0.9:
        return 0
    else:
        return (jump_distance/max_length)

# Define the boundaries

parameter_name = ["l2", "l3_c", "l4", "l5_c", "compression_ratio", "rest_angle", "stiffness", "link_angle"]

boundaries = np.array([
    [100, 200], #link 2
    [0.8, 1.2], # link 3 coeff
    [0.5, 0.9], # link 4 coeff
    [0.6, 1.4], # link 5 coeff
    [0.1, 0.8], #compression ratio
    [20, 70], #rest
    [5, 30], #spring stiffness
    [10, 80] #link_angle
    ])

#R*boundaries[i][1]/(boundaries[i][1]-boundaries[i][0])
def create_boundaries(parameter, R):
    B = np.array([[max(parameter[i] - R*boundaries[i][1]/(boundaries[i][1]-boundaries[i][0]),boundaries[i][0]), min(parameter[i] + R*boundaries[i][1]/(boundaries[i][1]-boundaries[i][0]),boundaries[i][1])] for i in range(len(parameter))])
    return B.copy()

def create_range(B):
    xrange = np.array([np.linspace(bounds[0], bounds[1], point) for bounds in B])
    return xrange.copy()

def create_array(B):
    xrange = np.array([{'low': bounds[0], 'high': bounds[1]} for bounds in B])
    return xrange.copy()
        

############ genetic algorithm ############ 

def genetic_algorith(generation,bound=boundaries):
    print("algo")
    ga_instance = pygad.GA(num_generations=generation,
                           num_parents_mating=5,
                           fitness_func=obj,
                           sol_per_pop=10,
                           num_genes=8,
                           gene_space=create_array(bound),
                           parent_selection_type="sss",
                           keep_parents=2,
                           crossover_type="single_point",
                           mutation_type="random",
                           mutation_percent_genes=None,
                           mutation_num_genes=5,
                          save_solutions=True)

    # Run the genetic algorithm
    ga_instance.run()

    # Get the best solution found by the genetic algorithm
    best_solution = ga_instance.best_solution()
    print("Best solution:", best_solution)
    
    # https://pygad.readthedocs.io/en/latest/pygad.html#plotting-methods-in-pygad-ga-class
    ga_instance.plot_fitness()
    #ga_instance.plot_genes()
    ga_instance.plot_new_solution_rate()
    return best_solution


############ Bayesian optimisation ############ 

# Set the value of beta for the UCB acquisition function
beta = 2.0

def expected_improvement(x, gp_model, best_y):
    y_pred, y_std = gp_model.predict(x, return_std=True) #.reshape(-1, 1)
    z = (y_pred - best_y) / y_std
    ei = (y_pred - best_y) * norm.cdf(z) + y_std * norm.pdf(z)
    return ei

def upper_confidence_bound(x, gp_model, beta):
    y_pred, y_std = gp_model.predict(x, return_std=True)
    ucb = y_pred + beta * y_std
    return ucb

def bayesian_optimisation(iteration, initial_sample_size = 10,params = [0], special_boundaries = boundaries, initial_best = [], printer = 1):
    
    if any(value > len(parameter_name)-1 or value < 0 for value in params):
        print("wrong parameter number, parameters are ",parameter_name)
        return 0
    
    print("optimising around parameter:")
    for value in params :
        print(parameter_name[value])
    
    pool = []
    result = []
    param = np.zeros(8)
    
    # if there is a parameter only, we want to optimise around this param
    if initial_best == []:
        for j, (lower, upper) in enumerate(special_boundaries): #random param
            param[j] = np.random.uniform(lower, upper)
    else:
        for j, (lower, upper) in enumerate(special_boundaries): #random param
            param[j] = initial_best[j]
        
    
    print("creating initial pool")
    for i in range(initial_sample_size):
        for P in params :
            param[P] = np.random.uniform(special_boundaries[P][0],special_boundaries[P][1])
        pool.append(param.copy())
        result.append(function_obj(param))
            
    print("running the algorithm")
    # Gaussian process regressor with an RBF kernel
    kernel = RBF(length_scale=1.0)
    gp_model = make_pipeline(StandardScaler(),GaussianProcessRegressor(kernel = kernel,optimizer = "fmin_l_bfgs_b",n_restarts_optimizer=100))
    gp_model.fit(pool, result)
                     
    
    for i in range(iteration):
        print("iteration",i)
        
        # Fit the Gaussian process model to the sampled points
        gp_model.fit(pool, result)
        
        ucb = []
        # Generate the Upper Confidence Bound (UCB) using the Gaussian process model
        param_combinations = list(product(*[np.linspace(special_boundaries[P][0], special_boundaries[P][1], point) for P in params]))
        best_combination = None
        best_ucb = float('-inf')

        for idx, combination in enumerate(param_combinations):
            for P, value in zip(params, combination):
                param[P] = value
            current_ucb = upper_confidence_bound([param], gp_model, beta)
            ucb.append(current_ucb)
        
            if current_ucb > best_ucb:
                best_ucb = current_ucb
                best_combination = param.copy()
                
        
        if i % printer == 0 or i == 0 or i == iteration - 1:
            
            if len(params) == 1:
                plt.figure(figsize=(10, 6))
            # Plot the black box function, surrogate function, previous points, and new points
            
                Range = np.linspace(special_boundaries[params[0]][0], special_boundaries[params[0]][1], point)
                plt.plot(Range, ucb, color='red', linestyle='dashed', label='Surrogate Function')
                plt.scatter([arr[params[0]] for arr in pool], result, color='blue', label='Previous Points')
                plt.show()

            elif len(params) == 2:
                fig = plt.figure(figsize=(10, 6))
                ax = fig.add_subplot(111, projection='3d')
                
                Range1 = np.linspace(special_boundaries[params[0]][0], special_boundaries[params[0]][1], point)
                Range2 = np.linspace(special_boundaries[params[1]][0], special_boundaries[params[1]][1], point)
                Range1_grid, Range2_grid = np.meshgrid(Range1, Range2)
                ucb_plotting = np.array(ucb).reshape(Range1_grid.shape)
        # Plot the black box function, surrogate function, previous points, and new points
                surface = ax.plot_surface(Range1_grid, Range2_grid, ucb_plotting, cmap='viridis', alpha=0.5,label='Surrogate Function')
                fig.colorbar(surface, ax=ax, shrink=0.5, aspect=5, label='UCB Value')

                ax.scatter([arr[params[0]] for arr in pool], [arr[params[1]] for arr in pool], result, color='blue', label='Previous Points')
                plt.show()

        if i % printer == 0 or i == 0 or i == iteration - 1:
            if len(params) == 1:
                plt.figure(figsize=(10, 6))
                Range = np.linspace(special_boundaries[params[0]][0], special_boundaries[params[0]][1], point)
            # Plot the black box function, surrogate function, previous points, and new points
                plt.plot(Range, ucb, color='red', linestyle='dashed', label='Surrogate Function')
                plt.scatter([arr[params[0]] for arr in pool], result, color='blue', label='Previous Points')

            elif len(params) == 2:
                fig = plt.figure(figsize=(10, 6))
                ax = fig.add_subplot(111, projection='3d')
                
                Range1 = np.linspace(special_boundaries[params[0]][0], special_boundaries[params[0]][1], point)
                Range2 = np.linspace(special_boundaries[params[1]][0], special_boundaries[params[1]][1], point)
                Range1_grid, Range2_grid = np.meshgrid(Range1, Range2)
                ucb_plotting = np.array(ucb).reshape(Range1_grid.shape)
            # Plot the black box function, surrogate function, previous points, and new points
                surface = ax.plot_surface(Range1_grid, Range2_grid, ucb_plotting, cmap='viridis', alpha=0.5,label='Surrogate Function')
                fig.colorbar(surface, ax=ax, shrink=0.5, aspect=5, label='UCB Value')
                ax.scatter([arr[params[0]] for arr in pool], [arr[params[1]] for arr in pool], result, color='blue', label='Previous Points')

                # Select the next point based on UCB
        print("the best",best_combination)
        new_result = function_obj(best_combination)
        pool.append(best_combination.copy())
        result.append(new_result)
        
        if i % printer == 0 or i == 0 or i == iteration - 1:
            if len(params) == 1:
                plt.scatter(best_combination[params[0]], new_result, color='green', label='New Points')
                plt.show()

            elif len(params) == 2:
                ax.scatter(best_combination[params[0]], best_combination[params[1]], new_result, color='red', label='New Points')
                plt.show()
            
    # Determine the point with the highest observed function value
    best_idx = np.argmax(result)
    best_x = pool[best_idx]
    best_y = result[best_idx]
    
    print("Best solution:", best_x)
    print("with result:", best_y)
    return best_x

############ PCA ############ 

def pca(sample, nb_component):
    
    print("Create pool")
    
    column_names = parameter_name
    df = pd.DataFrame(columns=column_names)
    result = pd.DataFrame()
    
    param = np.zeros(8)
    
    for i in range(sample):
        for j, (lower, upper) in enumerate(boundaries):
            param[j] = np.random.uniform(lower, upper)
            
        df = pd.concat([df, pd.DataFrame([param.copy()],columns=column_names)], ignore_index=True)
        #df = df.append(pd.Series(param, index=df.columns), ignore_index=True)
        #robot = Model.model(Model.Parameters(param))
        #robot.simulate()
        R = function_obj(param)
        result = pd.concat([result, pd.DataFrame([R])], ignore_index=True)
        #result = result.append([robot.max_length])

    print(df)
    print(result)
    print("apply PCA")
    #scaler = MinMaxScaler()
    scaler = StandardScaler()
    scaled_df = scaler.fit_transform(df)

    # Perform PCA
    pca = PCA(n_components=nb_component) 
    principal_components = pca.fit_transform(scaled_df)
    
    # Visualize the explained variance ratio
    plt.bar(range(1, pca.n_components_ + 1), pca.explained_variance_ratio_)
    plt.xlabel('Principal Component')
    plt.ylabel('Explained Variance Ratio')
    plt.title('Explained Variance Ratio per Principal Component')
    plt.show()

    return principal_components,result,pca,column_names,df

def biplot(principal_components,result, pca, labels=None, label_size=10, arrow_length=0.1, arrow_width=0.01):
    plt.figure(figsize=[12,12])
    # Plot data points
    
    #scaler = MinMaxScaler()
    #scaler = StandardScaler()
    #result = scaler.fit_transform(result) #better for visualisation)
    
    plt.scatter(principal_components[:, 0], principal_components[:, 1],c=result.iloc[:, 0], cmap='viridis', alpha=0.5)
    
    # Plot arrows for feature loadings
    feature_vectors = pca.components_.T
    for i, v in enumerate(feature_vectors):
        plt.arrow(0, 0, v[0]*arrow_length, v[1]*arrow_length, color='r', alpha=0.5, width=arrow_width, head_width=2*arrow_width)
        if labels is not None:
            plt.text(v[0]*arrow_length, v[1]*arrow_length, labels[i], color='r', ha='right', va='bottom', fontsize=label_size)
    
    plt.xlabel('Principal Component 1')
    plt.ylabel('Principal Component 2')
    plt.title('Biplot')
    plt.grid()
    plt.show()
    
def pca_plot(PC,result):
    
    #scaler = MinMaxScaler()
    #scaler = StandardScaler()
    #result = scaler.fit_transform(result) #better for visualisation)
    
    
    plt.figure()
    scatter = plt.scatter(PC[:, 0], PC[:, 1], c=result.iloc[:, 0], cmap='viridis', alpha=0.5)
    plt.xlabel('Principal Component 1')
    plt.ylabel('Principal Component 2')
    plt.title('Data Projected onto 2 Dimensions')
    plt.grid()
    plt.colorbar(scatter, label='Color Mapping')  # Add colorbar legend
    plt.show()
    
    plt.figure()
    scatter = plt.scatter(PC[:, 1], PC[:, 2], c=result.iloc[:, 0], cmap='viridis', alpha=0.5)
    plt.xlabel('Principal Component 1')
    plt.ylabel('Principal Component 2')
    plt.title('Data Projected onto 2 Dimensions')
    plt.grid()
    plt.colorbar(scatter, label='Color Mapping')  # Add colorbar legend
    plt.show()