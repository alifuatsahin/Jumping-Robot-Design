import numpy as np
import Model #import the simulation and the model
import pandas as pd
import matplotlib.pyplot as plt

#PCA 
from sklearn.decomposition import PCA
from sklearn.preprocessing import StandardScaler
from sklearn.preprocessing import MinMaxScaler
from scipy.stats import norm

#genetic algorithms

import pygad
#https://pygad.readthedocs.io/en/latest/

#bayesion algorithms

from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF

#https://medium.com/@okanyenigun/step-by-step-guide-to-bayesian-optimization-a-python-based-approach-3558985c6818

# Define the objective function 
def obj(ga_instance,parameters,solution_idx): # spe declaration of GA
    return function_obj(parameters)

def function_obj(parameters):
    print("new model")
    robot = Model.model(Model.Parameters(parameters))
    robot.simulate()
    max_length = max(robot.parameters.l1,robot.parameters.l2,robot.parameters.l3,robot.parameters.l4,robot.parameters.l5)
    
    if robot.energy > 1e-8 : #!= 0.0 : not 0 
        return (robot.max_length/max_length)#*(robot.energy)
    else :
        return 0.0

# Define the boundaries

parameter_name = ["l1", "l2", "l5", "compression", "rest_angle", "stiffness", "link_angle"]

boundaries = np.array([
    [30, 60], #link
    [70, 170], 
    [50, 170],
    [5, 40], #compression
    [10, 80], #rest
    [1.5, 14], #spring
    [10, 80] #link_angle
    ])

#this is not beautiful but it is working
xrange = np.array([
    np.linspace(boundaries[0][0],boundaries[0][1], 100), #link
    np.linspace(boundaries[1][0],boundaries[1][1], 100), 
    np.linspace(boundaries[2][0],boundaries[2][1], 100),
    np.linspace(boundaries[3][0],boundaries[3][1], 100),
    np.linspace(boundaries[4][0],boundaries[4][1], 100),
    np.linspace(boundaries[5][0],boundaries[5][1], 100), #compression
    np.linspace(boundaries[6][0],boundaries[6][1], 100) #rest #spring
    ])

############ genetic algorithm ############ 

def genetic_algorith(generation):
    print("algo")
    ga_instance = pygad.GA(num_generations=generation,
                           num_parents_mating=5,
                           fitness_func=obj,
                           sol_per_pop=10,
                           num_genes=7,
                           gene_space=boundaries,
                           parent_selection_type="sss",
                           keep_parents=2,
                           crossover_type="single_point",
                           mutation_type="random",
                           mutation_percent_genes=None,
                           mutation_num_genes=1,
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


def bayesian_optimisation(iteration, initial_sample_size = 10, special_boundaries = boundaries,initialpool = [],result = []):
    
    if len(initialpool) == 0 :
        pool = []
        param = np.zeros(8)
        print("creating initial pool")
        for i in range(initial_sample_size):
            for j, (lower, upper) in enumerate(boundaries): #random param
                param[j] = np.random.uniform(lower, upper)
            pool.append(param.copy())
            result.append(function_obj(param))
    else : 
        pool = initialpool.copy()
            
    print("algo")
    # Gaussian process regressor with an RBF kernel
    kernel = RBF(length_scale=1.0)
    gp_model = GaussianProcessRegressor(kernel=kernel)
    
    for i in range(iteration):
        print("iteration")
        
        # Fit the Gaussian process model to the sampled points
        gp_model.fit(pool, result)

        # Determine the point with the highest observed function value
        best_idx = np.argmax(result)
        best_x = pool[best_idx]
        best_y = result[best_idx]
        
        # Generate the Upper Confidence Bound (UCB) using the Gaussian process model
        ucb = upper_confidence_bound(xrange.T, gp_model, beta)
        
        plt.figure(figsize=(10, 6))
        # Plot the black box function, surrogate function, previous points, and new points
        plt.plot(xrange[0], ucb, color='red', linestyle='dashed', label='Surrogate Function')
        plt.scatter([arr[0] for arr in pool], result, color='blue', label='Previous Points')
        plt.show()

        if i < iteration - 1:
            
            plt.figure(figsize=(10, 6))
        # Plot the black box function, surrogate function, previous points, and new points
            plt.plot(xrange[0], ucb, color='red', linestyle='dashed', label='Surrogate Function')
            plt.scatter([arr[0] for arr in pool], result, color='blue', label='Previous Points')
            
            for j, (lower, upper) in enumerate(boundaries): #random param
                param[j] = np.random.uniform(lower, upper)
            param[0] = xrange[0][np.argmax(ucb)]  # Select the next point based on UCB
                
            new_result = function_obj(param)
            pool.append(param.copy())
            result.append(new_result)
            
            plt.scatter(param[0], new_result, color='green', label='New Points')
            plt.show()

def upper_confidence_bound(x, gp_model, beta):
    y_pred, y_std = gp_model.predict(x, return_std=True)
    ucb = y_pred + beta * y_std
    return ucb

def bayesian_optimisation_single(iteration, initial_sample_size = 10,this_param = 0, special_boundaries = boundaries,initialpool = [],initialresult = []):
    
    if (this_param <0 or this_param > len(parameter_name)-1):
        print("wrong parameter number, parameters are ",parameter_name)
        return 0
    
    print("optimising around parameter", parameter_name[this_param])
    
    the_range = xrange.copy()
    
    
    if len(initialpool) == 0 :
        pool = []
        result = []
        param = np.zeros(8)
        print("creating initial pool")
        for j, (lower, upper) in enumerate(boundaries): #random param
                param[j] = np.random.uniform(lower, upper)
                if (j !=this_param):
                    the_range[j] = np.linspace(param[j],param[j], 100)
        
        
        for i in range(initial_sample_size):
            param[this_param] = np.random.uniform(boundaries[this_param][0],boundaries[this_param][1])
            pool.append(param.copy())
            result.append(function_obj(param))
    else : 
        pool = initialpool.copy()
        result = initialresult.copy()
            
    print("running the algorithm")
    # Gaussian process regressor with an RBF kernel
    kernel = RBF(length_scale=1.0)
    gp_model = GaussianProcessRegressor(kernel=kernel)
    
    for i in range(iteration):
        print("iteration",i)
        
        # Fit the Gaussian process model to the sampled points
        gp_model.fit(pool, result)
        
        # Generate the Upper Confidence Bound (UCB) using the Gaussian process model
        ucb = upper_confidence_bound(the_range.T, gp_model, beta)
        
        plt.figure(figsize=(10, 6))
        # Plot the black box function, surrogate function, previous points, and new points
        plt.plot(the_range[this_param], ucb, color='red', linestyle='dashed', label='Surrogate Function')
        plt.scatter([arr[this_param] for arr in pool], result, color='blue', label='Previous Points')
        plt.show()

        if i < iteration - 1:
            
            plt.figure(figsize=(10, 6))
        # Plot the black box function, surrogate function, previous points, and new points
            plt.plot(the_range[this_param], ucb, color='red', linestyle='dashed', label='Surrogate Function')
            plt.scatter([arr[this_param] for arr in pool], result, color='blue', label='Previous Points')
            
            param[this_param] = the_range[this_param][np.argmax(ucb)]  # Select the next point based on UCB
                
            new_result = function_obj(param)
            pool.append(param.copy())
            result.append(new_result)
            
            plt.scatter(param[this_param], new_result, color='green', label='New Points')
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
    
    column_names = ['Link1', 'Link2', 'Link3', 'Link4', 'Link5', 'Compression', 'Rest', 'Spring']
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