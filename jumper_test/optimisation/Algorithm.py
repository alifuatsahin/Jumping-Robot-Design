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
    return robot.max_length/robot.energy #/energy 
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

def genetic_algorith(generation):
    print("algo")
    ga_instance = pygad.GA(num_generations=generation,
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

def expected_improvement(x, gp_model, best_y):
    y_pred, y_std = gp_model.predict(x.reshape(-1, 1), return_std=True)
    z = (y_pred - best_y) / y_std
    ei = (y_pred - best_y) * norm.cdf(z) + y_std * norm.pdf(z)
    return ei

def upper_confidence_bound(x, gp_model, beta):
    y_pred, y_std = gp_model.predict(x.reshape(-1, 1), return_std=True)
    ucb = y_pred + beta * y_std
    return ucb


def bayesian_optimisation(iteration, initial_sample_size = 10, special_boundaries = boundaries,pool = [],result = []):
    
    if len(pool) == 0 : 
        param = np.zeros(8)
        print("creating initial pool")
        for i in range(initial_sample_size):
            for j, (lower, upper) in enumerate(boundaries): #random param
                param[j] = np.random.uniform(lower, upper)
            pool.append(param)
            result.append(function_obj(param))
            
            
    print("algo")
    # Gaussian process regressor with an RBF kernel
    kernel = RBF(length_scale=1.0)
    gp_model = GaussianProcessRegressor(kernel=kernel)
    
    plt.figure(figsize=(10, 6))

    for i in range(iteration):
        
        # Fit the Gaussian process model to the sampled points
        gp_model.fit(pool.reshape(-1, 1), result)

        # Determine the point with the highest observed function value
        best_idx = np.argmax(result)
        best_x = pool[best_idx]
        best_y = result[best_idx]

        # Set the value of beta for the UCB acquisition function
        beta = 2.0

        # Generate the Upper Confidence Bound (UCB) using the Gaussian process model
        ucb = upper_confidence_bound(pool, gp_model, beta)

        # Plot the black box function, surrogate function, previous points, and new points
        plt.plot(pool[:, 0], ucb[:, 0], color='red', linestyle='dashed', label='Surrogate Function')
        plt.scatter(pool[:, 0], result[:, 0], color='blue', label='Previous Points')

        if i < iteration - 1:
            new_sample = x_range[np.argmax(ucb)]  # Select the next point based on UCB
            new_result = function_obj(new_sample)
            pool = np.append(pool, new_sample)
            result = np.append(result, new_result)
            plt.scatter(new_sample, new_result, color='green', label='New Points')
    


def pca(sample, nb_component):
    
    print("Create pool")
    
    column_names = ['Link1', 'Link2', 'Link3', 'Link4', 'Link5', 'Compression', 'Rest', 'Spring']
    df = pd.DataFrame(columns=column_names)
    result = pd.DataFrame()
    
    param = np.zeros(8)
    
    for i in range(sample):
        for j, (lower, upper) in enumerate(boundaries):
            param[j] = np.random.uniform(lower, upper)
            
        df = pd.concat([df, pd.DataFrame([param],columns=column_names)], ignore_index=True)
        #df = df.append(pd.Series(param, index=df.columns), ignore_index=True)
        #robot = Model.model(Model.Parameters(param))
        #robot.simulate()
        R = function_obj(param)
        result = pd.concat([result, pd.DataFrame([R])], ignore_index=True)
        #result = result.append([robot.max_length])

    print(df)
    print(result)
    print("apply PCA")
    scaler = MinMaxScaler()
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