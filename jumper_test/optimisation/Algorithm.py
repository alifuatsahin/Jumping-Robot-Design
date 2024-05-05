import numpy as np
import Model #import the simulation and the model
import pandas as pd
import matplotlib.pyplot as plt

#PCA 
from sklearn.decomposition import PCA
from sklearn.preprocessing import StandardScaler
from sklearn.preprocessing import MinMaxScaler

#genetic algorithms

import pygad
#https://pygad.readthedocs.io/en/latest/

# Define the objective function 
def obj(ga_instance,parameters,solution_idx):
    print("new model")
    robot = Model.model(Model.Parameters(parameters))
    robot.simulate()
    return robot.max_lenght/robot.energy #/energy 


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


def pca(sample, nb_component):
    
    print("Create pool")
    
    column_names = ['Link1', 'Link2', 'Link3', 'Link4', 'Link5', 'Compression', 'Rest', 'Spring']
    df = pd.DataFrame(columns=column_names)
    result = pd.DataFrame()
    
    param = np.zeros(8)
    
    for i in range(sample):
        for j, (lower, upper) in enumerate(boundaries):
            param[j] = np.random.uniform(lower, upper)
        df = df.append(pd.Series(param, index=df.columns), ignore_index=True)
        robot = Model.model(Model.Parameters(param))
        robot.simulate()
        result = result.append([robot.max_length])

    #print(df)
    #print(result)
    print("apply PCA")
    scaler = MinMaxScaler()
    scaled_df = scaler.fit_transform(df)

    # Perform PCA
    pca = PCA(n_components=nb_component)  # Keep 10 components
    principal_components = pca.fit_transform(scaled_df)

    return principal_components,result,pca,labels,df
# Visualize the explained variance ratio
    #plt.bar(range(1, pca.n_components_ + 1), pca.explained_variance_ratio_)
    #plt.xlabel('Principal Component')
    #plt.ylabel('Explained Variance Ratio')
    #plt.title('Explained Variance Ratio per Principal Component')
    #plt.show()



def biplot(principal_components,result, pca, labels=None, label_size=10, arrow_length=0.1, arrow_width=0.01):
    plt.figure(figsize=[12,12])
    # Plot data points
    
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
    # Project data onto 2 dimensions for visualization
    plt.figure()
    plt.scatter(PC[:, 0], PC[:, 1],c=result.iloc[:, 0], cmap='viridis', alpha=0.5)
    plt.xlabel('Principal Component 1')
    plt.ylabel('Principal Component 2')
    plt.title('Data Projected onto 2 Dimensions')
    plt.grid()
    plt.show()
    
    # Project data onto 2 dimensions for visualization
    plt.figure()
    plt.scatter(PC[:, 0], PC[:, 1],c=result.iloc[:, 0], cmap='viridis', alpha=0.5)
    plt.xlabel('Principal Component 1')
    plt.ylabel('Principal Component 2')
    plt.title('Data Projected onto 2 Dimensions')
    plt.grid()
    plt.show()