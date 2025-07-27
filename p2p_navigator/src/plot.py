import numpy as np
import matplotlib.pyplot as plt
sim_type="tunnel"
model_types = ['M-COST', 'obstacle cost-map', 'stacked obstacle cost-map' ,'lidar']  # List your model types here
stacked_states = 'your_stacked_states'
colors  = ['#60AA60','#E68A98','#3e55c7' ,'#ADD8E6' ]# Light Blue, Light Green, Light Pink

# Choose the type of plot: 'boxplot' or 'errorbar'
plot_type = 'errorbar'  # Change to 'errorbar' for error bar plots

distances_data = []
steps_data = []
success_rates = []

for model_type in model_types:
    # Load the data
    complete = np.load( f"{sim_type}/{model_type}/complete.npy")
    distances = np.load(f"{sim_type}/{model_type}/distances.npy")
    steps_all = np.load(f"{sim_type}/{model_type}/steps_all.npy")
      
    # Filter data for successful iterations only
    distances_of_successful_iteration = [f for b, f in zip(complete, distances) if b]
    steps_of_successful_iteration = [f for b, f in zip(complete, steps_all) if b]
    
    distances_data.append(distances_of_successful_iteration)
    steps_data.append(steps_of_successful_iteration)
    
    # Calculate success rate
    success_rate = np.mean(complete) * 100
    success_rates.append(success_rate)
    print("{} avg distance {} , success rate {}, steps {} " .format( model_type, np.mean(distances_of_successful_iteration) , success_rate,np.mean(steps_of_successful_iteration)))#, np.mean(distances_data)),success_rate ,np.mean(steps_of_successful_iteration 



# Plotting the data
plt.figure(figsize=(18, 6))
plt.rcParams.update({'font.size': 11})

# Plot for distances
plt.subplot(1, 3, 1)
if plot_type == 'boxplot':
    bp = plt.boxplot(distances_data, labels=model_types, patch_artist=True)
    # Set softer colors for each box
    for patch, color in zip(bp['boxes'], colors):
        patch.set_facecolor(color)
    plt.title('Mean Distances to goal')
else:
    # Calculate means and standard deviations
    distances_means = [np.mean(data) for data in distances_data]
    distances_stds = [np.std(data) for data in distances_data]
    x_positions = np.arange(len(model_types))
    plt.bar(x_positions, distances_means, yerr=distances_stds, color=colors, capsize=5)
    plt.xticks(x_positions, model_types)
    plt.title('Mean Distances to goal')
plt.ylabel('Distance(m)')
plt.xticks(rotation=45, ha='right')  # Rotate labels
# Plot for steps
plt.subplot(1, 3, 2)
if plot_type == 'boxplot':
    bp = plt.boxplot(steps_data, labels=model_types, patch_artist=True)
    # Set softer colors for each box
    for patch, color in zip(bp['boxes'], colors):
        patch.set_facecolor(color)
    plt.title('Mean number of steps to goal')
else:
    # Calculate means and standard deviations
    steps_means = [np.mean(data) for data in steps_data]
    steps_stds = [np.std(data) for data in steps_data]
    x_positions = np.arange(len(model_types))
    plt.bar(x_positions, steps_means, yerr=steps_stds, color=colors, capsize=5)
    plt.xticks(x_positions, model_types)
    plt.title('Mean number of steps to goal')
plt.ylabel('Number of Steps')
plt.xticks(rotation=45, ha='right')  # Rotate labels
plt.subplot(1, 3, 3)
x_positions = np.arange(len(model_types))
plt.bar(x_positions, success_rates, color=colors)
plt.title('Success Rate')
plt.ylabel('Success Rate (%)')
plt.xticks(x_positions, model_types, rotation=45, ha='right')


# Annotate bars with success rate percentages
for i, rate in enumerate(success_rates):
    plt.text(i, rate + 1, f'{rate:.2f}%', ha='center', va='bottom')

plt.tight_layout()
plt.show()