# Getting Started

## Prerequisites

- **Python 3.8+**
- **TensorFlow 2.x**
- **X-Plane 11** simulator
- Required Python libraries: `h5py`, `numpy`, `matplotlib`, `ipywidgets`, `tensorflow`

## Setup

1. Clone this repository:

   ```bash
   git clone https://github.com/cytronicoder/ai4all-robotics.git
   cd ai4all-robotics
   ```

2. Install the required Python packages:

   ```bash
   pip install -r requirements.txt
   ```

3. Ensure X-Plane 11 is properly installed and configured.

## Running the Project

1. Use the data generation notebook to create downsampled runway images and corresponding labels.
2. Train the neural network with the `taxinet_training.ipynb` notebook. Adjust hyperparameters as needed to improve model performance.
3. Test the full system in the X-Plane 11 simulator using the `taxinet_controller.ipynb` notebook.

## Key Files

- `data/taxi_ai4all_data.h5`: Pre-generated dataset with runway images and ground truth labels
- `code/taxinet_training.ipynb`: Notebook for training and evaluating the neural network
- `code/taxinet_controller.ipynb`: Notebook for integrating the neural network and PID controller in simulation
- `helpers/xpc3.py` and `helpers/xpc3_helper.py`: Interface libraries for interacting with the X-Plane 11 simulator
- `helpers/plotting_helper.py`: Helper functions for visualizing simulation results
