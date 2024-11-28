# System Overview

## 1. Data Collection

The X-Plane 11 Simulator is used to generate downsampled runway images with corresponding ground truth values for crosstrack error and heading error. We collected 20,000 labeled samples of 8x16 grayscale runway images, which are paired with ground truth labels for training and testing.

## 2. Neural Network (Taxinet)

We designed a feedforward neural network, named Taxinet, to predict crosstrack and heading errors from the input runway images. The network takes the downsampled images as input and outputs the predicted errors, which are used by the controller to adjust the aircraft's steering. It's optimized using stochastic gradient descent (SGD) and mean squared error (MSE) loss, and its performance is evaluated based on prediction accuracy and robustness via histograms and sample visualization.

## 3. **Controller**

The controller is responsible for steering the aircraft based on the predicted errors from the neural network. It consists of two main components:

- The PID controller manages steering adjustments based on predicted errors to align the aircraft with the centerline
- Crosstrack and heading error gains are manually tuned for optimal performance

## 4. Simulation

The system is tested in a closed-loop environment using X-Plane 11 for realistic feedback and visualization. Environmental conditions, such as varying lighting and shadows, are simulated to test system robustness.
