# CV2X-LOCA
This repo is the implementation of the paper "Toward C-V2X Enabled Connected Transportation System: RSU-Based Cooperative Localization Framework for Autonomous Vehicles".


## Overview
CV2X-LOCA innovatively uses C-V2X channel state information to achieve lane-level positioning accuracy. 

<div align=center><img src=./Demo/fig3.png width="600"></div>

It is composed of four modules:
- Data Processing Module: Standardizes and processes incoming data for accurate localization.
- Coarse Positioning Module: Overcomes non-convex optimization challenges by approximating ML estimators, ensuring robust vehicle localization.
- Environment Parameter Correcting Module: Adapts to environmental variations by dynamically adjusting signal propagation models, crucial for handling traffic scenarios heterogeneity.
- Vehicle Trajectory Filtering Module: Enhances localization precision through advanced filtering techniques, addressing spatio-temporal constraints effectively.


## Data Generation
This section outlines the simulation data included in this repository, generated within a Matlab environment to support the testing and validation of localization algorithms against the CV2X-LOCA framework. The data simulate real-world conditions to ensure the relevance and applicability of our findings to actual autonomous vehicle localization challenges.

### RSS Data (`rssi_noise.mat`)
Generated through Eq. (1), considering noise fluctuations and adjusting environmental parameters (γ) to mimic various road environments, reflecting the heterogeneity of urban traffic scenarios.

### Vehicle Trajectory Data (`trace_1.mat`)
Utilizes the constant velocity lane changing (CVLC) model for simulating realistic vehicle trajectories, assuming no acceleration or braking.

### RSUs Coordinate Data (`AP.mat`)
Deployed in a simulated two-way, four-lane road segment, with RSUs positioned on both sides to ensure angle diversity of measurements and reduce lateral dilution of precision.

### Distance Data from Vehicle to Each RSU (`real_d.mat`)
Calculated based on the simulated positions of vehicles and RSUs, providing essential data for assessing localization algorithm effectiveness.

### Attenuation Factor Value (`A.mat`)
Adjusts signal propagation models dynamically to accommodate various environmental conditions, enhancing the framework's adaptability.

The datasets were specifically generated to tackle the three main challenges identified in our research: non-convex objective function optimization, adaptation to traffic scenarios' heterogeneity, and managing spatio-temporal constraints. These datasets not only serve as a benchmark for the CV2X-LOCA framework but also enable researchers to validate their own localization algorithms under similar conditions.


## Models
The repository hosts implementations of several localization models that we have compared in our paper. These models are integral to the comparative analysis we conducted to demonstrate the effectiveness of our proposed CV2X-LOCA framework in addressing specific challenges in vehicle localization.

The following models are included:
- WCL
- WLLS
- LS-EKF
- GRNN-UKF
- CV2X-LOCA (named SDP_A_UKF)

Each model addresses different aspects of the localization challenges as indicated in the table below:

<div align=center><img src="./Results/Table_1.jpg" width="600"></div>

_NOTE: The checkmarks in the table denote which challenges each method is focused on addressing, providing an at-a-glance comparison of their capabilities._

The code for each model is provided, allowing researchers to replicate our findings and further explore the performance of these models in their own simulations.


## Preparation
Before running the models, ensure that you have the necessary MATLAB packages installed. The models in this repository rely on various MATLAB functions and toolboxes.

For example:
- `ML-True` localization solutions are calculated using the MATLAB function `lsqnonlin`, which is part of the Optimization Toolbox.
- SDP-based methods, such as `SDP-LSRE`, `SDP-ML-KF`, and `CV2X-SDP` (our proposed method), are solved using the MATLAB package [CVX](http://cvxr.com/cvx/). Ensure that you have CVX installed with the solver SeDuMi, which can be done following the instructions on the [CVX installation page](http://cvxr.com/cvx/doc/install.html).

Please follow the installation guides provided by MATLAB and the respective package documentation to set up your environment correctly.

### Getting Started
To run the localization models and evaluate their performance, execute the `main.m` script in MATLAB.

Upon completion, the script will automatically calculate various error metrics for the localization run and save them in the same directory. These metrics provide a quantitative assessment of the model's performance, including:
- X-axis maximum error
- X-axis minimum error
- Y-axis maximum error
- Y-axis minimum error
- Mean Absolute Percentage Error (MAPE)
- Mean Absolute Error (MAE)
- Mean Error (MEAN)
- Root Mean Square Error (RMSE)

Make sure to check the output files for a detailed analysis of the model's accuracy and reliability.

## Simulation Results Display
Our research presents a comprehensive evaluation of the CV2X-LOCA framework, showcasing its proficiency in overcoming localization challenges. The simulation results demonstrate our method's effectiveness in various traffic and environmental scenarios:

- **Non-Convex Objective Function**: Our solution efficiently handles the complexity of non-convex problems to maintain high accuracy.
- **Heterogeneous Traffic Scenarios**: We simulate diverse conditions to prove the robustness of our approach in different traffic densities and urban settings.
- **Spatio-Temporal Constraints**: Our framework addresses the real-time demands of vehicular movements and delivers accurate location estimations even in rapidly changing conditions.

The following figures illustrate the localization performance of CV2X-LOCA compared to other methods under simulated conditions, highlighting the strengths and reliability of our approach:

<div align=center><img src="./Results/fig4.png" width="600"></div>
<div align=center><img src="./Results/Table_1.jpg" width="600"></div>

## Field Test Demonstration
Complementing our simulation results, we conducted field tests to validate the practical applicability of the CV2X-LOCA framework. Our field tests employed experimental vehicles equipped with the necessary instrumentation to capture and process localization data. The setup included:
- **Experimental Vehicles**: Equipped with On-Board Units (OBUs) and other necessary hardware.
- **Experimental Instruments**: Including computers, mobile power supplies, data cables, and OBUs for data acquisition and processing.
- **RSUs Deployment**: Strategically placed Roadside Units (RSUs) along the test route to facilitate accurate localization.

The vehicles traversed predefined routes covering various environments, such as tunnels and urban areas with tall buildings and trees, to assess the localization framework under different conditions.

<div align=center><img src="./Demo/fig10.png" width="600"></div>
<div align=center><img src="./Demo/Field_test.gif" width="600"></div>


## Project Structure
CV2X-LOCA/  
├── Algorithm/  
│ ├── CF/  
│ ├── CF_UKF/  
│ ├── GRNN/  
│ ├── GRNN_UKF/  
│ ├── LSE/  
│ ├── LSE_KF/  
│ ├── SDP/  
│ ├── SDP_A_UKF/  
│ ├── WCL/  
│ └── WLSE/  
├── Demo/  
├── Environment_setting/  
├── Results/  
├── LICENSE  
└── README.md  


## Reference
If you find this repo to be useful in your research, please consider citing our work.

*Huang, Z., Chen, S., Pian, Y., Sheng, Z., Ahn, S., & Noyce, D. A. (2023). CV2X-LOCA: Roadside Unit-Enabled Cooperative Localization Framework for Autonomous Vehicles. arXiv preprint arXiv:2304.00676.*


## Limitations
Due to time limitation, there are still some unstandardized parts of the code in this project. The author is working hard to maintain and update the code of CV2X-LOCA, please kindly follow up.

Thank you~


