# CV2X-LOCA
This repo is the implementation of the paper "Toward C-V2X Enabled Connected Transportation System: RSU-Based Cooperative Localization Framework for Autonomous Vehicles".


# Overview
CV2X-LOCA innovatively uses C-V2X channel state information to achieve lane-level positioning accuracy. 

<div align=center><img src=./Demo/fig3.png width="600"></div>

It is composed of four modules:
- Data Processing Module: Standardizes and processes incoming data for accurate localization.
- Coarse Positioning Module: Overcomes non-convex optimization challenges by approximating ML estimators, ensuring robust vehicle localization.
- Environment Parameter Correcting Module: Adapts to environmental variations by dynamically adjusting signal propagation models, crucial for handling traffic scenarios heterogeneity.
- Vehicle Trajectory Filtering Module: Enhances localization precision through advanced filtering techniques, addressing spatio-temporal constraints effectively.




<div align=center><img src=https://github.com/julianwong-source/CV2X-LOCA/blob/main/Field_test.gif width="600"></div>


# Data
It contains various simulation data generated in Matlab environment (under **Environment setting** file). Researchers can use these data to test the localization algorithms they have developed. 

- RSS data (named rssi_noise.mat)
- Vehicle trajectory data (named trace_1.mat)
- RSUs coordinate data (named AP.mat)
- Distance data from vehicle to each RSU (named real_d.mat)
- Attenuation factor value (named A.mat)

# Models
This repo contains several major types of models compared in our paper, such as:
- WCL
- WLLS
- LS-EKF
- GRNN-UKF
- CV2X-LOCA (named SDP_A_UKF)

Note: The remaining models compared in the paper can be easily deduced based on the given code.

# Preparation
Install the dependent package

e.g., 

`ML-True, are solved by the MATLAB function lsqnonlin`

`The SDP-based methods, including SDP-LSRE, SDP-ML-KF, and CV2X-SDP (our), are solved by using the MATLAB package CVX , which the solver is SeDuMi.`

# Getting started
Run model

``` 
main.m
``` 

After finishing, the module will automatically calculate the error of this run and save it in the same directory. Including:
- X-axis maximum error
- X-axis minimum error
- Y-axis maximum error
- Y-axis minimum error
- MAPE
- MAE
- MEAN
- RMSE


# Project Structure
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


# Reference
If you find this repo to be useful in your research, please consider citing our work.

*Huang, Z., Chen, S., Pian, Y., Sheng, Z., Ahn, S., & Noyce, D. A. (2023). CV2X-LOCA: Roadside Unit-Enabled Cooperative Localization Framework for Autonomous Vehicles. arXiv preprint arXiv:2304.00676.*




# Limitations
Due to time limitation, there are still some unstandardized parts of the code in this project. The author is working hard to maintain and update the code of CV2X-LOCA, please kindly follow up.

Thank you~


