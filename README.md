# CV2X-SDP
This repo is the implementation of the paper "Roadside Units-Enabled Cooperative Localization Scheme for Autonomous Vehicles Under GNSS-denied Environments: Methodology and Demonstration" (Under Review).

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
- CV2X-SDP (named SDP_A_UKF)

Note: The remaining models compared in the paper can be easily deduced based on the given code. Therefore, the authors do not list them all here.

# Getting started
Install the dependent package

e.g., 

`ML-True, are solved by the MATLAB function lsqnonlin`
- The SDP-based methods, including SDP-LSRE, SDP-ML-KF, and CV2X-SDP (our), are solved by using the MATLAB package CVX , which the solver is SeDuMi
