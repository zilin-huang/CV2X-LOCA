# CV2X-LOCA
This repo is the implementation of the paper "CV2X-LOCA: Roadside Unit-Enabled Cooperative Localization Framework for Autonomous Vehicles".

![Field Test](https://raw.github.com/julianwong-source/repositpry/master/CV2X-LOCA/Field Test-min.gif)

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


# Reference
If you find this repo to be useful in your research, please consider citing our work.

*CV2X-LOCA: Roadside Unit-Enabled Cooperative Localization Framework for Autonomous Vehicles. Zilin Huang, Sikai Chen (Corresponding author), Yuzhuang Pian, ZIhao Sheng, Soyoung Ahn, David Noyce.*




# Limitations
Due to time limitation, there are still some unstandardized parts of the code in this project. The author is working hard to maintain and update the code of CV2X-LOCA, please kindly follow up.

Thank you~


