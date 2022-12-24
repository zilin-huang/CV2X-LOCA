%%
% 本Test文档的目的：选取每个轨迹点最近的3个AP数据,并应用加权质心定位（WCL）
% 注意：N取最近3个点
%  AP_x_dim,  AP_y_dim,  AP_Z_dim 数据格式：143*3

clc;
clear;

load('D:\MATLAB\R2016b\bin\7. TITS\Test1_定位性能\车速7-衰减1-噪声1\Environment_setting\rssi_noise.mat')
load('D:\MATLAB\R2016b\bin\7. TITS\Test1_定位性能\车速7-衰减1-噪声1\Environment_setting\AP.mat')


%%
% 对数阴影衰减模型，根据RSSI值估计距离
% A=1+abs(normrnd(0,1.5)) %衰减因子matlab    本例子中为：2.6971
% 为不失一般性，统一为LSE的A
load('D:\MATLAB\R2016b\bin\7. TITS\Test1_定位性能\车速7-衰减1-噪声1\ML\A.mat')


intial_rssi=abs(-37.5721)
distance= 10.^((abs(rssi_noise)-intial_rssi)/(10 * A))               %所有轨迹点到第j个AP的估计距离
% save('onlinedata','rssi_noise','distance')                           % 143*68

%%
% LSE算法应用之前先选取3个最近的点
distance_sort=sort(distance,2)  %按行排序
for k=1:length(distance(:,1))
    index(k,:)=find(distance(k,:)<=distance_sort(k,4)) %计算出最小索引的四个数，设置这里的3
end
distance_dim=distance_sort(:,1:4)    %3个最近的点的估计距离，设置这里的3
% save('distance_dim','distance_dim')

%% 
% 3个最近的点的AP坐标位置
AP_x=AP(:,1)  %AP的x轴
AP_y=AP(:,2)  %AP的y轴
for n=1:length(distance(:,1))    %n为143,n为第n个定位点
    for m=1:length(index(1,:))    %m为3，总共有3列
       mm=index(n,m)   %取出第n行第m列的索引
       AP_x_dim(n,m)=AP_x(mm)   %对于第n个定位点，取出AP的x轴
       AP_y_dim(n,m)=AP_y(mm)   %对于第n个定位点，取出AP的y轴
    end
end
% save('AP_x_y_dim','AP_x_dim','AP_y_dim')

%%
% 三个最近位置的噪音
for n=1:length(distance(:,1))    %n为143,n为第n个定位点
    for m=1:length(index(1,:))    %m为3，总共有3列
        mm=index(n,m)   %取出第n行第m列的索引
        rssi_noise_dim(n,m)= rssi_noise(mm)
    end
end
AP_z_dim=zeros(length(AP_y_dim),size(AP_x_dim,2))  %AP的z轴（全为0）,如果不需要Z轴，则删除这一行
% save('rssi_noise_dim','rssi_noise_dim')

%%
% 加权质心算法
len=size(AP_x_dim,2)    
for i=1:length(distance_dim(:,1))  
    x1=0.1;
    y1=0.1;
    dq=0.02;  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 参数
    for j=1:len
        x1=x1+AP_x_dim(i,j)/distance_dim(i,j);
        y1=y1+AP_y_dim(i,j)/distance_dim(i,j);
        dq=dq+1/distance_dim(i,j);
    end
    distance_WCL(:,i)= [x1/dq;y1/dq];
end

%%
% 计算误差
load('D:\MATLAB\R2016b\bin\7. TITS\Test1_定位性能\车速7-衰减1-噪声1\Environment_setting\trace_1.mat')

error_WCL=sqrt(sum((distance_WCL(1:2,:)-trace_1').^2))./2   %误差

diff = abs(distance_WCL(1:2,:)-trace_1')  % 预测值与真实值的差距
error_WCL_high_x = max(max(diff(1,:)))   % x轴最大误差
error_WCL_low_x = min(min(diff(1,:)))   % x轴最小误差
error_WCL_high_y = max(max(diff(2,:)))   % y轴最大误差
error_WCL_low_y = min(min(diff(2,:)))   % y轴最小误差

mean_error_WCL=mean(error_WCL)            %定位误差

rmse_error_WCL=(sqrt(mean((distance_WCL(1,:)-trace_1(:,1)').^2))+sqrt(mean((distance_WCL(2,:)-trace_1(:,2)').^2)))/2  % RMSE
rmse_error_WCL_high_x=sqrt(mean((distance_WCL(1,:)-trace_1(:,1)').^2))   % x轴的RMSE
rmse_error_WCL_high_y=sqrt(mean((distance_WCL(2,:)-trace_1(:,2)').^2))   % y轴的RMSE

mae_error_WCL= (mean(abs((distance_WCL(1,:)-trace_1(:,1)'))+mean(abs((distance_WCL(2,:)-trace_1(:,2)'))))/2)     % MAE
mape_error_WCL= (mean(abs((distance_WCL(1,:)-trace_1(:,1)')./trace_1(:,1)'))+mean(abs((distance_WCL(2,:)-trace_1(:,2)')./trace_1(:,2)')))/2   %MAPE

%%
% 如果本次运行还不错的话，保存相关参数
save('A','A') 
save('onlinedata','rssi_noise','distance')                           % 143*68
save('distance_dim','distance_dim')
save('AP_x_y_dim','AP_x_dim','AP_y_dim')
save('rssi_noise_dim','rssi_noise_dim')
save('distance_WCL','distance_WCL')
save('error','error_WCL','error_WCL_high_x','error_WCL_low_x','error_WCL_high_y','error_WCL_low_y','mean_error_WCL','rmse_error_WCL','rmse_error_WCL_high_x','rmse_error_WCL_high_y','mae_error_WCL','mape_error_WCL')    

%%
totall_error = [mean_error_WCL,error_WCL_high_x,error_WCL_low_x,error_WCL_high_y,error_WCL_low_y,rmse_error_WCL,rmse_error_WCL_high_x,rmse_error_WCL_high_y,mae_error_WCL,mape_error_WCL]
save('totall_error','totall_error')

clc;
clear;
load('D:\MATLAB\R2016b\bin\7. TITS\Test1_定位性能\车速7-衰减1-噪声1\WCL\totall_error.mat')