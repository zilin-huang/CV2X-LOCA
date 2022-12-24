%%
% 本Test文档的目的：选取每个轨迹点最近的4个AP数据,并应用GRNN+UKF
% 注意：N取最近4个点
%  AP_x_dim,  AP_y_dim,  AP_Z_dim 数据格式：143*4

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

% (假设这个距离数值就是GRNN要学习的数值)

%%
% LSE算法应用之前先选取4个最近的点
distance_sort=sort(distance,2)  %按行排序
for k=1:length(distance(:,1))
    index(k,:)=find(distance(k,:)<=distance_sort(k,3)) %计算出最小索引的四个数，设置这里的4
end
distance_dim=distance_sort(:,1:3)    %4个最近的点的估计距离，设置这里的3
% save('distance_dim','distance_dim')

%% 
% 4个最近的点的AP坐标位置
AP_x=AP(:,1)  %AP的x轴
AP_y=AP(:,2)  %AP的y轴
for n=1:length(distance(:,1))    %n为143,n为第n个定位点
    for m=1:length(index(1,:))    %m为4，总共有4列
       mm=index(n,m)   %取出第n行第m列的索引
       AP_x_dim(n,m)=AP_x(mm)   %对于第n个定位点，取出AP的x轴
       AP_y_dim(n,m)=AP_y(mm)   %对于第n个定位点，取出AP的y轴
    end
end
% save('AP_x_y_dim','AP_x_dim','AP_y_dim')

%%
% 四个最近位置的噪音
for n=1:length(distance(:,1))    %n为143,n为第n个定位点
    for m=1:length(index(1,:))    %m为4，总共有4列
        mm=index(n,m)   %取出第n行第m列的索引
        rssi_noise_dim(n,m)= rssi_noise(mm)
    end
end
% save('rssi_noise_dim','rssi_noise_dim')

%%
% GRNN
desired_spread_grnn=0.7     %光滑因子
net_grnn=newgrnn(rssi_noise,distance,desired_spread_grnn);   %训练网络，得到net_grnn

predict_distance_GRNN=sim(net_grnn,rssi_noise_dim)          %预测网络:A1设备

%%
% 三点定位法
for p=1:length(predict_distance_GRNN(:,1))  %d代表第d个轨迹点
    X=AP_x_dim(p,:)
    Y=AP_y_dim(p,:)
    xa = X(:,1)
    xb = X(:,2)
    xc = X(:,3)
    ya = Y(:,1)
    yb = Y(:,2)
    yc = Y(:,3)
    d_tri=predict_distance_GRNN(p,:)'  %将距离从行向量变成列向量
    da = d_tri(1,:)
    db = d_tri(2,:)
    dc = d_tri(3,:)
    [locx,locy]=triposition(xa,ya,da,xb,yb,db,xc,yc,dc) 
    distance_GRNN(:,p)=[locx,locy]
end

%%
% 计算误差
load('D:\MATLAB\R2016b\bin\7. TITS\Test1_定位性能\车速7-衰减1-噪声1\Environment_setting\trace_1.mat')

% distance_GRNN = distance_GRNN'

error_GRNN=sqrt(sum((distance_GRNN(1:2,:)-trace_1').^2))./2   %误差

diff = abs(distance_GRNN(1:2,:)-trace_1')  % 预测值与真实值的差距
error_GRNN_high_x = max(max(diff(1,:)))   % x轴最大误差
error_GRNN_low_x = min(min(diff(1,:)))   % x轴最小误差
error_GRNN_high_y = max(max(diff(2,:)))   % y轴最大误差
error_GRNN_low_y = min(min(diff(2,:)))   % y轴最小误差

mean_error_GRNN=mean(error_GRNN)            %定位误差

rmse_error_GRNN=(sqrt(mean((distance_GRNN(1,:)-trace_1(:,1)').^2))+sqrt(mean((distance_GRNN(2,:)-trace_1(:,2)').^2)))/2  % RMSE
rmse_error_GRNN_high_x=sqrt(mean((distance_GRNN(1,:)-trace_1(:,1)').^2))   % x轴的RMSE
rmse_error_GRNN_high_y=sqrt(mean((distance_GRNN(2,:)-trace_1(:,2)').^2))   % y轴的RMSE

mae_error_GRNN= (mean(abs((distance_GRNN(1,:)-trace_1(:,1)'))+mean(abs((distance_GRNN(2,:)-trace_1(:,2)'))))/2)     % MAE
mape_error_GRNN= (mean(abs((distance_GRNN(1,:)-trace_1(:,1)')./trace_1(:,1)'))+mean(abs((distance_GRNN(2,:)-trace_1(:,2)')./trace_1(:,2)')))/2   %MAPE

%%
% 如果本次运行还不错的话，保存相关参数
save('A','A') 
save('onlinedata','rssi_noise','distance')                           % 143*68
save('distance_dim','distance_dim')
save('AP_x_y_dim','AP_x_dim','AP_y_dim')
save('rssi_noise_dim','rssi_noise_dim')
save('distance_GRNN','distance_GRNN')
save('error','error_GRNN','error_GRNN_high_x','error_GRNN_low_x','error_GRNN_high_y','error_GRNN_low_y','mean_error_GRNN','rmse_error_GRNN','rmse_error_GRNN_high_x','rmse_error_GRNN_high_y','mae_error_GRNN','mape_error_GRNN')    

%%
totall_error = [mean_error_GRNN,error_GRNN_high_x,error_GRNN_low_x,error_GRNN_high_y,error_GRNN_low_y,rmse_error_GRNN,rmse_error_GRNN_high_x,rmse_error_GRNN_high_y,mae_error_GRNN,mape_error_GRNN]
save('totall_error','totall_error')

clc;
clear;
load('D:\MATLAB\R2016b\bin\7. TITS\Test1_定位性能\车速7-衰减1-噪声1\GRNN\totall_error.mat')

