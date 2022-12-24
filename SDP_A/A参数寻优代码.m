%%
% 本Test文档的目的：选取每个轨迹点最近的4个AP数据,并应用半定规划（SDP）
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
% load('D:\MATLAB\R2016b\bin\7. TITS\Test1_定位性能\车速7-衰减1-噪声1\ML\A.mat')

A = 1
uu =1

while A<=5
    
    intial_rssi=abs(-37.5721)
    distance= 10.^((abs(rssi_noise)-intial_rssi)/(10 * A))               %所有轨迹点到第j个AP的估计距离
    % save('onlinedata','rssi_noise','distance')                           % 143*68
    
    %%
    % SDP算法应用之前先选取4个最近的点
    distance_sort=sort(distance,2)  %按行排序
    for k=1:length(distance(:,1))
        index(k,:)=find(distance(k,:)<=distance_sort(k,4)) %计算出最小索引的四个数，设置这里的4
    end
    distance_dim=distance_sort(:,1:4)    %4个最近的点的估计距离，设置这里的3
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
    AP_z_dim=zeros(length(AP_y_dim),size(AP_x_dim,2))  %AP的z轴（全为0）,如果不需要Z轴，则删除这一行
    % save('rssi_noise_dim','rssi_noise_dim')
    
    %%
    % 半定规划(SDP)求解
    for i=1:length(distance(:,1))    %n为200,n为第n个定位点
        
        cvx_begin sdp quiet
        variable y(2,1)
        variable  x(2,2)
        variable t(4,1)
        
        minimize norm(t)
        subject to
        trace(x)-2*([AP_x_dim(i,1),AP_y_dim(i,1)])*y+AP_x_dim(i,1).^2+AP_y_dim(i,1).^2 <=  10.^((abs(rssi_noise_dim(i,1))-intial_rssi)/(10 * A))*t(1,1)
        trace(x)-2*([AP_x_dim(i,2),AP_y_dim(i,2)])*y+AP_x_dim(i,2).^2+AP_y_dim(i,2).^2<=  10.^((abs(rssi_noise_dim(i,2))-intial_rssi)/(10 * A))*t(2,1)
        trace(x)-2*([AP_x_dim(i,3),AP_y_dim(i,3)])*y+AP_x_dim(i,3).^2+AP_y_dim(i,3).^2 <=  10.^((abs(rssi_noise_dim(i,3))-intial_rssi)/(10 * A))*t(3,1)
        trace(x)-2*([AP_x_dim(i,4),AP_y_dim(i,4)])*y+AP_x_dim(i,4).^2+AP_y_dim(i,4).^2 <=  10.^((abs(rssi_noise_dim(i,4))-intial_rssi)/(10 * A))*t(4,1)
        
        %     trace(x)-2*([AP_x_dim(i,1),AP_y_dim(i,1)])*y+norm([AP_x_dim(i,1);AP_y_dim(i,1)])<=  10.^((abs(rssi_noise_dim(i,1))-intial_rssi)/(10 * A))*t(1,1)
        %     trace(x)-2*([AP_x_dim(i,2),AP_y_dim(i,2)])*y+norm([AP_x_dim(i,2);AP_y_dim(i,2)])<=  10.^((abs(rssi_noise_dim(i,2))-intial_rssi)/(10 * A))*t(1,1)
        %     trace(x)-2*([AP_x_dim(i,3),AP_y_dim(i,3)])*y+norm([AP_x_dim(i,3);AP_y_dim(i,3)])<=  10.^((abs(rssi_noise_dim(i,3))-intial_rssi)/(10 * A))*t(1,1)
        %     trace(x)-2*([AP_x_dim(i,4),AP_y_dim(i,4)])*y+norm([AP_x_dim(i,4);AP_y_dim(i,4)])<=  10.^((abs(rssi_noise_dim(i,4))-intial_rssi)/(10 * A))*t(1,1)
        
        
        [trace(x)-2*([AP_x_dim(i,1),AP_y_dim(i,1)])*y+AP_x_dim(i,1).^2+AP_y_dim(i,1).^2,sqrt(10.^((abs(rssi_noise_dim(i,1))-intial_rssi)/(10 * A)));
            sqrt(10.^((abs(rssi_noise_dim(i,1))-intial_rssi)/(10 * A))), t(1,1)]>=0
        [trace(x)-2*([AP_x_dim(i,2),AP_y_dim(i,2)])*y+AP_x_dim(i,2).^2+AP_y_dim(i,2).^2,sqrt(10.^((abs(rssi_noise_dim(i,2))-intial_rssi)/(10 * A)));
            sqrt(10.^((abs(rssi_noise_dim(i,2))-intial_rssi)/(10 * A))), t(2,1)]>=0
        [trace(x)-2*([AP_x_dim(i,3),AP_y_dim(i,3)])*y+AP_x_dim(i,3).^2+AP_y_dim(i,3).^2,sqrt(10.^((abs(rssi_noise_dim(i,3))-intial_rssi)/(10 * A)));
            sqrt(10.^((abs(rssi_noise_dim(i,3))-intial_rssi)/(10 * A))), t(3,1)]>=0
        [trace(x)-2*([AP_x_dim(i,4),AP_y_dim(i,4)])*y+AP_x_dim(i,4).^2+AP_y_dim(i,4).^2,sqrt(10.^((abs(rssi_noise_dim(i,4))-intial_rssi)/(10 * A)));
            sqrt(10.^((abs(rssi_noise_dim(i,4))-intial_rssi)/(10 * A))), t(4,1)]>=0
        
        [x,y;y',1]>=0
        
        cvx_end;
        
        distance_SDP(:,i)= y
        
    end
    
    %%
    % 计算误差
    load('D:\MATLAB\R2016b\bin\7. TITS\Test1_定位性能\车速7-衰减1-噪声1\Environment_setting\trace_1.mat')
    
    error_SDP=sqrt(sum((distance_SDP(1:2,:)-trace_1').^2))./2   %误差
    
    mean_error_SDP=mean(error_SDP)            %定位误差
    
    AA(uu,:)=mean_error_SDP

A = A+0.5
uu = uu+1
end 

save('AA','AA')