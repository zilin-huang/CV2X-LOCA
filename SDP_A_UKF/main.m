clc;
clear;

T=1;            % 采样周期
load('D:\MATLAB\R2016b\bin\7. TITS\Test1_定位性能\车速7-衰减1-噪声1\SDP_A\distance_SDP_A.mat')
N= length(distance_SDP_A);        % 采样次数
X=zeros(4,N);   % 初始化真实轨迹矩阵   4*60

% load('D:\MATLAB\R2016b\bin\7. TITS\Test1_定位性能\车速7-衰减1-噪声1\Environment_setting\trace_1.mat')
distance_SDP_A_T = distance_SDP_A'
% X(:,1)=[trace_1(1,1),trace_1(2,1)-trace_1(1,1),trace_1(1,2),trace_1(2,2)-trace_1(1,2)];    % 目标初始位置、速度     4*60 第一列是初始位置
X(:,1)=[distance_SDP_A_T(1,1),distance_SDP_A_T(2,1)-distance_SDP_A_T(1,1),distance_SDP_A_T(1,2),distance_SDP_A_T(2,2)-distance_SDP_A_T(1,2)];    % 目标初始位置、速度     4*60 第一列是初始位置

Z=zeros(2,N);   % 初始化 观测距离矩阵  2*60
Z(:,1) = [distance_SDP_A(1,1),distance_SDP_A(2,1)];

delta_w=1e-2; 
Q=delta_w*diag([2,2.5,2,2.5]) ;   % 过程噪声协方差矩阵   4*4 对角是0.5,1,0.5,1

R=100*eye(2);   % 观测噪声协方差  2*2 对角是100 100
W = sqrtm(Q)*randn(4,N);   % v均值为0，协方差为R的高斯白噪声 %sqrt(A)矩阵每个元素分别开方,与矩阵点乘有关;sqrtm(A)矩阵为整体,与矩阵相乘有关
V = sqrt(R)*randn(2,N);    % W均值为0，协方差为Q的高斯白噪声

F=[1,T,0,0;0,1,0,0;0,0,1,T;0,0,0,1];  % 状态转移矩阵  4*4
H = [1,0,0,0;0,0,1,0]; 

%%

for t=2:N
    X(:,t)=F*X(:,t-1)+W(:,t-1);      % 目标的真实轨迹    4*60
end

for t=1:N
    VV(:,t)=distance_SDP_A(:,t)-H*X(:,t)   
    Z(:,t)=H*X(:,t)+VV(:,t);         % 测试轨迹
end

R_1 = var(VV)

%%
% UKF滤波参数
L=4;                             % L为状态维度
alpha=1;                         % a控制采样点的分布状态
kalpha=0;                        % k为待选参数 没有界限，但是要保证（n+lamda）*P为半正定矩阵
belta=2;                         % b非负的权系数
ramda=alpha*alpha*(L+kalpha)-L;  % lambda为缩放比列参数，用于降低总的预测误差

% sigma点的权值
for j=1:2*L+1
    Wm(j)=1/(2*(L+ramda));
    Wc(j)=1/(2*(L+ramda));
end
Wm(1)=ramda/(L+ramda);
Wc(1)=ramda/(L+ramda)+1-alpha^2+belta;


%%
% UKF滤波
Xukf=zeros(4,N);      % 4*60
Xukf(:,1)=X(:,1);     %无迹Kalman滤波状态初始化
P0=10*eye(4);         %协方差阵初始化    对角线为10

for t=2:N
    xestimate= Xukf(:,t-1);
    P=P0;
    
%%%%%%% 第一步:获得一组采样点，Sigma点集 %%%%%%%%%%%%%%    
    cho=(chol(P*(L+ramda)))';  % cho=chol(X)用于对矩阵X进行Cholesky分解，产生一个上三角阵cho，使cho'cho=X。若X为非对称正定，则输出一个出错信息。
    xgamaP1 = zeros(4,L);
    xgamaP2 = zeros(4,L);
    for k=1:L
        xgamaP1(:,k)=xestimate+cho(:,k);
        xgamaP2(:,k)=xestimate-cho(:,k);
    end
    Xsigma=[xestimate xgamaP1 xgamaP2];    % 获得 2L+1 个Sigma点

%第二步:对Sigme点集进行一步预测   
    Xsigmapre=F*Xsigma;

%第三步：利用第二部结果计算均值和协方差
    Xpred=zeros(4,1);
    for k=1:2*L+1
        Xpred=Xpred+Wm(k)*Xsigmapre(:,k);
    end
    Ppred=zeros(4,4);    %协方差阵预测
    for k=1:2*L+1
        Ppred=Ppred+Wc(k)*(Xsigmapre(:,k)-Xpred(:,1))*(Xsigmapre(:,k)-Xpred(:,1))'; %%%%%%%%%%%%%%%%%
    end
    Ppred=Ppred+Q;
 
%第四步:根据预测值，再次使用UT变换，得到新的sigma点集
    chor=(chol((L+ramda)*Ppred))';
    XaugsigmaP1 = zeros(4,L);
    XaugsigmaP2 = zeros(4,L);
    for k=1:L
        XaugsigmaP1(:,k)=Xpred+chor(:,k);
        XaugsigmaP2(:,k)=Xpred-chor(:,k);
    end
    Xaugsigma=[Xpred XaugsigmaP1 XaugsigmaP2]; 
    
 %第五步:观测预测   
    Zsigmapre = zeros(2,2*L+1);
    for k=1:2*L+1
        Zsigmapre(:,k)=H*Xaugsigma(:,k);%%%观测预测%%%
    end
      
 %第六步:计算观测预测均值和协方差   
    Zpred=zeros(2,1);  %观测预测的均值
    for k=1:2*L+1
        Zpred=Zpred+Wm(k)*Zsigmapre(:,k);
    end
    Pzz=zeros(2,1);
    for k=1:2*L+1
        Pzz=Pzz+Wc(k)*(Zsigmapre(:,k)-Zpred(:,1))*(Zsigmapre(:,k)-Zpred(:,1))';
    end
    Pzz=Pzz+R;
    
    
    Pxz=0;
    for k=1:2*L+1
        Pxz=Pxz+Wc(k)*(Xaugsigma(:,k)-Xpred(:,1))*(Zsigmapre(:,k)-Zpred(:,1))';
    end
    
 %第七步:计算Kalman增益   
    K=Pxz*inv(Pzz);  % Kalman
    
%第八步:状态和方差更新    
    xestimate=Xpred+K*(Z(:,t)-Zpred(:,1));  % 状态更新
    P=Ppred-K*Pzz*K';%方差更新
    P0=P;
    
    Xukf(:,t)=xestimate;
end

distance_SDP_A_UKF = [Xukf(1,:);Xukf(3,:)]

%%
% 计算误差
load('D:\MATLAB\R2016b\bin\7. TITS\Test1_定位性能\车速7-衰减1-噪声1\Environment_setting\trace_1.mat')

error_SDP_A_UKF=sqrt(sum((distance_SDP_A_UKF(1:2,:)-trace_1').^2))./2   %误差

diff = abs(distance_SDP_A_UKF(1:2,:)-trace_1')  % 预测值与真实值的差距
error_SDP_A_UKF_high_x = max(max(diff(1,:)))   % x轴最大误差
error_SDP_A_UKF_low_x = min(min(diff(1,:)))   % x轴最小误差
error_SDP_A_UKF_high_y = max(max(diff(2,:)))   % y轴最大误差
error_SDP_A_UKF_low_y = min(min(diff(2,:)))   % y轴最小误差

mean_error_SDP_A_UKF=mean(error_SDP_A_UKF)            %定位误差

rmse_error_SDP_A_UKF=(sqrt(mean((distance_SDP_A_UKF(1,:)-trace_1(:,1)').^2))+sqrt(mean((distance_SDP_A_UKF(2,:)-trace_1(:,2)').^2)))/2  % RMSE
rmse_error_SDP_A_UKF_high_x=sqrt(mean((distance_SDP_A_UKF(1,:)-trace_1(:,1)').^2))   % x轴的RMSE
rmse_error_SDP_A_UKF_high_y=sqrt(mean((distance_SDP_A_UKF(2,:)-trace_1(:,2)').^2))   % y轴的RMSE

mae_error_SDP_A_UKF= (mean(abs((distance_SDP_A_UKF(1,:)-trace_1(:,1)'))+mean(abs((distance_SDP_A_UKF(2,:)-trace_1(:,2)'))))/2)     % MAE
mape_error_SDP_A_UKF= (mean(abs((distance_SDP_A_UKF(1,:)-trace_1(:,1)')./trace_1(:,1)'))+mean(abs((distance_SDP_A_UKF(2,:)-trace_1(:,2)')./trace_1(:,2)')))/2   %MAPE

%%
% 如果本次运行还不错的话，保存相关参数
% save('A','A') 
% save('onlinedata','rssi_noise','distance')                           % 143*68
% save('distance_dim','distance_dim')
% save('AP_x_y_dim','AP_x_dim','AP_y_dim')
% save('rssi_noise_dim','rssi_noise_dim')
save('distance_SDP_A_UKF','distance_SDP_A_UKF')
save('error','error_SDP_A_UKF','error_SDP_A_UKF_high_x','error_SDP_A_UKF_low_x','error_SDP_A_UKF_high_y','error_SDP_A_UKF_low_y','mean_error_SDP_A_UKF','rmse_error_SDP_A_UKF','rmse_error_SDP_A_UKF_high_x','rmse_error_SDP_A_UKF_high_y','mae_error_SDP_A_UKF','mape_error_SDP_A_UKF')    

totall_error = [mean_error_SDP_A_UKF,error_SDP_A_UKF_high_x,error_SDP_A_UKF_low_x,error_SDP_A_UKF_high_y,error_SDP_A_UKF_low_y,rmse_error_SDP_A_UKF,rmse_error_SDP_A_UKF_high_x,rmse_error_SDP_A_UKF_high_y,mae_error_SDP_A_UKF,mape_error_SDP_A_UKF]
save('totall_error','totall_error')

%%

clc;
clear;
load('D:\MATLAB\R2016b\bin\7. TITS\Test1_定位性能\车速7-衰减1-噪声1\SDP_A_UKF\totall_error.mat')

