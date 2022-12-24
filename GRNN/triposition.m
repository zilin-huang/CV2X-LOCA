function [locx,locy] = triposition(xa,ya,da,xb,yb,db,xc,yc,dc)
%              三点定位法                          %
%输入：
%   1.参考节点A（xa,ya）,B(xb,yb),C(xc,yc)
%   2.定位节点D(locx,locy)到这三点的距离分别为da,db,dc
%返回：
%   （locx，locy)为计算的定位节点D点的位置坐标
%
syms x y   %f符号变量
%--------------求解方程组------------------------------------
f1 = '2*x*(xa-xc)+xc^2-xa^2+2*y*(ya-yc)+yc^2-ya^2=dc^2-da^2';
f2 = '2*x*(xb-xc)+xc^2-xb^2+2*y*(yb-yc)+yc^2-yb^2=dc^2-db^2';
% 解关于x,y的符号方程组，得到解的符号表示，并存入xx,yy
[xx,yy] = solve(f1,f2,x,y); 
px = eval(xx);  %解的数值px(1),px(2)
py = eval(yy);  %解的数值py(1),py(2)
locx = px;
locy = py;