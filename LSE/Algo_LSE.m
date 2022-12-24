function [estimated_position]=Algo_LSE(X,Y,Z,D)

Num_temp=size(X);
Q=eye(Num_temp(2));
for i=1:Num_temp(2)
    K(i)=X(i)^2+Y(i)^2+Z(i)^2;
    h(i,1)=D(i)^2-K(i);
    Ga(i,:)=[-2*X(i),-2*Y(i),-2*Z(i),1];
end
Za0=pinv(Ga'*inv(Q)*Ga)*Ga'*inv(Q)*h;
estimated_position=Za0(1:3);
end
