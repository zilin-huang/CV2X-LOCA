function [estimated_position]=Algo_WLSE(X,Y,Z,D)

Num_temp=size(X);

Q=eye(Num_temp(2));
for i=1:Num_temp(2)
    K(i)=X(i)^2+Y(i)^2+Z(i)^2;
    h(i,1)=D(i)^2-K(i);
    Ga(i,:)=[-2*X(i),-2*Y(i),-2*Z(i),1];
end

Za0=pinv(Ga'*inv(Q)*Ga)*Ga'*inv(Q)*h;

% FI is weight matrix - 1
B=eye(Num_temp(2));
for i=1:Num_temp(2)
    B(i,i)=sqrt((X(i)-Za0(1))^2+(Y(i)-Za0(2))^2+(Z(i)-Za0(3))^2);
end
FI = B*Q*B;
Za1=pinv(Ga'*inv(FI)*Ga)*Ga'*inv(FI)*h;
estimated_position=Za1(1:3);
end
