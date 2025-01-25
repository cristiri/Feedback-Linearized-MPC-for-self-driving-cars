clear; close all;


%Sampling times 
Ts=0.01;
Ts2=Ts;
Ts_data=0.01;

l=0.256;  


plot_data=false;

N=10;
qz=1; qu=0.002;

% Decent performance
Q=[1 0 0 0; 0 1 0 0; 0 0 0.1 0; 0 0 0 0.1];
R=diag([0.3 0.1]);
R2=diag([0 0]);


Qhat=[]; Rhat=[];

for i=1:N
    Qhat=blkdiag(Qhat,Q);
    Rhat=blkdiag(Rhat,R);
end




%
% N=5;
% qz=1; qu=0.001;



%Filter parameters
alpha_kalman=0.9;
beta_kalman=2;
kappa_kalman=0;



Q_kalman=diag([0.001 0.001 0.1 10]);
R_kalman=diag([2e-5 2e-5 0.0001 0.00001]);
P0_kalman=diag([1e-5 1e-5 1e-6 1e-6]);


%Transfer function servomotor
s=tf('s');
G=1/(0.1026*s+1);
Gd=c2d(G,Ts);
[num_c,den_c] = tfdata(Gd);
num = cell2mat(num_c);
den=cell2mat(den_c);
n1=num(2);
d1=den(2);


%Constraints
v1max=1; %Driving velocity limit (m/s)
v2max=10; %Steering velocity limit (rad/s)
phi_max=0.6;

T=[-1 0; 0 -1; 1 0; 0 1];
g=[v1max;v2max;v1max;v2max];




trajectory_setup

load('trajectory.mat')



% Getting Car's initial pose from Vicon
vicon_port=18099;
vicon_address="192.168.0.5";
vicon_client=tcpclient(vicon_address,vicon_port);
vicon_data=read(vicon_client,7,"double");
flush(vicon_client);
x0=vicon_data(1);y0=vicon_data(2); theta0=vicon_data(6);
q0=[x0;y0;theta0;0];
clear vicon_client;
q0c=[x0+(l/2)*cos(theta0);y0+(l/2)*sin(theta0);theta0];
q0v=[vicon_data(1);vicon_data(2);unwrap(vicon_data(6))];


