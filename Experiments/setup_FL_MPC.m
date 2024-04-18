clear; close all;

Ts=0.01; %Sampling times
Ts2=Ts;
Ts_data=0.01; 

l=0.256; %wheelbase


plot_data=false; %true to plot trajectory



% %Control parameters
K=7*eye(2);
Delta=0.35;
% N=8; %good behaviour
N=10;
N_sides=10; %number of sides of the polyhedral approximation
qz=1; qu=0.01;



%Active Set solver Parameters
Phi=repmat(eye(2),N,1);
Theta=Ts*repmat(eye(2),N);
Theta=tril(Theta);
Qhat=qz*eye(2*N);
Rhat=qu*eye(2*N);
H_obj=Theta'*Qhat*Theta+Rhat;

[L,p] = chol(H_obj,'lower');
Linv = linsolve(L,eye(size(L)),struct('LT',true));



%Filter parameters
% Q_kalman=1e-10*eye(4);
% R_kalman=1e-12*eye(4);
% Po_kalman=1e-8*eye(4);

%Filter parameters
alpha_kalman=0.9;
beta_kalman=2;
kappa_kalman=0;
%
%
% Q_kalman=diag([0.001 0.001 0.1 1]);
% R_kalman=diag([2e-5 2e-5 0.0001 0.0001]);
% P0_kalman=diag([1e-5 1e-5 1e-6 1e-6]);


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



%Worst case feasible input set

phi_worst=0.6;

r1=sqrt((Delta^2*l^2*v2max^2)/(Delta^2-Delta^2*cos(phi_worst)^2+l^2));
r2=sqrt(v1max^2/cos(0)^2);
r_u=min(r1,r2);


Q_u=(1/r_u^2)*eye(2);
ell_u=ellipsoid(inv(Q_u));

if plot_data
    figure
    plot(ell_u)
    hold on
end


V_oct=zeros(N_sides,2);
for i=1:N_sides

    V_oct(i,1)=r_u*cos(i*(2*pi/N_sides));
    V_oct(i,2)=r_u*sin(i*(2*pi/N_sides));

end
%
P_worst=Polyhedron(V_oct);


H_w=P_worst.A;
g_w=P_worst.b;
H_w=H_w/g_w(1);
g_w=ones(N_sides,1);

P_worst=Polyhedron(H_w,g_w);

if plot_data
    plot(P_worst)
end



%Terminal set
S=1/r_u^2*(K'*K);
r_N=sqrt(1/S(1,1));
inv_ell=ellipsoid(inv(S));
if plot_data
    figure
    plot(inv_ell)
    hold on
end

V_PN=zeros(N_sides,2);
for i=1:N_sides
    V_PN(i,1)=r_N*cos(i*(2*pi/N_sides));
    V_PN(i,2)=r_N*sin(i*(2*pi/N_sides));

end
%
P_N=Polyhedron(V_PN);
H_N=P_N.A;
g_N=P_N.b;
P_N=Polyhedron(H_N,g_N);

if plot_data
    plot(P_N,'Color','green')
end


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


