clear; close all;
Ts=0.1;


%Defining eight-shaped Trajectory
alpha=3;
beta=1.7;

Tf=4*pi*beta;
t=0:Ts:Tf;

l=0.256;

xr=alpha*sin(t/beta);
yr=alpha*sin(t/(2*beta));

xdr=alpha*cos(t/beta)*(1/(beta));
ydr=alpha*cos(t/(2*beta))*(1/(2*beta));


xddr=-alpha*sin(t/beta)*(1/(beta))^2;
yddr=-alpha*sin(t/(2*beta))*(1/(2*beta))^2;

xdddr=-alpha*cos(t/beta)*(1/(beta))^3;
ydddr=-alpha*cos(t/(2*beta))*(1/(2*beta))^3;




%Driving velocity reference
v1r=sqrt(xdr.^2+ydr.^2); %Linear velocity of the trajectory


%Trajectory Orientation
% thetar=unwrap(atan2(ydr./v1r,xdr./v1r));

thetar=unwrap(atan2(ydr./v1r,xdr./v1r));




%Vehicle Angular velocity reference
thetadr=(yddr.*xdr-xddr.*ydr)./v1r.^2; %Angular velocity velocity of the trajectory

%Steering angle reference
phir=atan(l*(yddr.*xdr-xddr.*ydr)./v1r.^3);

%Steering velocity reference
v2r=l*v1r.*((ydddr.*xdr-xdddr.*ydr).*v1r.^2-3*(yddr.*xdr-xddr.*ydr).*(xdr.*xddr+ydr.*yddr))./(v1r.^6+l^2.*(yddr.*xdr-xddr.*ydr).^2);



x0=-0.4; y0=-0.5; theta0=thetar(1); phi0=0;

q0=[x0;y0;theta0;phi0];
% q0c=[x0+(l/2)*cos(theta0);y0+(l/2)*sin(theta0);theta0];

%Linear Controller gain

K=5*eye(2);
Delta=0.35;
N=5;

qz=1;
qu=0.01;


%Filter parameters
alpha_kalman=0.9;
beta_kalman=2;
kappa_kalman=0;
% 

% Q_kalman=diag([0.01 0.01 0.1 0.1]);
% R_kalman=diag([2e-5 2e-5 0.01 0.01]);
% P0_kalman=diag([1e-3 1e-3 0.1 0.01]);

Q_kalman=diag([0.00001 0.00001 0.00001 0.00001]);
R_kalman=diag([0.01 0.01 0.01 0.01]);
P0_kalman=diag([1e-3 1e-3 1e-3 1e-3]);

% Q_kalman=diag([0.001 0.001 0.1 0.1]);
% R_kalman=diag([2e-5 2e-5 0.00001 0.00001]);
% P0_kalman=diag([1e-5 1e-5 1e-6 1e-6]);



%Constraints
v1max=2; %Driving velocity limit (m/s)
v2max=2; %Steering velocity limit (rad/s)
T=[-1 0; 0 -1; 1 0; 0 1];
g=[v1max;v2max;v1max;v2max];


%Worst case feasible input set
phi_worst=0.6;
r_u=sqrt((Delta^2*l^2*v2max^2)/(Delta^2-Delta^2*cos(phi_worst)^2+l^2));
Q_u=(1/r_u^2)*eye(2);
ell_u=ellipsoid(inv(Q_u));


N_sides=15;
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



return%remove return to plot trajectory
figure
plot(ell_u)
hold on
plot(P_worst)
%Plots



figure
plot(xr,yr);
grid
legend("2D-Trajectory")




figure
subplot(6,1,1)
plot(t,xr)
grid
legend("Reference Position along x-axis")


subplot(6,1,2)
plot(t,yr)
grid
legend("Reference Position along y-axis")


subplot(6,1,3)
plot(t,xdr)
grid
legend("Reference Velocity along x-axis")


subplot(6,1,4)
plot(t,ydr)
grid
legend("Reference Velocity along y-axis")



subplot(6,1,5)
plot(t,xddr)
grid
legend("Reference Acceleration along x-axis")



subplot(6,1,6)
plot(t,yddr)
grid
legend("Reference Acceleration along y-axis")



figure
subplot(4,1,1)
plot(t,thetar)
grid
legend("Reference Orientation")

subplot(4,1,2)
plot(t,phir)
grid
legend("Reference Steering Angle")

subplot(4,1,3)
plot(t,v1r)
grid
legend("Reference Longitudinal Velocity")


subplot(4,1,4)
plot(t,v2r)
grid
legend("Reference Steering Angular Velocity")



sol_info=Simulink.BusElement;
sol_info(1).Name="Solution";
sol_info(2).Name="FunctionVal";
sol_info(3).Name="ExitFlag";
sol_info(4).Name="Output";


