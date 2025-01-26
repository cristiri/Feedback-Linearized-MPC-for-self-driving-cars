clear; close all;
Ts=0.01;

%Defining Eight-shaped Trajectory


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


%Initial conditions 
x0=-0.1; y0=-0.05; theta0=thetar(1); phi0=0;

q0=[x0;y0;theta0;phi0];
% q0c=[x0+(l/2)*cos(theta0);y0+(l/2)*sin(theta0);theta0];

%Linear Controller gain

K=5*eye(2);
Delta=0.35;
N=10;
N_sides=10;

qz=10;
qu=0.00001;


%Active Set solver Parameters
Phi=repmat(eye(2),N,1);
Theta=Ts*repmat(eye(2),N);
Theta=tril(Theta);
Qhat=qz*eye(2*N);
Rhat=qu*eye(2*N);
H_obj=Theta'*Qhat*Theta+Rhat;

[L,p] = chol(H_obj,'lower');
Linv = linsolve(L,eye(size(L)),struct('LT',true));


%Constraints
v1max=2; %Driving velocity limit (m/s)
v2max=2; %Steering velocity limit (rad/s)
T=[-1 0; 0 -1; 1 0; 0 1];
g=[v1max;v2max;v1max;v2max];


%Worst case feasible input set
phi_worst=0.6;
% r_u=sqrt((Delta^2*l^2*v2max^2)/(Delta^2-Delta^2*cos(phi_worst)^2+l^2));
r_u=min(v1max,sqrt((Delta^2*l^2*v2max^2)/(Delta^2-Delta^2*cos(phi_worst)^2+l^2)));


%Terminal set
S=1/r_u^2*(K'*K);
r_N=sqrt(1/S(1,1));
inv_ell=ellipsoid(inv(S));
figure
plot(inv_ell)
hold on
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
plot(P_N,'Color','green')



figure
%Input constraint set 
Q_u=(1/r_u^2)*eye(2);
ell_u=ellipsoid(inv(Q_u));
plot(ell_u)
hold on


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
plot(P_worst,'Color','blue')
%Plots



z_tilde=[0.1;0.1];
Theta=Ts*repmat(eye(2),N);
Theta=tril(Theta);
Theta_N=Theta(2*N-1:2*N,:);
Psi_N=eye(2);


return %remove return to plot trajectory 

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


