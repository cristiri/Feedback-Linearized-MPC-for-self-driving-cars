clear; close all;
Ts=0.01;


%Defining eight-shaped trajectory
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

% add_2pi=false;
% for i=2:length(thetar)
%     if thetar(i)+thetar(i-1)<1e-2
%         add_2pi=true;
%     end
%     if add_2pi
%         thetar(i)=thetar(i)+2*pi;
%     end
% end



%Vehicle Angular velocity reference
thetadr=(yddr.*xdr-xddr.*ydr)./v1r.^2; %Angular velocity velocity of the trajectory

%Steering angle reference
phir=atan(l*(yddr.*xdr-xddr.*ydr)./v1r.^3);

%Steering velocity reference
v2r=l*v1r.*((ydddr.*xdr-xdddr.*ydr).*v1r.^2-3*(yddr.*xdr-xddr.*ydr).*(xdr.*xddr+ydr.*yddr))./(v1r.^6+l^2.*(yddr.*xdr-xddr.*ydr).^2);



x0=-0.4; y0=-0.5; theta0=thetar(1); phi0=0;

q0=[x0;y0;theta0;phi0];
% q0c=[x0+(l/2)*cos(theta0);y0+(l/2)*sin(theta0);theta0];



% d_x=1;
% sigma=0.3;
% alpha_c=0.75;
% k3=1.65;


% % %Control parameters
d_x=0.2;
sigma=0.3;
alpha_c=0.4;
k3=2;


%Constraints
v1max=2; %Driving velocity limit (m/s)
v2max=10; %Steering velocity limit (rad/s)
phi_max=0.6;
N=10;



return  %remove return to plot trajectory
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


