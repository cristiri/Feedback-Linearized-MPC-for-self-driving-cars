clear; close all 
load("trajectory_data_q2.mat")
load("trajectory_data_u2.mat")

load("control_inputs_FL_MPC2.mat")
load("car_speed_FL_MPC2.mat");
load("q_car_FL_MPC2.mat")


load("control_inputs_FL_MPC_T2.mat")
load("car_speed_FL_MPC_T2.mat");
load("q_car_FL_MPC_T2.mat")

load("control_inputs_Zhang2.mat")
load("car_speed_Zhang2.mat");
load("q_car_Zhang2.mat")


load("control_inputs_NL_MPC2.mat")
load("car_speed_NL_MPC2.mat");
load("q_car_NL_MPC2.mat")





dg=[0 128/255 0];
or=[1 127/255 80/255];



%FL-MPC no terminal controller data
t=q_car1(1,:);

x_car1=q_car1(2,:);
y_car1=q_car1(3,:);
theta_car1=q_car1(4,:);
phi_car1=q_car1(5,:);


x_traj=q_traj(2,:);
y_traj=q_traj(3,:);
theta_traj=q_traj(4,:);
phi_traj=q_traj(5,:);


e_x1=x_car1-x_traj;
e_y1=y_car1-y_traj;
e_theta1=theta_car1-theta_traj;
e_phi1=phi_car1-x_traj;

e_x_avg1=mean(e_x1);
e_y_avg1=mean(e_y1);
e_theta_avg1=mean(e_theta1);
e_phi_avg1=mean(e_phi1);

e_avg1=[e_x_avg1;e_y_avg1;e_theta_avg1];

e_norm1=norm(e_avg1,2)




%FL-MPC with terminal controller data
t=q_car2(1,:);

x_car2=q_car2(2,:);
y_car2=q_car2(3,:);
theta_car2=q_car2(4,:);
phi_car2=q_car2(5,:);


x_traj=q_traj(2,:);
y_traj=q_traj(3,:);
theta_traj=q_traj(4,:);
phi_traj=q_traj(5,:);


e_x2=x_car2-x_traj;
e_y2=y_car2-y_traj;
e_theta2=theta_car2-theta_traj;
e_phi2=phi_car2-x_traj;

e_x_avg2=mean(e_x2);
e_y_avg2=mean(e_y2);
e_theta_avg2=mean(e_theta2);
e_phi_avg2=mean(e_phi2);

e_avg2=[e_x_avg2;e_y_avg2;e_theta_avg2];

e_norm2=norm(e_avg2,2)



%NonLinear MPC

x_car3=q_car3(2,:);
y_car3=q_car3(3,:);
theta_car3=q_car3(4,:);
phi_car3=q_car3(5,:);



e_x3=x_car3-x_traj;
e_y3=y_car3-y_traj;
e_theta3=theta_car3-theta_traj;
e_phi3=phi_car3-x_traj;

e_x_avg3=mean(e_x3);
e_y_avg3=mean(e_y3);
e_theta_avg3=mean(e_theta3);
e_phi_avg3=mean(e_phi3);

e_avg3=[e_x_avg3;e_y_avg3;e_theta_avg3];

e_norm3=norm(e_avg3,2)



%Zhang controller


x_car4=q_car4(2,:);
y_car4=q_car4(3,:);
theta_car4=q_car4(4,:);
phi_car4=q_car4(5,:);



e_x4=x_car4-x_traj;
e_y4=y_car4-y_traj;
e_theta4=theta_car4-theta_traj;
e_phi4=phi_car4-x_traj;

e_x_avg4=mean(e_x4);
e_y_avg4=mean(e_y4);
e_theta_avg4=mean(e_theta4);
e_phi_avg4=mean(e_phi4);

e_avg4=[e_x_avg4;e_y_avg4;e_theta_avg4];

e_norm4=norm(e_avg4,2)


%Plots


figure
grid
hold on
p1=plot(x_car1,y_car1,'b');
p2=plot(x_car2,y_car2,'r');
p3=plot(x_car3,y_car3,'Color',dg);
p4=plot(x_car4,y_car4,'Color',or);

p5=plot(x_traj,y_traj,'k--');
p6=plot(x_car1,y_car1,'b-p','MarkerIndices',[1 1],'MarkerFaceColor','red','MarkerSize',25);




p1.LineWidth=1.5;
p2.LineWidth=1.5;
p3.LineWidth=1.5;
p4.LineWidth=1.5;
p5.LineWidth=2;
p6.LineWidth=1.5;

% 
% axis([-0.8 1 -0.8 0.8])
l=legend([p1,p2,p3,p4,p5,p6],'FL-MPC (No Terminal)','FL-MPC (Terminal)','NL-MPC','[Zhang]','Reference Trajectory','Initial point');
l.FontSize=27;
xlabel('x[m]')
ylabel('y[m]')


figure
subplot(2,1,1)
grid
hold on
p1=plot(t,q_car1(4,:),'b');
p2=plot(t,q_car2(4,:),'r');
p3=plot(t,q_car3(4,:),'Color',dg);
p4=plot(t,q_car4(4,:),'Color',or);

p5=plot(t,q_traj(4,:),'k--');
p1.LineWidth=1.5;
p2.LineWidth=1.5;
p3.LineWidth=1.5;
p4.LineWidth=1.5;
p5.LineWidth=1.5;
axis([0 t(end) -4.5 2])

xlabel('t[sec]')
ylabel('\theta[RAD]')


% l=legend([p1,p2,p3,p4,p5],'FL-MPC (No Terminal)','FL-MPC (Terminal)','NL-MPC','[Zhang]','Reference Trajectory');
% l.FontSize=27;




subplot(2,1,2)
grid
hold on
p4=plot(t(1:3:end),q_car4(5,1:3:end),'Color',or);
p1=plot(t,q_car1(5,:),'b');
p2=plot(t,q_car2(5,:),'r');
p3=plot(t,q_car3(5,:),'Color',dg);


p5=plot(t,q_traj(5,:),'k--');
p1.LineWidth=1.5;
p2.LineWidth=1.5;
p3.LineWidth=1.5;
p4.LineWidth=0.2;
p5.LineWidth=1.5;
axis([0 t(end) -0.65 0.65])
xlabel('t[sec]')
ylabel('\phi[RAD]')


% l=legend([p1,p2,p3,p4,p5],'FL-MPC (No Terminal)','FL-MPC (Terminal)','NL-MPC','[Zhang]','Reference Trajectory');
% l.FontSize=27;

figure
subplot(2,1,1)
grid
hold on
p1=plot(t,u_fb1(2,:),'b');
p2=plot(t,u_fb2(2,:),'r');
p3=plot(t,u_fb3(2,:),'Color',dg);
p4=plot(t,u_fb4(2,:),'Color',or);
p5=plot(t,ones(1,length(t)),'k--');
plot(t,-ones(1,length(t)),'k--')
xlabel('t[sec]')
ylabel('v(t)[m/s]')
axis([0 t(end) -0.4 1.1])

l=legend([p1,p2,p3,p4,p5],'FL-MPC (No Terminal)','FL-MPC (Terminal)','NL-MPC','[Zhang]','v_{max}');
l.FontSize=20;

subplot(2,1,2)
grid
hold on
p1=plot(t,u_fb1(3,:),'b');
p2=plot(t,u_fb2(3,:),'r');
p3=plot(t,u_fb3(3,:),'Color',dg);
p4=plot(t,u_fb4(3,:),'Color',or);
p5=plot(t,10*ones(1,length(t)),'k--');
plot(t,-10*ones(1,length(t)),'k--')
xlabel('t[sec]')
ylabel('\omega(t)[RAD/s]')
l=legend([p1,p2,p3,p4,p5],'FL-MPC (No Terminal)','FL-MPC (Terminal)','NL-MPC','[Zhang]','\omega_{max}');
l.FontSize=20;
axis([0 t(end) -11 11])




