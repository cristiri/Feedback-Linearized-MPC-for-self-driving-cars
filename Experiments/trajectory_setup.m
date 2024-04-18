

load("trajectory_state.mat")
load("trajectory_inputs.mat")

Tf=(length(ur)-1)*Ts2;
t=0:Ts2:Tf;



Ts_new=0.008;
xr=qr(1,:);
yr=qr(2,:);
thetar=qr(3,:);
phir=qr(4,:);

ur=(Ts_data/Ts)*ur;

v1r=ur(1,:);
v2r=ur(2,:);

qr=timeseries(qr,t);
vr=timeseries(ur,t);

if plot_data
    f=figure;
    plot(xr,yr)
    grid
    hold on
    legend("Trajectory in the x-y plane")


    figure
    plot(t,xr)
    grid
    hold on
    plot(t,yr)
    legend("Trajectory along x","Trajectory along y")


    figure
    plot(t,thetar)
    grid
    hold on
    legend("Orientation \theta")


    figure
    plot(t,phir)
    grid
    hold on
    legend("Steering Angle  \phi")



    figure
    plot(t,v1r)
    grid
    hold on
    legend("Longitudinal Velocity")



    figure
    plot(t,v2r)
    grid
    hold on
    legend("Angular Steering Velocity")
end

i_f=length(xr);


xr=[xr xr(end)*ones(1,N)];
yr=[yr yr(end)*ones(1,N)];
thetar=[thetar thetar(end)*ones(1,N)];
v1r=[v1r v1r(end)*ones(1,N)];
v2r=[v2r v2r(end)*ones(1,N)];
phir=[phir phir(end)*ones(1,N)];



xr_pred=[];
yr_pred=[];
thetar_pred=[];
phir_pred=[];
v1r_pred=[];
v2r_pred=[];

for i=1:i_f
    xr_pred=[xr_pred; xr(i:i+N-1)];
    yr_pred=[yr_pred; yr(i:i+N-1)];
    thetar_pred=[thetar_pred; thetar(i:i+N-1)];
    phir_pred=[phir_pred; phir(i:i+N-1)];
    v1r_pred=[v1r_pred; v1r(i:i+N-1)];
    v2r_pred=[v2r_pred; v2r(i:i+N-1)];

end

xr_pred=xr_pred';
yr_pred=yr_pred';
thetar_pred=thetar_pred';
phir_pred=phir_pred';
v1r_pred=v1r_pred';
v2r_pred=v2r_pred';


xr_pred_series=timeseries(xr_pred,t);
yr_pred_series=timeseries(yr_pred,t);
thetar_pred_series=timeseries(thetar_pred,t);
phir_pred_series=timeseries(phir_pred,t);
v1r_pred_series=timeseries(v1r_pred,t);
v2r_pred_series=timeseries(v2r_pred,t);


save('trajectory','xr_pred_series','yr_pred_series','thetar_pred_series','v1r_pred_series','v2r_pred_series','phir_pred_series','qr','ur','Tf');





