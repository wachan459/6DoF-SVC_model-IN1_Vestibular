% This file is part of 6DoF-SVC_IN model.
% Copyright (c) 2025 Takahiro Wada
% Released under the MIT License.
%
% 2025, June 27
% Test program for 6DoF-SVC-IN1 model (sixdofSVC_IN1_public.m).
% Sinusoidal vertical movement is used.
%
%[1] Inoue, S., Liu, H., Wada, T. Revisiting Motion Sickness Models Based on SVC Theory Considering Motion Perception. 
% SAE Technical Paper, 2023
%[2] T. Wada and J. Bos, Theoretical Considerations on Models of Vestibular Self-motion Perception 
%    as Inherent in Computational Frameworks of Motion Sickness, Biological Cybernetics, 2025


clc
clear

DEG2RAD=3.141592/180.0;
Dt=0.01;
Fs=floor(1/Dt);
FTIME=7200; %simulation time 

MABIKI = 10;
NTIME = 30*60*Fs;
fileID = fopen('MSIfile.dat','w');

%Parameters used in 
%[1] Inoue, S., Liu, H., Wada, T. Revisiting Motion Sickness Models Based on SVC Theory Considering Motion Perception. 
% SAE Technical Paper, 2023
%[2] T. Wada and J. Bos, Theoretical Considerations on Models of Vestibular Self-motion Perception 
%    as Inherent in Computational Frameworks of Motion Sickness, Biological Cybernetics, 2025

    sixdofSVCparam = setSVCIN1Param();


%omega_ts = timeseries(omega, t);
%a_ts = timeseries(a, t);

t = [0:Dt:FTIME]';       % Time data
n_time = length(t);

fmat=zeros(3,n_time);
omegamat=zeros(3,n_time);
MSIarray = zeros(n_time,1); %for File output
tarray = zeros(n_time,1);

% Longitudinal vibration
Amp_RMSG=0.234 %G
freq=0.2 %Hz

for i=1:1:n_time
    fmat(:,i) = [0.0, 0.0, 9.81+9.81*sqrt(2)*Amp_RMSG*sin(2*pi*freq*t(i))]; 
    omegamat(:,i) = [0,0,0];
end

uinput = [omegamat; fmat]';

fdummy=zeros(3,1);
fdummy(3)=9.81; %for simulation in vertical direction
fdummy = fdummy/norm(fdummy)*9.81;

%Preparation of initial value of state variable x
% set tentative initial value of state variable x
  xscc=[0;0;0]; %1-3
  dotx_scc = [0;0;0]; %4-6
  v_s=fdummy;   %7-9
  hv_s = fdummy;    %10-12
  x_hscc = [0;0;0]; %13-15
  x_vc = fdummy/sixdofSVCparam.Kvc; %16-18
   g = fdummy;  %19-21
  dotMsi = 0;   %22
  Msi = 0;  %23

x=[xscc;
 dotx_scc;
 v_s;
 hv_s;
 x_hscc;
 x_vc;
 g;
 dotMsi;
 Msi];

uinputprepare = [ones(NTIME,1)*0.0,ones(NTIME,1)*0.0,ones(NTIME,1)*0.0,ones(NTIME,1)*fdummy(1),ones(NTIME,1)*fdummy(2),ones(NTIME,1)*fdummy(3)];

xdumarray=zeros(23,n_time);
xdumarray(:,1) =x;
deltaarray=ones(n_time,1);

%Calculation loop to determine initial value of state value x
for count = 1:NTIME;

	nextx = sixdofSVC_IN1_public(x, uinputprepare(count,:), Dt, sixdofSVCparam);
    deltaarray(count)=norm(x-nextx);
    x = nextx;
    xdumarray(:,count)=nextx;

end

epsilon=10.0^(-6);
shushokugosa=deltaarray(count)
if(deltaarray(count)<=epsilon)
    disp('The convergence of the state variable initialization calculation was successful.');
else
    warning("The calculation for state variable initialization has not converged.");
end

x(19:21) = fdummy; %g(0) = fdummy
x(22)=0.0; %dotMSI(0) = 0
x(23)=0.0; %MSI(0) = 0

% At this point, the initial values of state variables x(0) to x(23) have been determined. 

% From here on, MSI calculations will be performed using the data.
xarray=zeros(23,n_time);
conflict=zeros(n_time,1);

for count = 1:n_time
nextx = sixdofSVC_IN1_public(x, uinput(count,:), Dt, sixdofSVCparam);
    x = nextx;
    xarray(:,count) = x;
    dumv_s = [x(7),x(8),x(9)];
    dumhv_s = [x(10), x(11), x(12)];
    conflict(count) = norm(dumv_s - dumhv_s);
%    MSIarray(count) = x(23);
    tarray(count) = count*Dt;


    if rem(count, MABIKI) == 0
        fprintf(fileID,'%f, %f \n', count*Dt, x(23));
    end

end

omega = uinput(:,1:3);
f = uinput(:,4:6);
x_scc = xarray(1:3,:)';
dotx_scc = xarray(4:6,:)';
v_s = xarray(7:9,:)';
hv_s = xarray(10:12,:)';
x_hscc = xarray(13:15,:)';
x_vc = xarray(16:18,:)';
g = xarray(19:21,:)';
dotMsi = xarray(22,:)';
Msi=xarray(23,:)';

prm=sixdofSVCparam;
a=f-g;
omega_s = (1/prm.taud + 1/prm.taua)*dotx_scc + 1/prm.taud/prm.taua*x_scc + omega;
a_s = f - v_s;
hg  = prm.Kvc*x_vc;
hf = (1/(1+prm.Kac)) * (prm.Ka*a + prm.Kvc*x_vc + prm.Kac*(a_s + hv_s));
ha = hf-hg; 
ha_s = hf-hv_s;
homega_s = (1/(1+prm.Kwc)) * (x_hscc + prm.Kw*omega + prm.Kwc*omega_s);
homega = prm.Kw*omega + prm.Kwc*(omega_s-homega_s);
dumm = (norm(v_s-hv_s)/prm.b)^2;
x_H = dumm/(1+dumm);

time=tarray;

Tbl = table(time, f, homega, ha, hf, hg, g,conflict);

sp= stackedplot(Tbl, 'XVariable', 'time');
sp.AxesProperties(1).YLimits = [-20 20]; 
sp.AxesProperties(2).YLimits = [-1 1]; 
sp.AxesProperties(3).YLimits = [-5 5]; 
sp.AxesProperties(4).YLimits = [-20 20]; 
sp.AxesProperties(5).YLimits = [-20 20];
sp.AxesProperties(6).YLimits = [0 10];
sp.AxesProperties(7).YLimits = [0 5]; 

MSIatFinaltime = Msi(end)




