function sixdofSVCparam = setSVCIN1Param()

%Setting model parameters by structures
%the followings are parameters employed in 
% Wada, T., Fujisawa, S., Doi, S. 
% Analysis of driver’s head tilt using a mathematical model of motion sickness. 
% International Journal of Industrial Ergonomics, 89-97, 2018.
%sixdofSVCparam.Ka=0.1;
%sixdofSVCparam.Kw=0.8;
%sixdofSVCparam.Kwc=5; %10
%sixdofSVCparam.Kvc=5;
%sixdofSVCparam.Kac=1;
%sixdofSVCparam.taud=7;
%sixdofSVCparam.taua=190;
%sixdofSVCparam.tau=5; %5s
%sixdofSVCparam.P=85;
%sixdofSVCparam.tauI = 12*60; %[sec]
%sixdofSVCparam.b=0.5;

%Parameters used in 
% Inoue, S., Liu, H., Wada, T. Revisiting Motion Sickness Models Based on SVC Theory Considering Motion Perception. 

% SAE Technical Paper, IN1, SAE2023
sixdofSVCparam.Ka=0.1;  % 0.1を暫定的に0にした．（2024年1月）
sixdofSVCparam.Kw=0.1; 
sixdofSVCparam.Kwc=10;
sixdofSVCparam.Kvc=5; 
sixdofSVCparam.Kac=0.5; %changed
sixdofSVCparam.taud=7;  
sixdofSVCparam.taua=190; 
sixdofSVCparam.tau=2;    %changed
sixdofSVCparam.P=85; 
sixdofSVCparam.tauI = 12*60; %[sec] 
sixdofSVCparam.b=0.5;


