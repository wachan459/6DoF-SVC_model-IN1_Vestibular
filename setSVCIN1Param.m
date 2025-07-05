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


function sixdofSVCparam = setSVCIN1Param()

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


