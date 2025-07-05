%2023.June, 4 
% 6DoFSVC-IN1 model with only Vestibular input
%[1] S. Inoue, H. Liu, and T. Wada, ÅgRevisiting Motion Sickness Models Based on SVC Theory Considering Motion Perception,Åh 
% SAE-paper, Apr. 2023. doi: 10.4271/2023-01-0176.
%[2] T. Wada and J. Bos, Theoretical Considerations on Models of Vestibular Self-motion Perception 
%    as Inherent in Computational Frameworks of Motion Sickness, Biological Cybernetics, 2025

% xnext = sixdofSVCforEXP(xstate, uinput, Dt, prm)

%[OUTPUT]
% xnext = state at next time step (x_i+1) in 23 dimension.
%
%[INPUT]
% xstate = current state (x_i)
%{
	x_scc = xstate(1:3);
	dotx_scc = xstate(4:6);
	v_s = xstate(7:9);
	hv_s = xstate(10:12);
	x_hscc = xstate(13:15);
	x_vc = xstate(16:18);
	g = xstate(19:21);
	dotMsi = xstate(22);
	Msi=xstate(23); 
%}
% uinput = current input (u_i) 6 dim.
% uinput=[omega, f]
% where omega= angular velocity in R^3.
% GIF vector f:= g + a
% where g & a are gravitational & intertial accelerations in R^3, respectively. 
% Dt = sampling time [s]
% prm: Struct for model parameters


function	xnext = sixdofSVC_In1_EXP(xstate, uinput, Dt, prm)

% uinput=[omega, f];

x_scc = xstate(1:3);
dotx_scc = xstate(4:6);
v_s = xstate(7:9);
hv_s = xstate(10:12);
x_hscc = xstate(13:15);
x_vc = xstate(16:18);
g = xstate(19:21);
dotMsi = xstate(22);
Msi=xstate(23);

omega = uinput(1:3)';

f = uinput(4:6)';
a = f - g;

% a = uinput(4:6)';  % for simulation
% f = a + g;  % for simulation

omega_s = (1/prm.taud + 1/prm.taua)*dotx_scc + 1/prm.taud/prm.taua*x_scc + omega;
a_s = f - v_s;
hf = (1/(1+prm.Kac)) * (prm.Ka*a + prm.Kvc*x_vc + prm.Kac*(a_s + hv_s));
homega_s = (1/(1+prm.Kwc)) * (x_hscc + prm.Kw*omega + prm.Kwc*omega_s);
dumm = (norm(v_s-hv_s)/prm.b)^2;
x_H = dumm/(1+dumm);

fnc = [
dotx_scc;
-(1/prm.taud+1/prm.taua)*dotx_scc - 1/prm.taud/prm.taua*x_scc - omega;
(f-v_s)/prm.tau - cross(omega_s, v_s);
(hf-hv_s)/prm.tau - cross(homega_s, hv_s);
-x_hscc/prm.taud - (prm.Kw*omega + prm.Kwc*(omega_s - homega_s))/prm.taud;
v_s - hv_s;
- cross(omega, g);
(-2*prm.tauI*dotMsi - Msi + prm.P*x_H)/prm.tauI/prm.tauI;
dotMsi];

dumxnext = xstate + fnc*Dt;
dumxnext(19:21) = dumxnext(19:21)/norm(dumxnext(19:21))*9.81;
xnext = dumxnext;

end

