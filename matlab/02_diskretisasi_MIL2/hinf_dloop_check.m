function T = hinf_dloop_check(K, Pz, w)
%HINF_DLOOP_CHECK  Margin & stabilitas loop diskret H-inf K(z) + plant Pz + Unit Delay.
% loop L = K(z) Pz(z) z^-1, closed loop dari interkoneksi state-space
[~,~,~,~,Ts] = ssdata(Pz);
z  = exp(1j*w(:)*Ts);
Lw = ss_fr(K, w).*ss_fr(Pz, w)./z;
T  = loop_margins(Lw, w);
[Ak,Bk,Ck,Dk] = ssdata(K);  [Ap,Bp,Cp,Dp] = ssdata(Pz);
nk = size(Ak,1);  np = size(Ap,1);
% state [xp; xk; ud], e = -(Cp xp + Dp ud), u = Ck xk + Dk e, ud+ = u
Acl = [Ap,            zeros(np,nk), Bp;
       -Bk*Cp,        Ak,           -Bk*Dp;
       -Dk*Cp,        Ck,           -Dk*Dp];
T.rho = max(abs(eig(Acl)));  T.stable = T.rho < 1;
end
