%% ===================================================================
%  hinf_synthesis_MIL1.m
%  Sintesis H-Infinity mixed-sensitivity (S/KS/T) untuk Quadratic Boost
%  Converter (QBC) + persiapan kontroler untuk MIL-1 (kuasi-kontinu).
%
%  Titik kerja final (sesuai Khinf_QBC_MIL1.mat, gamma = 0,609, orde 6):
%    Vin = 18 V, Vo = 150 V, R = 135 ohm (Po = 166,7 W)
%    L1 = 1 mH, L2 = 3 mH, C1 = C2 = 100 uF, fs = 10 kHz
%
%  Output: Khinf_QBC_MIL1.mat  (K, Kd, Ts_ctrl, G, W1-W3, parameter)
%          Hinf_S.png, Hinf_T.png, Hinf_KS.png, Hinf_step.png
%
%  Di Simulink (blok Discrete State-Space kontroler):
%    A = Kd.A, B = Kd.B, C = Kd.C, D = Kd.D, Sample time = Ts_ctrl
%    Model Properties > Callbacks > InitFcn:  load('Khinf_QBC_MIL1.mat');
%  ===================================================================
clear; clc; close all;

%% 0) Parameter rangkaian
Vin = 18;          % tegangan input (V)
Vo  = 150;         % tegangan output target (V)
R   = 135;         % beban nominal (ohm)
L1  = 1e-3;        % (H)
L2  = 3e-3;        % (H)
C1  = 100e-6;      % Cdc (F)
C2  = 100e-6;      % Co (F)
fs  = 10e3;        % frekuensi switching (Hz)
Ts  = 1e-7;        % periode

%% 1) Titik kerja (steady-state, lossless) + cek CCM
D_op = 1 - sqrt(Vin/Vo);          % Vo = Vin/(1-D)^2
Dp   = 1 - D_op;
VC1  = Vin/Dp;
Io   = Vo/R;
IL2  = Io/Dp;
IL1  = IL2/Dp;
Po   = Vo^2/R;

dIL1 = Vin*D_op/(L1*fs);          % ripple p-p
dIL2 = VC1*D_op/(L2*fs);
dVC1 = IL2*D_op/(C1*fs);
dVo  = Io*D_op/(C2*fs);

fprintf('=== 1) Titik kerja ===\n');
fprintf('D = %.4f | VC1 = %.2f V | IL1 = %.3f A | IL2 = %.3f A | Io = %.3f A | Po = %.1f W\n', ...
        D_op, VC1, IL1, IL2, Io, Po);
fprintf('Ripple p-p: dIL1 = %.3f A (%.1f%%) | dIL2 = %.3f A (%.1f%%) | dVC1 = %.3f V (%.1f%%) | dVo = %.3f V (%.2f%%)\n', ...
        dIL1, 100*dIL1/IL1, dIL2, 100*dIL2/IL2, dVC1, 100*dVC1/VC1, dVo, 100*dVo/Vo);

% CCM: arus minimum induktor (I - dI/2) harus > 0
ccm1 = IL1 - dIL1/2 > 0;
ccm2 = IL2 - dIL2/2 > 0;
% Batas beban CCM (D tetap): IL1 = Vo/(R*Dp^2), IL2 = Vo/(R*Dp)
R_crit = min(Vo/(Dp^2*dIL1/2), Vo/(Dp*dIL2/2));
fprintf('CCM L1: %d | CCM L2: %d | iL1,min = %.2f A | iL2,min = %.2f A\n', ...
        ccm1, ccm2, IL1-dIL1/2, IL2-dIL2/2);
fprintf('Batas CCM: R <= %.1f ohm (Po >= %.0f W). Uji variasi beban harus di bawah batas ini.\n\n', ...
        R_crit, Vo^2/R_crit);
if ~(ccm1 && ccm2)
    warning('Titik kerja nominal TIDAK CCM -> model Gvd(s) di bawah tidak berlaku.');
end

%% 2) Plant Gvd(s) — small-signal state-space, x = [iL1; iL2; vC1; vCo]
Ag = [0       0       -Dp/L1   0;
      0       0        1/L2   -Dp/L2;
      Dp/C1  -1/C1     0       0;
      0       Dp/C2    0      -1/(R*C2)];
Bg = [VC1/L1; Vo/L2; -IL1/C1; -IL2/C2];
Cg = [0 0 0 1];
Dg = 0;
G  = ss(Ag,Bg,Cg,Dg);

fprintf('=== 2) Plant Gvd(s) ===\n');
damp(G)
zG = zero(G);
zR = zG(real(zG) > 0);                         % semua zero RHP (real & kompleks)
fprintf('DC gain Gvd(0) = %.2f\n', dcgain(G));
fprintf('Zero RHP:\n'); disp(zR);
wz = min(abs(zR));
fprintf('Zero RHP terdekat |z| = %.1f rad/s -> crossover sebaiknya < ~%.0f rad/s\n\n', wz, wz/3);

%% 3) Spesifikasi performa -> bobot W1, W2, W3
wb    = 100;     % bandwidth target (rad/s)
M1    = 2;       % batas puncak |S|
A1    = 1e-3;    % batas |S| frekuensi rendah (~ error steady-state)
u_max = 0.6*min(D_op, 1-D_op);   % margin duty ke saturasi (60%)
W2c   = 1/u_max;
M3    = 5;       % batas |T| frekuensi rendah
eps3  = 0.02;    % batas |T| frekuensi tinggi
wbt   = 1500;    % corner W3 (rad/s)

W1 = tf([1/M1 wb],[1 wb*A1]);
W2 = tf(W2c);
W3 = tf([1 wbt/M3],[eps3 wbt]);
fprintf('=== 3) Bobot ===\n');
fprintf('wb = %g | M1 = %g | A1 = %g | u_max = %.4f | W2 = %.4f | M3 = %g | eps3 = %g | wbt = %g\n\n', ...
        wb, M1, A1, u_max, W2c, M3, eps3, wbt);

%% 4) Generalized plant + sintesis H-infinity
P = augw(G, W1, W2, W3);
[K, CL, gamma] = hinfsyn(P, 1, 1);
K = minreal(ss(K));             % kontroler yang dianalisis = yang diimplementasikan

fprintf('=== 4) Hasil sintesis ===\n');
fprintf('gamma = %.4f -> %s\n', gamma, ...
        ternary(gamma < 1, 'spesifikasi S/KS/T terpenuhi', 'BELUM terpenuhi, turunkan wb'));
fprintf('Orde K(s) = %d\n', order(K));
pK = pole(K);
disp(pK);
[~, is] = min(abs(pK));
p_slow = pK(is);
if abs(p_slow) < 1
    fprintf(['Pole paling lambat K = %.3g rad/s (konstanta waktu %.1f s): kuasi-integrator.\n' ...
             '-> Feedforward D0 TIDAK diperlukan. Sisa error steady-state kecil (orde A1).\n\n'], ...
             real(p_slow), 1/abs(p_slow));
else
    fprintf('K tanpa (kuasi-)integrator -> tambahkan feedforward D0 = %.4f sebelum saturasi.\n\n', D_op);
end

%% 5) Kontroler untuk MIL-1 (kuasi-kontinu)
%  MIL-1 memakai sample time sangat kecil (= step powergui) sehingga perilaku
%  K(z) praktis identik dengan K(s). MIL-2 nanti: Ts_ctrl = 1/fs + unit delay.
Ts_ctrl = 1e-7;
Kd = c2d(K, Ts_ctrl, 'tustin');
fprintf('=== 5) Kontroler MIL-1 ===\n');
fprintf('Ts_ctrl = %g s | max|pole Kd| = %.8f (<= 1 -> stabil/marginal integrator)\n\n', ...
        Ts_ctrl, max(abs(pole(Kd))));

%% 6) Verifikasi closed-loop
L  = G*K;
S  = feedback(1, L);
Tc = feedback(L, 1);
KS = K*S;
w  = logspace(0, 8, 2000);

fig1 = mkfig;
bodemag(S,'b', 1/W1,'r--', w); grid on;
title('Sensitivity S(s) vs 1/W_1(s)'); legend('S(s)','1/W_1(s)','Location','southeast');
[mS,~,wS] = bode(S,w); [Spk,iS] = max(20*log10(squeeze(mS)));

fig2 = mkfig;
bodemag(Tc,'b', 1/W3,'r--', w); grid on;
title('Complementary Sensitivity T(s) vs 1/W_3(s)'); legend('T(s)','1/W_3(s)','Location','southwest');
[mT,~,wT] = bode(Tc,w); [Tpk,iT] = max(20*log10(squeeze(mT)));

fig3 = mkfig;
bodemag(KS,'b', 1/W2,'r--', w); grid on;
title('KS(s) vs 1/W_2(s)'); legend('KS(s)','1/W_2(s)','Location','southeast');
[mKS,~,~] = bode(KS,w); KSpk = max(20*log10(squeeze(mKS)));

fig4 = mkfig;
[y,t] = step(Tc);
plot(t, y, 'b', 'LineWidth', 1.2); grid on; hold on;
yline(1,'k--'); yline(1.02,'g:'); yline(0.98,'g:');
title(sprintf('Step response closed-loop linear (V_o = %g V, R = %g \\Omega)', Vo, R));
xlabel('Waktu (s)'); ylabel('Amplitudo (ternormalisasi)');
legend('Respons','Setpoint','Pita \pm2%','Location','southeast');
info = stepinfo(y, t, 'SettlingTimeThreshold', 0.02);

mg = allmargin(L);
dm = diskmargin(L);

fprintf('=== 6) Verifikasi ===\n');
fprintf('Puncak |S|  = %.2f dB pada %.1f rad/s\n', Spk, wS(iS));
fprintf('Puncak |T|  = %.2f dB pada %.1f rad/s\n', Tpk, wT(iT));
fprintf('Puncak |KS| = %.2f dB\n', KSpk);
fprintf('Rise time = %.2f ms | Overshoot = %.2f %% | Undershoot = %.2f %% | Settling = %.2f ms\n', ...
        info.RiseTime*1e3, info.Overshoot, info.Undershoot, info.SettlingTime*1e3);
fprintf('Gain margin  = %s dB pada %s rad/s\n', mat2str(20*log10(mg.GainMargin),3), mat2str(mg.GMFrequency,4));
fprintf('Phase margin = %s deg pada %s rad/s\n', mat2str(mg.PhaseMargin,3), mat2str(mg.PMFrequency,4));
fprintf('Disk margin  : GM = [%.2f %.2f] dB, PM = %.1f deg\n', ...
        20*log10(dm.GainMargin(1)), 20*log10(dm.GainMargin(2)), dm.PhaseMargin(2));
fprintf('Delay margin = %.3e s\n', min(mg.DelayMargin));

delay_impl = 0.5/fs + 1e-6;     % ~ setengah periode PWM + konversi ADC
fprintf('Estimasi delay implementasi = %.2e s -> rasio delay margin = %.0fx\n', ...
        delay_impl, min(mg.DelayMargin)/delay_impl);
fprintf('Stabil closed-loop: %d\n\n', isstable(Tc));

%% 7) Simpan
save('Khinf_QBC_MIL1.mat', 'K','Kd','Ts_ctrl','G','W1','W2','W3','gamma', ...
     'D_op','Vin','Vo','R','L1','L2','C1','C2','fs');
exportgraphics(fig1,'Hinf_S.png','Resolution',300);
exportgraphics(fig2,'Hinf_T.png','Resolution',300);
exportgraphics(fig3,'Hinf_KS.png','Resolution',300);
exportgraphics(fig4,'Hinf_step.png','Resolution',300);
fprintf('Tersimpan: Khinf_QBC_MIL1.mat + Hinf_S/T/KS/step.png\n');

%% ===================== Fungsi lokal =====================
function f = mkfig
    f = figure('Color','w');
    try, theme(f,'light'); catch, end    % figur putih walau MATLAB dark mode
end

function s = ternary(c, a, b)
    if c, s = a; else, s = b; end
end
