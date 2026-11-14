%% ===================================================================
%  hinf_discretize_MIL2.m
%  Menyiapkan kontroler H-infinity untuk MIL-2 (dan SIL/PIL):
%    K(s) hasil MIL-1  ->  (opsional) reduksi orde  ->  Kd2 = c2d(., Ts2, 'tustin')
%
%  Syarat : Khinf_QBC_MIL1.mat (hasil hinf_synthesis_MIL1.m) ada di folder ini.
%  Output : Khinf_QBC_MIL2.mat  berisi Kr, Kd2, Ak2,Bk2,Ck2,Dk2, Ts2, dll.
%
%  Di Simulink (blok Discrete State-Space kontroler MIL-2):
%    A = Ak2, B = Bk2, C = Ck2, D = Dk2, Initial states = 0, Sample time = Ts2
%    Model Properties > Callbacks > InitFcn:  load('Khinf_QBC_MIL2.mat');
%
%  PENTING soal nama variabel: skrip parameter rangkaian (hinf_synthesis_MIL1.m)
%  memakai C1, C2 untuk KAPASITOR (skalar, Farad). Supaya matriks keluaran
%  kontroler tidak pernah tertimpa oleh variabel rangkaian itu (atau sebaliknya),
%  matriks kontroler diberi nama Ak2/Bk2/Ck2/Dk2 -- bukan A2/B2/C2/D2.
%  ===================================================================
clear; clc; close all;

%% 0) Pengaturan
%  CATATAN (11 Okt 2026): akhirnya dipakai K PENUH (orde 6), bukan Kr
%  hasil reduksi -- SIL yang sudah divalidasi & cocok dgn MIL-2 memakai
%  controller_coeffs.h orde 6. use_reduced diset false di sini supaya
%  skrip ini konsisten/reproducible dgn Khinf_QBC_MIL2.mat yang final.
use_reduced = false;     % true: buang pole K yang jauh di atas Nyquist (balred)
n_red       = 5;         % (dipakai hanya jika use_reduced = true)
S = load('Khinf_QBC_MIL1.mat');   % K, G, W1-W3, Vo, fs, ...
K  = S.K;   G = S.G;   fs = S.fs;
Ts2 = 1/fs;                        % periode kontroler = 100 us
wN  = pi/Ts2;                      % frekuensi Nyquist (rad/s)
fprintf('Ts2 = %.3g s | Nyquist = %.0f rad/s\n\n', Ts2, wN);

%% 1) Pole K dan pemetaan tustin (cari pole yang bermasalah)
pK = pole(K);
fprintf('Pole K (rad/s) dan |p|/Nyquist:\n');
for i = 1:numel(pK)
    fprintf('  %12.2f %+12.2fi   rasio = %.3f\n', real(pK(i)), imag(pK(i)), abs(pK(i))/wN);
end
Kd0 = c2d(K, Ts2, 'tustin');
fprintf('Pole z (K tanpa reduksi), diurut |z|:\n'); disp(sort(pole(Kd0)));
fprintf('  -> pole z negatif/mendekati -1 menandakan pole K terlalu cepat.\n\n');

%% 2) Reduksi orde (MatchDC mempertahankan gain DC)
if use_reduced
    opt = balredOptions('StateElimMethod','MatchDC');
    Kr  = balred(K, n_red, opt);
else
    Kr = K;
end
fprintf('Pole Kr (rad/s):\n'); disp(pole(Kr));

figure('Color','w'); bode(K, Kr, {1, 1e6}); grid on; legend('K','Kr');
title('K vs Kr (setelah reduksi)');

Sk  = feedback(1, G*K);   Sr = feedback(1, G*Kr);
fprintf('||S||inf : K = %.3f dB | Kr = %.3f dB\n', ...
    20*log10(norm(Sk,inf)), 20*log10(norm(Sr,inf)));
fprintf('Stabil closed-loop kontinu (Kr): %d\n', isstable(feedback(G*Kr,1)));
figure('Color','w'); step(feedback(G*K,1), feedback(G*Kr,1), 0.1); grid on;
legend('K','Kr'); title('Step linear: K vs Kr');

%% 3) Diskretisasi + cek loop diskret (delay 1 sampel + ZOH pada plant)
Kd2 = c2d(Kr, Ts2, 'tustin');
Gd  = c2d(G,  Ts2, 'zoh');
Ld  = Kd2 * Gd * tf(1, [1 0], Ts2);       % z^-1 = Unit Delay 1 sampel
Td  = feedback(Ld, 1);
fprintf('\nmax|pole Kd2| = %.8f\n', max(abs(pole(Kd2))));
fprintf('Stabil closed-loop diskret (delay 1 sampel): %d\n', isstable(Td));
mg = allmargin(Ld);
fprintf('GM = %s dB | PM = %s deg\n', ...
    mat2str(20*log10(mg.GainMargin),3), mat2str(mg.PhaseMargin,3));
figure('Color','w'); margin(Ld); grid on; title('Loop diskret (Kd2, Gd, z^{-1})');
figure('Color','w'); step(feedback(G*Kr,1), 0.1); hold on; step(Td, 0.1); grid on;
legend('kontinu (Kr)','diskret + delay'); title('Step: kontinu vs diskret');

%% 4) Matriks untuk blok Simulink dan simpan
%  Nama Ak2/Bk2/Ck2/Dk2 sengaja dipakai (bukan A2/B2/C2/D2) supaya tidak
%  bentrok dengan C1/C2 = kapasitor rangkaian di Khinf_QBC_MIL1.mat.
Ak2 = Kd2.A;  Bk2 = Kd2.B;  Ck2 = Kd2.C;  Dk2 = Kd2.D;
Vo = S.Vo;  D_op = S.D_op;   % dibawa untuk keperluan skrip lain
save('Khinf_QBC_MIL2.mat', 'Kr','Kd2','Ak2','Bk2','Ck2','Dk2','Ts2','fs','Vo','D_op');
fprintf('\nTersimpan: Khinf_QBC_MIL2.mat (Ak2,Bk2,Ck2,Dk2,Ts2). Orde Kd2 = %d\n', order(Kd2));
