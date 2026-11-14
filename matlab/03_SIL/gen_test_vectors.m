%% ===================================================================
%  gen_test_vectors.m — Uji unit kontroler: MATLAB (dlsim) vs kode C
%
%  Langkah:
%   1) Skrip ini bikin sinyal error e[k] uji dan hitung u[k] "jawaban
%      benar" pakai dlsim(Kd2,...) di MATLAB.
%   2) Compile & jalankan test_controller_main.c (lihat instruksi di
%      akhir skrip ini) -> hasilnya y_c_test.csv.
%   3) Jalankan bagian "Bandingkan" di skrip ini untuk lihat selisihnya.
%  ===================================================================
clear; clc;
S = load('Khinf_QBC_MIL2.mat');
Kd2 = S.Kd2;   % hasil c2d(Kr, Ts2, 'tustin') -- harus ss diskret orde sama dgn Ak2 dkk.

%% 1) Sinyal uji: gabungan step + sedikit noise, supaya menguji dinamika
%  Amplitudo sengaja KECIL (0.01 V) supaya duty TIDAK pernah menyentuh
%  DUTY_MIN/DUTY_MAX di controller_step.c -- dlsim tidak tahu soal
%  saturasi, jadi tes ini cuma valid untuk memvalidasi persamaan beda
%  A/B/C/D (bagian linear). Saturasi divalidasi belakangan lewat
%  perbandingan SIL vs MIL-2 secara utuh (closed-loop), bukan di sini.
N  = 2000;                 % jumlah sampel (Ts2 = 1e-4 -> 200 ms)
k  = (0:N-1)';
e  = zeros(N,1);
e(k >= 50)  = 0.01;          % step error kecil di k=50
e(k >= 1000) = e(k>=1000) - 0.004;   % step turun lagi di k=1000 (uji dua arah)
rng(1); e = e + 0.0*randn(N,1);    % set amplitudo noise >0 kalau mau uji lebih keras

writematrix(e, 'e_test.csv');

%% 2) Jawaban "benar" dari MATLAB
u_matlab = dlsim(Kd2.A, Kd2.B, Kd2.C, Kd2.D, e);
writematrix(u_matlab, 'y_matlab_test.csv');
fprintf('Tersimpan e_test.csv dan y_matlab_test.csv (%d sampel).\n\n', N);

fprintf(['Sekarang compile & jalankan test_controller_main.c, lalu lanjut ke\n' ...
         'bagian "Bandingkan" di bawah ini. Contoh compile (gcc):\n' ...
         '  gcc -O2 controller_step.c test_controller_main.c -o test_ctrl.exe\n' ...
         '  test_ctrl.exe\n\n']);

%% 3) Bandingkan (jalankan setelah y_c_test.csv ada)
if exist('y_c_test.csv', 'file') == 2
    u_c = readmatrix('y_c_test.csv');
    if numel(u_c) ~= numel(u_matlab)
        error('Jumlah sampel y_c_test.csv (%d) != y_matlab_test.csv (%d).', ...
            numel(u_c), numel(u_matlab));
    end
    d = u_c(:) - u_matlab(:);
    fprintf('=== Bandingkan kode C vs MATLAB (dlsim) ===\n');
    fprintf('max|delta|   = %.6g\n', max(abs(d)));
    fprintf('RMS delta    = %.6g\n', rms(d));
    figure('Color','w');
    subplot(2,1,1); plot(k, u_matlab, 'b', k, u_c, 'r--'); grid on;
    legend('MATLAB (dlsim)','Kode C'); ylabel('u[k]'); title('Keluaran kontroler');
    subplot(2,1,2); plot(k, d, 'k'); grid on; ylabel('\Delta u[k]'); xlabel('k');
    title('Selisih (C - MATLAB)');
else
    fprintf('y_c_test.csv belum ada -- compile & jalankan dulu test_controller_main.c\n');
end
