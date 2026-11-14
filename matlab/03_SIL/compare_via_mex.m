%% ===================================================================
%  compare_via_mex.m — Uji unit kontroler lewat MEX (ganti path CSV/exe)
%  Syarat: controller_mex.c sudah di-compile: mex controller_mex.c controller_step.c
%          e_test.csv dan y_matlab_test.csv sudah ada (dari gen_test_vectors.m)
%  ===================================================================
clear; clc;
e        = readmatrix('e_test.csv');
u_matlab = readmatrix('y_matlab_test.csv');

u_c = controller_mex(e);

d = u_c(:) - u_matlab(:);
fprintf('=== Bandingkan kode C (via MEX) vs MATLAB (dlsim) ===\n');
fprintf('max|delta|   = %.6g\n', max(abs(d)));
fprintf('RMS delta    = %.6g\n', rms(d));

k = (0:numel(e)-1)';
figure('Color','w');
subplot(2,1,1); plot(k, u_matlab, 'b', k, u_c, 'r--'); grid on;
legend('MATLAB (dlsim)','Kode C (mex)'); ylabel('u[k]'); title('Keluaran kontroler');
subplot(2,1,2); plot(k, d, 'k'); grid on; ylabel('\Delta u[k]'); xlabel('k');
title('Selisih (C - MATLAB)');
