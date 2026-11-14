%% ===================================================================
%  export_controller_c.m
%  Menulis koefisien kontroler H-infinity diskret (Ak2,Bk2,Ck2,Dk2,Ts2)
%  dari Khinf_QBC_MIL2.mat ke controller_coeffs.h (presisi penuh, double).
%
%  Jalankan SETELAH hinf_discretize_MIL2.m (atau load Khinf_QBC_MIL2.mat).
%  Duty_min/Duty_max HARUS kamu isi sendiri sesuai parameter blok
%  Saturation yang sebenarnya di Simulink -- skrip ini tidak menebaknya.
%  ===================================================================
clear; clc;
S = load('Khinf_QBC_MIL2.mat');
A = S.Ak2; B = S.Bk2; C = S.Ck2; D = S.Dk2; Ts = S.Ts2;
n = size(A,1);

% <<< ISI SESUAI BLOK SATURATION DI SIMULINK (bukan dugaan) >>>
Duty_min = -5;
Duty_max = 5;

f = fopen('controller_coeffs.h', 'w');
fprintf(f, '/* Auto-generated oleh export_controller_c.m -- JANGAN diedit manual.\n');
fprintf(f, '   Sumber: Khinf_QBC_MIL2.mat (Ak2,Bk2,Ck2,Dk2,Ts2), orde %d. */\n', n);
fprintf(f, '#ifndef CONTROLLER_COEFFS_H\n#define CONTROLLER_COEFFS_H\n\n');
% %.17e (notasi ilmiah) dipakai, bukan %.17g -- supaya literalnya SELALU
% valid di C (selalu ada '.' dan 'e' sebelum suffix 'f'). %.17g untuk
% nilai bulat seperti 1.0 akan menulis "1" lalu jadi "1f" yang BUKAN
% angka float valid di C (error "bad suffix on number").
fprintf(f, '#define CTRL_ORDER %d\n', n);
fprintf(f, '#define CTRL_TS    %.17ef\n', Ts);
fprintf(f, '#define DUTY_MIN   %.17ef\n', Duty_min);
fprintf(f, '#define DUTY_MAX   %.17ef\n\n', Duty_max);

fprintf(f, 'static const float CTRL_A[CTRL_ORDER][CTRL_ORDER] = {\n');
for i = 1:n
    fprintf(f, '    { ');
    fprintf(f, '%.17ef, ', A(i,:));
    fprintf(f, '},\n');
end
fprintf(f, '};\n\n');

fprintf(f, 'static const float CTRL_B[CTRL_ORDER] = { ');
fprintf(f, '%.17ef, ', B(:));
fprintf(f, '};\n\n');

fprintf(f, 'static const float CTRL_C[CTRL_ORDER] = { ');
fprintf(f, '%.17ef, ', C(:));
fprintf(f, '};\n\n');

fprintf(f, 'static const float CTRL_D = %.17ef;\n\n', D);
fprintf(f, '#endif /* CONTROLLER_COEFFS_H */\n');
fclose(f);

fprintf('Tersimpan: controller_coeffs.h (orde %d, Ts=%.3g s, Duty=[%.3f, %.3f])\n', ...
    n, Ts, Duty_min, Duty_max);
fprintf('>>> CEK LAGI Duty_min/Duty_max di atas -- samakan dgn blok Saturation Simulink!\n');
