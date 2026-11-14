%% ===================================================================
%  mil_metrics2_vin.m — Uji variasi Vin +-10% pada MIL-2
%  (pola sama dengan bagian load-step di mil_metrics2.m, tapi
%   gangguannya sekarang perubahan Vin, bukan perubahan R)
%
%  Syarat di Simulink:
%   - Sumber Vin diganti dari DC tetap -> Controlled Voltage Source
%     (atau sejenis) yang digerakkan sinyal Simulink "Vin_sig".
%   - Beban R tetap nominal (tidak berubah) selama uji ini -> supaya
%     satu uji hanya mengandung satu jenis gangguan.
%   - Blok To Workspace: Variable name = Vout, Save format = Timeseries
%   - InitFcn model: load('Khinf_QBC_MIL2.mat');
%
%  Dua kali jalan: Vin +10% lalu Vin -10% (ganti vin_pct di bawah).
%  Output: MIL2_Vin_<+10|-10>_metrics.png/.csv
%  ===================================================================

%% 0) Pengaturan
model    = 'MIL2';                % <-- FIX: dulu sempat 'SIL' (salah pakai untuk file ini)
matfile  = 'Khinf_QBC_MIL2.mat';
Vref0    = 150;
t_ramp   = 0.025;
fs       = 10e3;
dt       = 1e-6;

Vin_nom  = 18;          % Vin nominal (V) -- samakan dgn parameter rangkaian
t_vin    = 0.2;         % waktu Vin berubah (s) -- beri jeda dari start-up (>= 4x ts)
t_win    = 0.15;        % jendela analisis setelah perubahan (s)
band_rec = 0.02;        % pita pemulihan, fraksi dari Vref0

vin_pct  = -0.10;        % <-- GANTI: +0.10 lalu jalankan lagi dengan -0.10
Vin_step = Vin_nom*(1+vin_pct);

fig_name = sprintf('MIL2_Vin_%+.0f_metrics', vin_pct*100);

fprintf('=== Uji Vin %+.0f%% (%.2f V -> %.2f V) pada t = %.3f s ===\n', ...
    vin_pct*100, Vin_nom, Vin_step, t_vin);

%% 1) Set variabel sinyal Vin untuk dibaca blok Step di Simulink
%  Di Simulink: blok Step "Vin_sig" -> Step time = t_vin,
%  Initial value = Vin_nom, Final value = Vin_step (Simulink akan
%  membaca variabel2 ini langsung dari base workspace lewat nama yang
%  sama kalau kotak isian Step diisi "Vin_nom" dan "Vin_step").
load(matfile);
out = sim(model);

%% 2) Ambil Vout, resample, rata-rata 1 periode
[t, v] = getSig(out, 'Vout');
t_end  = t(end);
if t_vin >= t_end
    error('t_vin >= waktu simulasi. Perpanjang Stop Time model dulu.');
end
tu = (t(1):dt:t_end).';
vu = interp1(t, v, tu, 'linear');
N  = max(1, round(1/(fs*dt)));
va = movmean(vu, N);

%% 3) Metrik gangguan (sama pola dgn load step)
tB_end = min(t_end, t_vin + t_win);
iB  = tu >= t_vin & tu < tB_end;
V0  = mean(va(tu >= t_vin-0.01 & tu < t_vin));
dev = va(iB) - V0;
[~, ip] = max(abs(dev));
tB  = tu(iB);

m.dev_V   = dev(ip);
m.dev_pct = abs(dev(ip))/Vref0*100;
m.tpk     = tB(ip);
m.ypk     = va(find(iB,1,'first') + ip - 1);
iout = find(abs(va(iB) - Vref0) > band_rec*Vref0, 1, 'last');
if isempty(iout)
    m.trec = 0;
elseif iout >= numel(tB)
    m.trec = NaN;
else
    m.trec = tB(iout+1) - t_vin;
end
m.nrmse = sqrt(mean((va(iB) - Vref0).^2))/Vref0*100;
m.V0    = V0;

fprintf('Vout sebelum         = %.3f V\n', m.V0);
fprintf('Deviasi puncak       = %.3f V (%.3f %% Vref)\n', m.dev_V, m.dev_pct);
fprintf('Waktu pemulihan      = %.3f ms (pita +-%g%% Vref)\n', m.trec*1e3, band_rec*100);
fprintf('NRMSE (thd Vref)     = %.3f %%\n\n', m.nrmse);

%% 4) Figur
fig = figure('Color','w','Position',[80 80 1000 480]);
try, theme(fig,'light'); catch, end
hold on; grid on; box on;
k = tu >= t_vin-0.01 & tu < tB_end;
plot(tu(k), vu(k), 'Color',[0.75 0.82 0.95], 'DisplayName','V_o (switching)');
plot(tu(k), va(k), 'b', 'LineWidth',1.4, 'DisplayName','V_o (rata-rata 1 T_s)');
yline(Vref0, 'k--', 'LineWidth',1.1, 'DisplayName','V_{ref}');
yline(Vref0*(1+band_rec), 'g:', 'HandleVisibility','off');
yline(Vref0*(1-band_rec), 'g:', 'DisplayName','pita pemulihan');
plot(m.tpk, m.ypk, 'rv', 'MarkerFaceColor','r', 'DisplayName','deviasi puncak');
xline(t_vin, ':', 'Color',[0.85 0.4 0], 'DisplayName', sprintf('Vin %+.0f%%', vin_pct*100));
xlim([t_vin-0.01 tB_end]);
xlabel('Waktu (s)'); ylabel('Tegangan output (V)');
title(sprintf('Respons terhadap variasi Vin %+.0f%% (%.1f -> %.1f V)', vin_pct*100, Vin_nom, Vin_step));
legend('Location','southeast');
text(0.02, 0.95, sprintf(['Deviasi puncak = %.2f V (%.2f %%)\nWaktu pemulihan = %.2f ms\nNRMSE = %.3f %%'], ...
    m.dev_V, m.dev_pct, m.trec*1e3, m.nrmse), 'Units','normalized', 'VerticalAlignment','top', ...
    'FontSize',9, 'BackgroundColor','w', 'EdgeColor',[0.6 0.6 0.6], 'Margin',4);
exportgraphics(fig, [fig_name '.png'], 'Resolution', 300);

%% 5) Tabel
T = table(m.V0, m.dev_V, m.dev_pct, m.trec, m.nrmse, ...
    'VariableNames', {'V_before','PeakDev_V','PeakDev_pct','RecoveryTime_s','NRMSE_pct'}, ...
    'RowNames', {sprintf('Vin%+.0f_MIL2', vin_pct*100)});
disp(T);
writetable(T, [fig_name '.csv'], 'WriteRowNames', true);
fprintf('Tersimpan: %s.png dan %s.csv\n', fig_name, fig_name);

%% ===================== Fungsi lokal =====================
function [t, v] = getSig(out, name)
    names = out.who;
    if any(strcmp(names, name))
        s = out.get(name);
    elseif any(strcmp(names, 'logsout')) && ~isempty(out.logsout.find('Name', name))
        s = out.logsout.getElement(name).Values;
    else
        error(['Sinyal "%s" tidak ditemukan. Variabel di out: %s\n' ...
               'Tambahkan To Workspace (Variable name = %s, Save format = Timeseries).'], ...
               name, strjoin(names, ', '), name);
    end
    if isa(s, 'timeseries')
        t = s.Time; v = squeeze(s.Data);
    elseif isstruct(s)
        if isfield(s,'time') && ~isempty(s.time), t = s.time; else, t = out.tout; end
        v = s.signals.values;
    else
        v = s; t = out.tout;
    end
    t = double(t(:)); v = double(v(:));
    [t, iu] = unique(t); v = v(iu);
end
