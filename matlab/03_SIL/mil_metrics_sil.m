%% ===================================================================
%  mil_metrics.m — Figur respons MIL + parameter performa otomatis
%  (versi 10 Okt 2026: step referensi 150->155 V DIHAPUS)
%
%  Skenario:
%   (a) Start-up 0 -> Vref0 : rise time, overshoot, undershoot,
%       settling time, steady-state error, ripple
%   (b) Load step (opsional, aktif bila t_load tidak kosong): deviasi
%       puncak, waktu pemulihan, NRMSE terhadap Vref0
%
%  Syarat di model Simulink:
%   - Blok To Workspace: Variable name = Vout, Save format = Timeseries
%   - Referensi: Vref0 -> Rate Limiter (ramp t_ramp). Tidak ada blok Step referensi.
%   - File hasil sintesis Khinf_QBC_MIL1.mat ada di folder kerja
%  Output: <fig_name>.png, <fig_name>.csv (+ <fig_name>_load.csv bila ada load step)
%  Jalankan dari Command Window (agar variabel ada di base workspace).
%  ===================================================================

%% 0) Pengaturan — samakan dengan model Simulink
model    = 'SIL';
matfile  = 'Khinf_QBC_MIL2.mat';   % tetap, cuma dipakai plant/referensi lain
run_sim  = true;                  % false jika 'out' sudah ada di workspace
Vref0    = 150;                   % referensi nominal (V)
t_ramp   = 0.025;                 % durasi ramp di Rate Limiter (s)
fs       = 10e3;                  % frekuensi switching (Hz)
tol      = 0.02;                  % pita settling +-2%
dt       = 1e-6;                  % grid resampling (s)

% Load step (isi t_load = [] bila belum dipakai)
t_load   = 0.2;                    % waktu load step (s), mis. 0.2
t_win    = 0.15;                  % jendela analisis setelah load step (s)
band_rec = 0.02;                  % pita pemulihan, fraksi dari Vref0 (+-2% = +-3 V); sesuaikan bila dosen menentukan lain

fig_name = 'SIL_Vout_metrics';

%% 1) Ambil data
if run_sim
    if exist(matfile, 'file') == 2
        load(matfile);            % membawa K, Kd, Ts_ctrl, G, ... ke workspace
    else
        warning('%s tidak ditemukan; pastikan Kd dan Ts_ctrl sudah ada di workspace.', matfile);
    end
    Ts  = 1e-7;                   % sample time powergui / blok Discrete State-Space (MIL-1)
    fs  = 10e3;                   % (load di atas dapat menimpa; samakan lagi)
    % tambahkan ini sebelum baris "out = sim(model);"
    Vin_nom  = 18;
    Vin_step = Vin_nom;   % sama dengan nominal -> Step tidak benar2 mengubah apa pun
    t_vin    = 1;         % di luar durasi simulasi, tidak pernah terpicu
    out = sim(model);
end
[t, v] = getSig(out, 'Vout');
t_end  = t(end);
if ~isempty(t_load) && t_load >= t_end
    warning('t_load >= waktu simulasi, uji load step diabaikan.');
    t_load = 0.2;
end

tu = (t(1):dt:t_end).';
vu = interp1(t, v, tu, 'linear');
N  = max(1, round(1/(fs*dt)));
va = movmean(vu, N);                          % rata-rata 1 periode switching
ref = min(Vref0, Vref0*tu/t_ramp);

%% 2) Metrik start-up
if isempty(t_load), tA_end = t_end; else, tA_end = t_load; end
iA  = tu < tA_end;
yfA = mean(va(tu >= tA_end-0.01 & iA));
mA  = stepMetrics(tu(iA), va(iA), 0, yfA, 0, tol);
mA.ess    = abs(Vref0 - yfA)/Vref0*100;
mA.ripple = rippleOf(tu, vu, tA_end, fs);

%% 3) Metrik load step (opsional)
hasLoad = ~isempty(t_load);
if hasLoad
    tB_end = min(t_end, t_load + t_win);
    iB  = tu >= t_load & tu < tB_end;
    V0  = mean(va(tu >= t_load-0.01 & tu < t_load));   % Vout sebelum gangguan
    dev = va(iB) - V0;
    [~, ip] = max(abs(dev));
    tB  = tu(iB);
    mL.dev_V    = dev(ip);                              % deviasi puncak bertanda (V)
    mL.dev_pct  = abs(dev(ip))/Vref0*100;               % terhadap Vref0
    mL.tpk      = tB(ip);
    mL.ypk      = va(find(iB,1,'first') + ip - 1);
    iout = find(abs(va(iB) - Vref0) > band_rec*Vref0, 1, 'last');
    if isempty(iout)
        mL.trec = 0;                                    % tidak pernah keluar pita
    elseif iout >= numel(tB)
        mL.trec = NaN;                                  % belum pulih dalam jendela
    else
        mL.trec = tB(iout+1) - t_load;
    end
    mL.nrmse = sqrt(mean((va(iB) - Vref0).^2))/Vref0*100;
    mL.V0    = V0;
    mL.Vend  = mean(va(tu >= tB_end-0.01 & tu < tB_end));
end

%% 4) Cetak ringkasan
fprintf('=== Parameter performa MIL ===\n');
printM(sprintf('Start-up 0 -> %g V', Vref0), mA);
if hasLoad
    fprintf('\n[Load step pada t = %g s]\n', t_load);
    fprintf('  Vout sebelum         = %.3f V\n', mL.V0);
    fprintf('  Deviasi puncak       = %.3f V (%.3f %% Vref)\n', mL.dev_V, mL.dev_pct);
    fprintf('  Waktu pemulihan      = %.3f ms (pita +-%g%% Vref)\n', mL.trec*1e3, band_rec*100);
    fprintf('  NRMSE (thd Vref)     = %.3f %%\n', mL.nrmse);
end

%% 5) Figur
fig = figure('Color','w','Position',[80 80 1100 760]);
try, theme(fig,'light'); catch, end
if hasLoad, tiledlayout(2,1,'TileSpacing','compact'); else, tiledlayout(1,1); end

% --- (a) Start-up
nexttile; hold on; grid on; box on;
plot(tu, vu, 'Color',[0.75 0.82 0.95], 'DisplayName','V_o (switching)');
plot(tu, va, 'b', 'LineWidth',1.4, 'DisplayName','V_o (rata-rata 1 T_s)');
plot(tu, ref, 'k--', 'LineWidth',1.1, 'DisplayName','V_{ref}');
yline(yfA*(1+tol), 'g:', 'HandleVisibility','off');
yline(yfA*(1-tol), 'g:', 'DisplayName','pita \pm2%');
markM(mA);
ylim([0 1.12*Vref0]);
xlabel('Waktu (s)'); ylabel('Tegangan output (V)');
title('(a) Respons start-up SIL --- QBC dengan kontroler H_\infty');
legend('Location','southeast');
txtBox(sprintf(['Rise time (10–90%%) = %.2f ms\nOvershoot = %.2f %%\n' ...
    'Settling time (\\pm2%%) = %.2f ms\nSteady-state error = %.3f %%\nRipple V_o (p-p) = %.3f V'], ...
    mA.tr*1e3, mA.OS, mA.ts*1e3, mA.ess, mA.ripple));

% --- (b) Load step
if hasLoad
    nexttile; hold on; grid on; box on;
    k = tu >= t_load-0.01 & tu < tB_end;
    plot(tu(k), vu(k), 'Color',[0.75 0.82 0.95], 'DisplayName','V_o (switching)');
    plot(tu(k), va(k), 'b', 'LineWidth',1.4, 'DisplayName','V_o (rata-rata 1 T_s)');
    yline(Vref0, 'k--', 'LineWidth',1.1, 'DisplayName','V_{ref}');
    yline(Vref0*(1+band_rec), 'g:', 'HandleVisibility','off');
    yline(Vref0*(1-band_rec), 'g:', 'DisplayName','pita pemulihan');
    plot(mL.tpk, mL.ypk, 'rv', 'MarkerFaceColor','r', 'DisplayName','deviasi puncak');
    xline(t_load, ':', 'Color',[0.85 0.4 0], 'DisplayName','load step');
    xlim([t_load-0.01 tB_end]);
    xlabel('Waktu (s)'); ylabel('Tegangan output (V)');
    title('(b) Respons terhadap load step');
    legend('Location','southeast');
    txtBox(sprintf(['Deviasi puncak = %.2f V (%.2f %%)\nWaktu pemulihan = %.2f ms\nNRMSE = %.3f %%'], ...
        mL.dev_V, mL.dev_pct, mL.trec*1e3, mL.nrmse));
end

exportgraphics(fig, [fig_name '.png'], 'Resolution', 300);

%% 6) Tabel metrik (untuk Bab IV / perbandingan MIL-SIL-PIL)
T = struct2table(rmfield(mA, {'tpk','ypk','t10','t90','tsAbs'}));
T.Properties.RowNames = {'Startup_SIL'};
T.Properties.VariableNames = {'RiseTime_s','Overshoot_pct','Undershoot_pct', ...
    'SettlingTime_s','PeakTime_s','SSError_pct','RippleVpp'};
disp(T);
writetable(T, [fig_name '.csv'], 'WriteRowNames', true);
if hasLoad
    TL = table(mL.V0, mL.dev_V, mL.dev_pct, mL.trec, mL.nrmse, ...
        'VariableNames', {'V_before','PeakDev_V','PeakDev_pct','RecoveryTime_s','NRMSE_pct'}, ...
        'RowNames', {'LoadStep_SIL'});
    disp(TL);
    writetable(TL, [fig_name '_load.csv'], 'WriteRowNames', true);
end
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

function m = stepMetrics(t, y, y0, yf, t0, tol)
    dy = yf - y0;
    yn = (y - y0)/dy;
    i90 = find(yn >= 0.9, 1);
    if isempty(i90), i10 = []; else, i10 = find(yn(1:i90) <= 0.1, 1, 'last'); end
    if isempty(i90) || isempty(i10)
        m.tr = NaN; m.t10 = NaN; m.t90 = NaN;
    else
        m.tr = t(i90) - t(i10); m.t10 = t(i10); m.t90 = t(i90);
    end
    [pk, ipk] = max(yn);
    m.OS = max(0, (pk - 1)*100);
    m.US = max(0, -min(yn(1:ipk))*100);
    iout = find(abs(y - yf) > tol*abs(dy), 1, 'last');
    if isempty(iout), m.ts = 0; else, m.ts = t(min(iout+1, numel(t))) - t0; end
    m.tsAbs = t0 + m.ts;
    m.Tp  = t(ipk) - t0;
    m.tpk = t(ipk); m.ypk = y(ipk);
    m.ess = NaN; m.ripple = NaN;
    m = orderfields(m, {'tr','OS','US','ts','Tp','ess','ripple','tpk','ypk','t10','t90','tsAbs'});
end

function r = rippleOf(tu, vu, t_ref, fs)
    k = tu >= t_ref - 5/fs & tu < t_ref;
    r = max(vu(k)) - min(vu(k));
end

function markM(m)
    if m.OS > 0.01
        plot(m.tpk, m.ypk, 'rv', 'MarkerFaceColor','r', 'DisplayName','puncak');
    end
    if ~isnan(m.tr)
        xline(m.t10, ':', 'Color',[0.85 0.4 0], 'HandleVisibility','off');
        xline(m.t90, ':', 'Color',[0.85 0.4 0], 'DisplayName','t_{10%}, t_{90%}');
    end
    xline(m.tsAbs, '-.', 'Color',[0.1 0.6 0.1], 'LineWidth',1.1, 'DisplayName','t_s');
end

function txtBox(s)
    text(0.02, 0.95, s, 'Units','normalized', 'VerticalAlignment','top', ...
        'FontSize',9, 'BackgroundColor','w', 'EdgeColor',[0.6 0.6 0.6], 'Margin',4);
end

function printM(label, m)
    fprintf('\n[%s]\n', label);
    fprintf('  Rise time (10-90%%)   = %.3f ms\n', m.tr*1e3);
    fprintf('  Overshoot            = %.2f %%\n', m.OS);
    fprintf('  Undershoot           = %.2f %%\n', m.US);
    fprintf('  Settling time (±2%%)  = %.3f ms\n', m.ts*1e3);
    fprintf('  Peak time            = %.3f ms\n', m.Tp*1e3);
    fprintf('  Steady-state error   = %.3f %%\n', m.ess);
    fprintf('  Ripple Vo (p-p)      = %.3f V\n', m.ripple);
end