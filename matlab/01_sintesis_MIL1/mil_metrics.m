%% ===================================================================
%  mil_metrics.m — Figur respons MIL-1 + parameter performa otomatis
%  (rise time, overshoot, undershoot, settling time, steady-state error,
%   ripple) untuk start-up dan uji step referensi, plus pembanding
%  prediksi linear dari hasil sintesis H-infinity.
%
%  Syarat di model Simulink:
%   - Blok To Workspace: Variable name = Vout, Save format = Timeseries
%   - Referensi: 150 -> Rate Limiter (ramp t_ramp) + Step (t_step, dV_step)
%   - File hasil sintesis (matfile) ada di folder kerja
%  Output: <fig_name>.png dan <fig_name>.csv
%  ===================================================================

%% 0) Pengaturan — samakan dengan model Simulink
model    = 'MIL';                 % nama file model .slx
matfile  = 'Khinf_QBC_MIL1.mat';  % hasil hinf_synthesis_MIL1.m
run_sim  = true;                  % false jika 'out' sudah ada di workspace
Vref0    = 150;                   % referensi nominal (V)
t_ramp   = 0.025;                 % durasi ramp di Rate Limiter (s)
t_step   = 0.2;                   % waktu step referensi (s); [] jika tidak ada
dV_step  = 5;                     % besar step referensi (V)
t_win    = 0.15;                  % jendela analisis setelah step (s)
fs       = 10e3;                  % frekuensi switching (Hz)
tol      = 0.02;                  % pita settling ±2%
dt       = 1e-6;                  % grid resampling (s)
fig_name = 'MIL2_Vout_metrics';

%% 1) Ambil data
if run_sim
    out = sim(model);
end
[t, v] = getSig(out, 'Vout');
t_end  = t(end);
if ~isempty(t_step) && t_step >= t_end
    warning('t_step >= waktu simulasi, uji step diabaikan.');
    t_step = [];
end

tu = (t(1):dt:t_end).';
vu = interp1(t, v, tu, 'linear');
N  = max(1, round(1/(fs*dt)));
va = movmean(vu, N);                         % rata-rata 1 periode switching

ref = min(Vref0, Vref0*tu/t_ramp);
if ~isempty(t_step), ref = ref + dV_step*(tu >= t_step); end

%% 2) Metrik start-up
if isempty(t_step), tA_end = t_end; else, tA_end = t_step; end
iA  = tu < tA_end;
yfA = mean(va(tu >= tA_end-0.01 & iA));
mA  = stepMetrics(tu(iA), va(iA), 0, yfA, 0, tol);
mA.ess    = abs(Vref0 - yfA)/Vref0*100;
mA.ripple = rippleOf(tu, vu, tA_end, fs);

%% 3) Metrik uji step + prediksi linear
hasStep = ~isempty(t_step);
if hasStep
    tB_end = min(t_end, t_step + t_win);
    iB  = tu >= t_step & tu < tB_end;
    y0B = mean(va(tu >= t_step-0.01 & tu < t_step));
    yfB = mean(va(tu >= tB_end-0.01 & tu < tB_end));
    if abs(yfB - y0B) < 0.3*abs(dV_step)
        warning(['Vout tidak ikut naik setelah t_step (dV terukur = %.2f V). ' ...
                 'Cek blok Step di model. Panel (b) dilewati.'], yfB - y0B);
        hasStep = false;
    else
        mB = stepMetrics(tu(iB), va(iB), y0B, yfB, t_step, tol);
        mB.ess    = abs((Vref0+dV_step) - yfB)/(Vref0+dV_step)*100;
        mB.ripple = rippleOf(tu, vu, tB_end, fs);

        % Prediksi linear (small-signal) dengan K dan G dari sintesis
        hasLin = exist(matfile, 'file') == 2;
        if hasLin
            S_  = load(matfile, 'G', 'K');
            Tcl = feedback(S_.G*S_.K, 1);
            tl  = (0:dt*10:t_win).';
            yl  = step(dV_step*Tcl, tl);
            mL  = stepMetrics(tl, yl, 0, yl(end), 0, tol);
            mL.ess = NaN; mL.ripple = NaN;
        else
            warning('%s tidak ditemukan, prediksi linear dilewati.', matfile);
        end
    end
end

%% 4) Cetak ringkasan
fprintf('=== Parameter performa MIL-1 ===\n');
printM(sprintf('Start-up 0 -> %g V', Vref0), mA);
if hasStep
    printM(sprintf('Step %g -> %g V (MIL-1)', Vref0, Vref0+dV_step), mB);
    if hasLin, printM('Step (prediksi linear)', mL); end
end

%% 5) Figur
fig = figure('Color','w','Position',[80 80 1100 760]);
try, theme(fig,'light'); catch, end
if hasStep, tiledlayout(2,1,'TileSpacing','compact'); else, tiledlayout(1,1); end

% --- (a) Start-up
nexttile; hold on; grid on; box on;
plot(tu, vu, 'Color',[0.75 0.82 0.95], 'DisplayName','V_o (switching)');
plot(tu, va, 'b', 'LineWidth',1.4, 'DisplayName','V_o (rata-rata 1 T_s)');
plot(tu, ref, 'k--', 'LineWidth',1.1, 'DisplayName','V_{ref}');
yline(yfA*(1+tol), 'g:', 'HandleVisibility','off');
yline(yfA*(1-tol), 'g:', 'DisplayName','pita \pm2%');
markM(mA);
ylim([0 1.12*(Vref0 + hasStep*dV_step)]);
xlabel('Waktu (s)'); ylabel('Tegangan output (V)');
title('(a) Respons start-up MIL-1 — QBC dengan kontroler H_\infty');
legend('Location','southeast');
txtBox(sprintf(['Rise time (10–90%%) = %.2f ms\nOvershoot = %.2f %%\n' ...
    'Settling time (\\pm2%%) = %.2f ms\nSteady-state error = %.3f %%\nRipple V_o (p-p) = %.3f V'], ...
    mA.tr*1e3, mA.OS, mA.ts*1e3, mA.ess, mA.ripple));

% --- (b) Uji step
if hasStep
    nexttile; hold on; grid on; box on;
    k = tu >= t_step-0.01 & tu < tB_end;
    plot(tu(k), vu(k), 'Color',[0.75 0.82 0.95], 'DisplayName','V_o (switching)');
    plot(tu(k), va(k), 'b', 'LineWidth',1.4, 'DisplayName','V_o MIL-1 (rata-rata 1 T_s)');
    if hasLin
        plot(t_step + tl, y0B + yl, 'm-', 'LineWidth',1.2, 'DisplayName','Prediksi linear');
    end
    plot(tu(k), ref(k), 'k--', 'LineWidth',1.1, 'DisplayName','V_{ref}');
    band = tol*abs(yfB - y0B);
    yline(yfB+band, 'g:', 'HandleVisibility','off');
    yline(yfB-band, 'g:', 'DisplayName','pita \pm2% \DeltaV');
    markM(mB);
    xlim([t_step-0.01 tB_end]);
    xlabel('Waktu (s)'); ylabel('Tegangan output (V)');
    title(sprintf('(b) Uji step referensi %g \\rightarrow %g V', Vref0, Vref0+dV_step));
    legend('Location','southeast');
    txtBox(sprintf(['MIL-1:\nRise time (10–90%%) = %.2f ms\nOvershoot = %.2f %%\nUndershoot = %.2f %%\n' ...
        'Settling time (\\pm2%%) = %.2f ms\nSteady-state error = %.3f %%'], ...
        mB.tr*1e3, mB.OS, mB.US, mB.ts*1e3, mB.ess));
end

exportgraphics(fig, [fig_name '.png'], 'Resolution', 300);

%% 6) Tabel metrik (untuk Bab IV / perbandingan MIL-SIL-PIL)
names = {'Startup_MIL1'}; M = mA;
if hasStep
    names{end+1} = 'Step_MIL1'; M(end+1) = mB;
    if hasLin, names{end+1} = 'Step_Linear'; M(end+1) = mL; end
end
T = struct2table(rmfield(M, {'tpk','ypk','t10','t90','tsAbs'}));
T.Properties.RowNames = names;
T.Properties.VariableNames = {'RiseTime_s','Overshoot_pct','Undershoot_pct', ...
    'SettlingTime_s','PeakTime_s','SSError_pct','RippleVpp'};
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