function compare_scenarios(dataRoot, outDir)
%% COMPARE_SCENARIOS  Membuat Gambar 3.10, 4.8, dan 4.43 untuk Tugas Akhir
%  Gaya visual dibuat konsisten dengan analyze_run.m
%  (warna robot sama, penanda waypoint/goal sama, latar peta opsional).
%
%  Menghasilkan:
%    Gambar 3.10  Layout & lintasan 4 skenario (convoy/split/merge/crossing)
%                 -> gambar_3_10_layout_skenario.png
%    Gambar 4.8   Karakterisasi Within-Run (6 gambar terpisah):
%                 (a) deviasi progres antarrobot - split (konsensus+fault)
%                 (b) penyelarasan progres p_i(t) menuju rata-rata - convoy
%                 (c) konvergensi konsensus - crossing
%                 (d) margin keselamatan antarrobot - split (konsensus+fault)
%                 (e) margin keselamatan antarrobot - convoy
%                 (f) distribusi cross-track error per robot (boxplot)
%                 -> gambar_4_8a..f_*.png
%    Gambar 4.43  Perbandingan skenario split antar-kondisi (4 metrik)
%                 -> gambar_4_43_perbandingan_split.png
%
%  CARA PAKAI:
%    1) Taruh SEMUA folder run di dalam SATU folder induk, mis. 'data fix':
%         data fix/1.split_baseline_01_...
%         data fix/2.split_cons_nofault_01_...
%         data fix/3.split_cons_fault_01_...
%         data fix/4.convoy_cons_fault_01_...
%         data fix/5.merge_cons_fault_01_...
%         data fix/6.crossing_cons_fault_01_...
%    2) Jalankan salah satu:
%         compare_scenarios                      % pakai folder default di bawah
%         compare_scenarios('data fix')          % tentukan folder data
%         compare_scenarios('data fix','keluaran')% + folder keluaran
%  Nama folder run dicocokkan otomatis via kata kunci, jadi timestamp bebas.

    %% ==================== KONFIGURASI ====================
    if nargin < 1 || isempty(dataRoot)
        dataRoot = 'data fix';      % <-- GANTI ke nama folder berisi semua run
    end
    if ~isfolder(dataRoot)
        for a = {'data fix','data_fix','data','.'}
            if isfolder(a{1}); dataRoot = a{1}; break; end
        end
    end
    if nargin < 2 || isempty(outDir); outDir = 'gambar_ta'; end
    if ~isfolder(outDir); mkdir(outDir); end

    D_EMERGENCY = 0.30;   % ambang keselamatan jarak antar-robot (m)
    CONV_THRESH = 0.02;   % ambang konvergensi konsensus

    % Teks polos (tanpa TeX) supaya underscore/karakter khusus aman & tak error
    set(groot,'defaultTextInterpreter','none');
    set(groot,'defaultLegendInterpreter','none');
    set(groot,'defaultAxesTickLabelInterpreter','none');

    robots = {'robot1','robot2','robot3'};
    fprintf('dataRoot = %s\n', dataRoot);

    % ---- Resolusi folder run per skenario/kondisi (via kata kunci) ----
    runConvoy  = findRun(dataRoot,'convoy_cons_fault');
    runSplitNF = findRun(dataRoot,'split_cons_nofault');
    runMerge   = findRun(dataRoot,'merge_cons_fault');
    runCross   = findRun(dataRoot,'crossing_cons_fault');
    runSplitBL = findRun(dataRoot,'split_baseline');
    runSplitF  = findRun(dataRoot,'split_cons_fault');
    runOffset  = findRun(dataRoot,'convoy_offset');

    % ---- Peta (opsional; dilewati bila .pgm tak ada) ----
    M = loadMapAuto(dataRoot, {runConvoy,runSplitNF,runMerge,runCross});

    %% ==================== GAMBAR 3.10 ====================
    panels = { runConvoy,'convoy','(a) convoy'; runSplitNF,'split','(b) split'; ...
               runMerge,'merge','(c) merge'; runCross,'crossing','(d) crossing' };
    f310 = figure('Name','Gambar 3.10','Color','w','Position',[60 60 1100 900]);
    tl = tiledlayout(f310,2,2,'TileSpacing','compact','Padding','compact');
    ax1 = [];
    for k = 1:size(panels,1)
        ax = nexttile(tl); hold(ax,'on');
        if k==1; ax1 = ax; end
        rd = panels{k,1}; scn = panels{k,2}; ttl = panels{k,3};
        drawMap(M);
        S = scenarioWaypoints(scn);
        if ~isempty(rd)
            % Hanya JALUR GLOBAL (path_log). Lintasan aktual/local sengaja tidak digambar.
            pth = tryRead(fullfile(rd,'path_log.csv'));
            for i = 1:numel(robots)
                r = robots{i}; c = colOf(r);
                if ~isempty(pth)
                    m = strcmp(string(pth.robot), r);
                    if any(m)
                        gp = sortrows(pth(m,:),'point_index');
                        plot(ax, gp.x, gp.y, '-','Color',c,'LineWidth',2.0,'HandleVisibility','off');
                        plot(ax, gp.x(1), gp.y(1), 'o','MarkerSize',8,'MarkerEdgeColor',c, ...
                             'MarkerFaceColor','w','LineWidth',1.5,'HandleVisibility','off');
                    end
                end
                drawWaypoints(S, r, c);
            end
        else
            text(ax,0.5,0.5,'(folder run tidak ditemukan)','Units','normalized', ...
                 'HorizontalAlignment','center','Color',[0.6 0 0]);
        end
        axis(ax,'equal'); grid(ax,'on'); box(ax,'on');
        xlabel(ax,'x (m)'); ylabel(ax,'y (m)'); title(ax,ttl);
    end
    % ---- legenda bersama (di bawah) ----
    if ~isempty(ax1)
        p1  = plot(ax1,nan,nan,'-','Color',colOf('robot1'),'LineWidth',2);
        p2  = plot(ax1,nan,nan,'-','Color',colOf('robot2'),'LineWidth',2);
        p3  = plot(ax1,nan,nan,'-','Color',colOf('robot3'),'LineWidth',2);
        ps  = plot(ax1,nan,nan,'o','MarkerEdgeColor','k','MarkerFaceColor','w','LineWidth',1.2);
        pw  = plot(ax1,nan,nan,'s','MarkerEdgeColor','k','MarkerFaceColor','none','LineWidth',1.2);
        pgo = plot(ax1,nan,nan,'p','MarkerEdgeColor','k','MarkerFaceColor','k','MarkerSize',10);
        lg  = legend([p1 p2 p3 ps pw pgo], ...
            {'robot1','robot2','robot3','start (o)','waypoint (kotak)','goal (bintang)'}, ...
            'Orientation','horizontal','NumColumns',3,'FontSize',8);
        try; lg.Layout.Tile = 'south'; catch; end
    end
    title(tl,'Layout dan lintasan empat skenario pengujian','FontWeight','bold');
    savePNG(f310, fullfile(outDir,'gambar_3_10_layout_skenario.png'));

    %% ============= GAMBAR 4.8  (Subbab: Karakterisasi Within-Run) =============
    %  Subbab 4.8 memuat 6 gambar; masing-masing disimpan sebagai berkas PNG terpisah.

    % ---- 4.8(a) Deviasi progres antarrobot vs waktu - split (konsensus+fault) ----
    fA = figure('Name','Gambar 4.8a','Color','w','Position',[100 100 780 430]);
    axA = axes('Parent',fA); hold(axA,'on'); grid(axA,'on'); box(axA,'on');
    plotMaxDev(axA, runSplitF, CONV_THRESH);
    title(axA,'Deviasi progres antarrobot - split (konsensus + fault)');
    savePNG(fA, fullfile(outDir,'gambar_4_8a_deviasi_split_fault.png'));

    % ---- 4.8(b) Penyelarasan progres p_i(t) menuju rata-rata - convoy ----
    fB = figure('Name','Gambar 4.8b','Color','w','Position',[100 100 780 430]);
    axB = axes('Parent',fB); hold(axB,'on'); grid(axB,'on'); box(axB,'on');
    plotProgress(axB, runConvoy, robots);
    title(axB,'Penyelarasan progres lintasan menuju rata-rata - convoy');
    savePNG(fB, fullfile(outDir,'gambar_4_8b_progres_convoy.png'));

    % ---- 4.8(c) Konvergensi konsensus - crossing (interaksi terpadat) ----
    fC = figure('Name','Gambar 4.8c','Color','w','Position',[100 100 780 430]);
    axC = axes('Parent',fC); hold(axC,'on'); grid(axC,'on'); box(axC,'on');
    plotMaxDev(axC, runCross, CONV_THRESH);
    title(axC,'Konvergensi konsensus - crossing (interaksi terpadat)');
    savePNG(fC, fullfile(outDir,'gambar_4_8c_konvergensi_crossing.png'));

    % ---- 4.8(d) Margin keselamatan antarrobot vs waktu - split (konsensus+fault) ----
    fD = figure('Name','Gambar 4.8d','Color','w','Position',[100 100 780 400]);
    axD = axes('Parent',fD); hold(axD,'on'); grid(axD,'on'); box(axD,'on');
    plotMinDist(axD, runSplitF, D_EMERGENCY);
    title(axD,'Margin keselamatan antarrobot - split (konsensus + fault)');
    savePNG(fD, fullfile(outDir,'gambar_4_8d_margin_split_fault.png'));

    % ---- 4.8(e) Margin keselamatan antarrobot - convoy (beriringan rapat) ----
    fE = figure('Name','Gambar 4.8e','Color','w','Position',[100 100 780 400]);
    axE = axes('Parent',fE); hold(axE,'on'); grid(axE,'on'); box(axE,'on');
    plotMinDist(axE, runConvoy, D_EMERGENCY);
    title(axE,'Margin keselamatan antarrobot - convoy (beriringan rapat)');
    savePNG(fE, fullfile(outDir,'gambar_4_8e_margin_convoy.png'));

    % ---- 4.8(f) Distribusi cross-track error per robot antar-skenario (boxplot) ----
    fF = figure('Name','Gambar 4.8f','Color','w','Position',[80 80 1000 470]);
    axF = axes('Parent',fF); hold(axF,'on'); grid(axF,'on'); box(axF,'on');
    boxRuns = { runSplitBL,'baseline'; runSplitNF,'no-fault'; runSplitF,'fault'; ...
                runConvoy,'convoy'; runMerge,'merge'; runCross,'crossing'; runOffset,'offset' };
    xpos = 0; ticks = []; ticklab = {};
    for s = 1:size(boxRuns,1)
        rd = boxRuns{s,1}; scn = boxRuns{s,2};
        if isempty(rd); continue; end
        ct = tryRead(fullfile(rd,'crosstrack_log.csv'));
        if isempty(ct) || ~ismember('crosstrack_error_m',ct.Properties.VariableNames); continue; end
        for i = 1:numel(robots)
            r = robots{i}; m = strcmp(string(ct.robot), r);
            if ~any(m); continue; end
            v = abs(toNum(ct.crosstrack_error_m(m)));
            xpos = xpos + 1;
            drawBox(axF, xpos, v, colOf(r), 0.32);
            ticks(end+1) = xpos; ticklab{end+1} = sprintf('%s R%d',scn,i); %#ok<AGROW>
        end
        xpos = xpos + 1;   % jarak antar-skenario
    end
    yline(axF, 0.30, '--r', 'ambang 0,30 m', 'LineWidth',1.0,'HandleVisibility','off');
    hp1 = plot(axF,nan,nan,'s','MarkerFaceColor',colOf('robot1'),'MarkerEdgeColor','none','MarkerSize',9);
    hp2 = plot(axF,nan,nan,'s','MarkerFaceColor',colOf('robot2'),'MarkerEdgeColor','none','MarkerSize',9);
    hp3 = plot(axF,nan,nan,'s','MarkerFaceColor',colOf('robot3'),'MarkerEdgeColor','none','MarkerSize',9);
    legend([hp1 hp2 hp3],{'robot1','robot2','robot3'},'Location','northwest','FontSize',8);
    if ~isempty(ticks); set(axF,'XTick',ticks,'XTickLabel',ticklab); xtickangle(axF,90); xlim(axF,[0 xpos]); end
    ylabel(axF,'cross-track error |e| (m)');
    title(axF,'Distribusi cross-track error per robot antar-skenario (within-run)');
    savePNG(fF, fullfile(outDir,'gambar_4_8f_boxplot_xte.png'));

    %% ==================== GAMBAR 4.43 ====================
    condRuns  = { runSplitBL, runSplitNF, runSplitF };
    condLbl   = {'tanpa konsensus','konsensus tanpa fault','konsensus + fault'};
    condShort = {'baseline','no-fault','fault'};
    condCol   = [0.50 0.50 0.50; 0.12 0.47 0.71; 0.84 0.15 0.16];
    nC = numel(condRuns);

    spread  = nan(1,nC);
    mindist = nan(1,nC);
    arr     = nan(nC,numel(robots));
    for k = 1:nC
        rd = condRuns{k}; if isempty(rd); continue; end
        a = arrivalByRobot(rd, robots); arr(k,:) = a;
        spread(k)  = max(a,[],'omitnan') - min(a,[],'omitnan');
        mindist(k) = minInterrobot(rd);
    end

    f443 = figure('Name','Gambar 4.43','Color','w','Position',[60 60 1150 850]);
    tl2 = tiledlayout(f443,2,2,'TileSpacing','compact','Padding','compact');

    % (a) selisih waktu tiba (spread)
    ax = nexttile(tl2); hold(ax,'on'); grid(ax,'on'); box(ax,'on');
    for k = 1:nC
        bar(ax,k,spread(k),0.6,'FaceColor',condCol(k,:));
        if ~isnan(spread(k)); text(ax,k,spread(k),sprintf('%.2f s',spread(k)), ...
                'HorizontalAlignment','center','VerticalAlignment','bottom'); end
    end
    set(ax,'XTick',1:nC,'XTickLabel',condShort); xlim(ax,[0.4 nC+0.6]);
    ylabel(ax,'selisih waktu tiba (s)'); title(ax,'(a) Keserempakan kedatangan');

    % (b) jarak antarrobot minimum
    ax = nexttile(tl2); hold(ax,'on'); grid(ax,'on'); box(ax,'on');
    for k = 1:nC
        bar(ax,k,mindist(k),0.6,'FaceColor',condCol(k,:));
        if ~isnan(mindist(k)); text(ax,k,mindist(k),sprintf('%.3f m',mindist(k)), ...
                'HorizontalAlignment','center','VerticalAlignment','bottom'); end
    end
    yline(ax,D_EMERGENCY,'--r',sprintf('d_emergency = %.2f m',D_EMERGENCY),'LineWidth',1.3);
    set(ax,'XTick',1:nC,'XTickLabel',condShort); xlim(ax,[0.4 nC+0.6]);
    ylabel(ax,'jarak antarrobot minimum (m)'); title(ax,'(b) Keselamatan spasial');

    % (c) waktu tiba per robot (grouped)
    ax = nexttile(tl2); hold(ax,'on'); grid(ax,'on'); box(ax,'on');
    hb = bar(ax, arr);   % nC grup (kondisi) x nRobot seri
    for i = 1:numel(hb)
        if i <= numel(robots); set(hb(i),'FaceColor',colOf(robots{i}),'DisplayName',robots{i}); end
    end
    set(ax,'XTick',1:nC,'XTickLabel',condShort);
    ylabel(ax,'waktu tiba (s)'); title(ax,'(c) Waktu tiba per robot');
    legend(ax,'Location','best','FontSize',8);

    % (d) konvergensi konsensus (max |p_i - p_bar| vs waktu)
    ax = nexttile(tl2); hold(ax,'on'); grid(ax,'on'); box(ax,'on');
    for k = 1:nC
        rd = condRuns{k}; if isempty(rd); continue; end
        [Tg,dev] = devSeries(rd, robots);
        if ~isempty(Tg)
            plot(ax,Tg,dev,'-','Color',condCol(k,:),'LineWidth',1.6,'DisplayName',condLbl{k});
        end
    end
    yline(ax,CONV_THRESH,'--k',sprintf('ambang konvergen %.2f',CONV_THRESH),'HandleVisibility','off');
    xlabel(ax,'waktu (s)'); ylabel(ax,'max |p_i - p_bar|');
    title(ax,'(d) Konvergensi konsensus'); legend(ax,'Location','best','FontSize',8);

    title(tl2,'Perbandingan skenario split antar-kondisi','FontWeight','bold');
    savePNG(f443, fullfile(outDir,'gambar_4_43_perbandingan_split.png'));

    fprintf('Selesai. Semua gambar tersimpan di: %s\n', outDir);
end

%% ============================ FUNGSI BANTU ============================
function rd = findRun(root, key)
%FINDRUN  Cari folder run pertama yang namanya memuat KEY di dalam ROOT.
    rd = '';
    d = dir(fullfile(root, ['*' key '*']));
    d = d([d.isdir]);
    if isempty(d)
        d = dir(fullfile(root, '**', ['*' key '*']));   % fallback rekursif
        d = d([d.isdir]);
    end
    if ~isempty(d)
        rd = fullfile(d(1).folder, d(1).name);
        fprintf('  [%s] -> %s\n', key, d(1).name);
    else
        warning('Folder run untuk "%s" tidak ditemukan di %s', key, root);
    end
end

function M = loadMapAuto(root, runDirs)
%LOADMAPAUTO  Cari berkas peta .pgm (opsional) di root / root/map / run / cwd
%  (termasuk subfolder 'map' & pencarian rekursif) lalu muat.
    M = []; cand = {};
    globs = { fullfile(root,'*.pgm'), ...
              fullfile(root,'map','*.pgm'), ...
              fullfile(root,'**','*.pgm'), ...
              '*.pgm', fullfile('map','*.pgm'), fullfile('**','*.pgm') };
    for k=1:numel(runDirs)
        if isempty(runDirs{k}); continue; end
        globs{end+1} = fullfile(runDirs{k},'*.pgm'); %#ok<AGROW>
        globs{end+1} = fullfile(runDirs{k},'map','*.pgm'); %#ok<AGROW>
    end
    for g=1:numel(globs)
        pgm = dir(globs{g});
        for i=1:numel(pgm)
            if pgm(i).isdir; continue; end
            cand{end+1} = fullfile(pgm(i).folder,pgm(i).name); %#ok<AGROW>
        end
    end
    if isempty(cand)
        fprintf('Peta .pgm tidak ditemukan (lanjut tanpa latar peta).\n'); return;
    end
    % Utamakan peta yang punya berkas .yaml pendamping (info resolusi & origin).
    pick = cand{1};
    for i=1:numel(cand)
        [p,n,~] = fileparts(cand{i});
        if exist(fullfile(p,[n '.yaml']),'file')==2; pick = cand{i}; break; end
    end
    [p,n,~] = fileparts(pick); yamlPath = fullfile(p,[n '.yaml']);
    fprintf('  peta -> %s\n', pick);
    M = loadMap(pick, yamlPath, 0.05, [-3.16 -2.82]);
end

function a = arrivalByRobot(runDir, robots)
%ARRIVALBYROBOT  Waktu tiba (s) per robot dari goal_result.csv.
    a = nan(1,numel(robots));
    g = tryRead(fullfile(runDir,'goal_result.csv'));
    if isempty(g) || ~ismember('arrival_time_from_start_s',g.Properties.VariableNames); return; end
    names = string(g.robot); val = toNum(g.arrival_time_from_start_s);
    for i = 1:numel(robots)
        idx = find(names==robots{i},1);
        if ~isempty(idx); a(i) = val(idx); end
    end
end

function dmin = minInterrobot(runDir)
%MININTERROBOT  Jarak antar-robot minimum (m) sepanjang misi.
    dmin = nan;
    ir = tryRead(fullfile(runDir,'interrobot_log.csv'));
    if isempty(ir); return; end
    if ismember('min_dist',ir.Properties.VariableNames)
        dmin = min(toNum(ir.min_dist));
    elseif all(ismember({'dist_r1_r2','dist_r1_r3','dist_r2_r3'},ir.Properties.VariableNames))
        dmin = min([toNum(ir.dist_r1_r2); toNum(ir.dist_r1_r3); toNum(ir.dist_r2_r3)]);
    end
end

function [Tg, dev] = devSeries(runDir, robots)
%DEVSERIES  Deret deviasi progres antar-robot max|p_i - p_bar| vs waktu
%           (progres geometrik path_progress dari crosstrack_log, robust).
    Tg = []; dev = [];
    ct = tryRead(fullfile(runDir,'crosstrack_log.csv'));
    if isempty(ct) || ~ismember('path_progress',ct.Properties.VariableNames); return; end
    t0 = computeT0(runDir,{'crosstrack_log.csv'});
    tmax = 0; series = cell(1,numel(robots));
    for i = 1:numel(robots)
        m = strcmp(string(ct.robot), robots{i});
        if any(m)
            tt = toNum(ct.timestamp_s(m)) - t0; pp = toNum(ct.path_progress(m));
            [tt,si] = sort(tt); pp = pp(si);
            series{i} = [tt pp]; tmax = max(tmax, max(tt));
        end
    end
    if tmax <= 0; return; end
    Tg = linspace(0,tmax,300)'; Pmat = nan(numel(Tg),numel(robots));
    for i = 1:numel(robots)
        s = series{i};
        if ~isempty(s) && size(s,1) > 1
            Pmat(:,i) = interp1(s(:,1), s(:,2), Tg, 'linear', NaN);
        end
    end
    dev = max(abs(Pmat - mean(Pmat,2,'omitnan')), [], 2);
end

function plotMaxDev(ax, runDir, convThresh)
%PLOTMAXDEV  Deviasi progres maksimum max_i|p_i - p_bar| vs waktu (consensus_log).
    if isempty(runDir)
        text(ax,0.5,0.5,'(folder run tidak ditemukan)','Units','normalized', ...
             'HorizontalAlignment','center','Color',[0.6 0 0]); return;
    end
    con = tryRead(fullfile(runDir,'consensus_log.csv'));
    if isempty(con) || ~ismember('max_deviation',con.Properties.VariableNames)
        text(ax,0.5,0.5,'(data consensus_log tidak tersedia)','Units','normalized', ...
             'HorizontalAlignment','center','Color',[0.6 0 0]); return;
    end
    t = toNum(con.timestamp_s); md = toNum(con.max_deviation);
    plot(ax, t, md, '-','Color',[0.2 0.2 0.2],'LineWidth',1.5, ...
         'DisplayName','deviasi progres maks |p_i - p_bar|');
    yline(ax, convThresh, '--', sprintf('ambang konvergen %.2f',convThresh), ...
          'Color',[0.15 0.5 0.15],'LineWidth',1.2,'HandleVisibility','off');
    shadeFault(ax, faultWindow(runDir));
    xlabel(ax,'waktu (s)'); ylabel(ax,'deviasi progres (ternormalisasi)');
    legend(ax,'Location','northeast','FontSize',8);
end

function plotProgress(ax, runDir, robots)
%PLOTPROGRESS  Progres lintasan p_i(t) tiap robot + rata-rata p_bar (consensus_log).
    if isempty(runDir)
        text(ax,0.5,0.5,'(folder run tidak ditemukan)','Units','normalized', ...
             'HorizontalAlignment','center','Color',[0.6 0 0]); return;
    end
    con = tryRead(fullfile(runDir,'consensus_log.csv'));
    need = {'p_robot1','p_robot2','p_robot3','p_bar'};
    if isempty(con) || ~all(ismember(need,con.Properties.VariableNames))
        text(ax,0.5,0.5,'(data progres p_i tidak tersedia)','Units','normalized', ...
             'HorizontalAlignment','center','Color',[0.6 0 0]); return;
    end
    t = toNum(con.timestamp_s);
    for i = 1:numel(robots)
        r = robots{i};
        plot(ax, t, toNum(con.(['p_' r])), '-','Color',colOf(r),'LineWidth',1.3,'DisplayName',r);
    end
    plot(ax, t, toNum(con.p_bar), ':','Color','k','LineWidth',1.6,'DisplayName','rata-rata (p_bar)');
    shadeFault(ax, faultWindow(runDir));
    xlabel(ax,'waktu (s)'); ylabel(ax,'progres lintasan p_i');
    legend(ax,'Location','southeast','FontSize',8);
end

function plotMinDist(ax, runDir, dEmergency)
%PLOTMINDIST  Jarak antarrobot minimum vs waktu + ambang keselamatan (interrobot_log).
    if isempty(runDir)
        text(ax,0.5,0.5,'(folder run tidak ditemukan)','Units','normalized', ...
             'HorizontalAlignment','center','Color',[0.6 0 0]); return;
    end
    ir = tryRead(fullfile(runDir,'interrobot_log.csv'));
    if isempty(ir) || ~ismember('min_dist',ir.Properties.VariableNames)
        text(ax,0.5,0.5,'(data interrobot_log tidak tersedia)','Units','normalized', ...
             'HorizontalAlignment','center','Color',[0.6 0 0]); return;
    end
    t = toNum(ir.timestamp_s); mn = toNum(ir.min_dist);
    plot(ax, t, mn, '-','Color',[0.12 0.47 0.71],'LineWidth',1.4, ...
         'DisplayName','jarak antarrobot minimum');
    yline(ax, dEmergency, '--r', sprintf('d_darurat = %.2f m',dEmergency), ...
          'LineWidth',1.2,'HandleVisibility','off');
    shadeFault(ax, faultWindow(runDir));
    xlabel(ax,'waktu (s)'); ylabel(ax,'jarak (m)');
    legend(ax,'Location','best','FontSize',8);
end

function fw = faultWindow(runDir)
%FAULTWINDOW  [t_start t_end] jendela fault dari fault_event_log.csv (kosong bila tak ada).
    fw = [];
    fe = tryRead(fullfile(runDir,'fault_event_log.csv'));
    if isempty(fe) || ~ismember('event_type',fe.Properties.VariableNames); return; end
    ev = string(fe.event_type); st = []; en = [];
    if ismember('actual_start_s',fe.Properties.VariableNames)
        st = toNum(fe.actual_start_s(ev=="START")); st = st(~isnan(st));
    end
    if ismember('actual_end_s',fe.Properties.VariableNames)
        en = toNum(fe.actual_end_s(ev=="END")); en = en(~isnan(en));
    end
    if isempty(st); return; end
    t0 = min(st);
    if ~isempty(en); t1 = max(en); else; t1 = t0 + 2; end
    fw = [t0 t1];
end

function shadeFault(ax, fw)
%SHADEFAULT  Arsir rentang waktu fault (dikirim ke belakang kurva).
    if isempty(fw); return; end
    yl = ylim(ax); xl = xlim(ax);
    hp = patch(ax, [fw(1) fw(2) fw(2) fw(1)], [yl(1) yl(1) yl(2) yl(2)], ...
               [1 0.6 0], 'FaceAlpha',0.20, 'EdgeColor','none', 'DisplayName','jendela fault');
    try; uistack(hp,'bottom'); catch; end
    ylim(ax,yl); xlim(ax,xl);
end

function drawBox(ax, x, v, c, w)
%DRAWBOX  Boxplot manual (median, kuartil, whisker 1.5*IQR) tanpa toolbox statistik.
    v = v(~isnan(v)); if isempty(v); return; end
    q1 = pctl(v,25); q2 = pctl(v,50); q3 = pctl(v,75);
    iqr = q3 - q1; lo = q1 - 1.5*iqr; hi = q3 + 1.5*iqr;
    wl = min(v(v>=lo)); wh = max(v(v<=hi));
    if isempty(wl); wl = min(v); end
    if isempty(wh); wh = max(v); end
    patch(ax, [x-w x+w x+w x-w], [q1 q1 q3 q3], c, ...
          'FaceAlpha',0.5, 'EdgeColor',c*0.6, 'LineWidth',1.0, 'HandleVisibility','off');
    plot(ax, [x-w x+w], [q2 q2], '-', 'Color',c*0.4, 'LineWidth',1.8, 'HandleVisibility','off');
    plot(ax, [x x], [q1 wl], '-', 'Color',c*0.6, 'HandleVisibility','off');
    plot(ax, [x x], [q3 wh], '-', 'Color',c*0.6, 'HandleVisibility','off');
    plot(ax, [x-w/2 x+w/2], [wl wl], '-', 'Color',c*0.6, 'HandleVisibility','off');
    plot(ax, [x-w/2 x+w/2], [wh wh], '-', 'Color',c*0.6, 'HandleVisibility','off');
end

function y = pctl(v, p)
%PCTL  Persentil interpolasi linear (setara numpy 'linear'), tanpa toolbox.
    v = sort(v(~isnan(v))); n = numel(v);
    if n==0; y = NaN; return; end
    if n==1; y = v(1); return; end
    idx = (p/100)*(n-1) + 1;
    lo = floor(idx); hi = ceil(idx); frac = idx - lo;
    y = v(lo) + (v(hi)-v(lo))*frac;
end

function T = tryRead(fp)
    T = [];
    if exist(fp,'file')~=2; return; end
    try
        T = readtable(fp,'VariableNamingRule','preserve');
        T.Properties.VariableNames = matlab.lang.makeValidName(T.Properties.VariableNames);
    catch ME
        warning('Gagal baca %s (%s)', fp, ME.message); T=[];
    end
end

function v = toNum(x)
    if isnumeric(x); v=double(x); return; end
    v = str2double(string(x));
end

function t0 = computeT0(runDir, files)
    t0 = inf;
    for i = 1:numel(files)
        T = tryRead(fullfile(runDir,files{i}));
        if ~isempty(T) && ismember('timestamp_s',T.Properties.VariableNames)
            v = toNum(T.timestamp_s); v = v(~isnan(v));
            if ~isempty(v); t0 = min(t0, min(v)); end
        end
    end
    if ~isfinite(t0); t0 = 0; end
end

function c = colOf(r)
    switch r
        case 'robot1'; c=[0.12 0.47 0.71];
        case 'robot2'; c=[0.84 0.15 0.16];
        case 'robot3'; c=[0.17 0.63 0.17];
        otherwise;     c=[0 0 0];
    end
end

function drawWaypoints(S, r, c)
    if ~isfield(S,r); return; end
    wp = S.(r).wp; n = size(wp,1);
    for j = 1:n
        if j < n
            plot(wp(j,1),wp(j,2),'s','MarkerSize',9,'MarkerEdgeColor',c, ...
                'MarkerFaceColor','none','LineWidth',1.4,'HandleVisibility','off');
            text(wp(j,1),wp(j,2),sprintf('  W%d',j),'Color',c,'FontSize',7,'HandleVisibility','off');
        else
            plot(wp(j,1),wp(j,2),'p','MarkerSize',16,'MarkerFaceColor',c, ...
                'MarkerEdgeColor','k','LineWidth',1.0,'HandleVisibility','off');
        end
    end
end

function S = scenarioWaypoints(name)
% Waypoint (x,y) tiap robot dari scenarios.yaml; baris TERAKHIR = goal.
    S = struct();
    switch lower(name)
        case 'split'
            S.robot1.wp=[6.413 3.350];
            S.robot2.wp=[6.430 2.95; 6.430 -0.270];
            S.robot3.wp=[2.940 2.55; 2.940 -0.250];
        case 'crossing'
            S.robot1.wp=[0.80 2.52; 2.35 2.60; 2.72 2.52; 2.94 2.05; 2.94 -0.249];
            S.robot2.wp=[3.16 2.00; 3.45 2.45; 3.85 2.62; 6.413 2.897];
            S.robot3.wp=[3.95 2.85; 3.38 2.45; 2.55 2.45; 0.80 2.45; -0.573 2.741];
        case 'convoy'
            S.robot1.wp=[7.03 3.35; 7.03 0.20; 7.03 -0.27];
            S.robot2.wp=[6.43 2.90; 6.43 0.20; 6.43 -0.27];
            S.robot3.wp=[5.83 2.55; 5.83 0.20; 5.83 -0.27];
        case 'merge'
            S.robot1.wp=[-0.559552 3.19564; 3.00784 2.93583];
            S.robot2.wp=[2.70784 -0.460505; 3.00784 2.93583];
            S.robot3.wp=[6.3485 2.67602; 3.00784 2.93583];
        otherwise
            S = struct();
    end
end

function M = loadMap(pgmPath, yamlPath, resFallback, originFallback)
    M = [];
    if isempty(pgmPath) || exist(pgmPath,'file')~=2
        if ~isempty(pgmPath); warning('Peta .pgm tak ditemukan: %s', pgmPath); end
        return;
    end
    try; img=imread(pgmPath); catch ME; warning('Gagal baca pgm (%s)',ME.message); return; end
    res = resFallback; origin = originFallback;
    if ~isempty(yamlPath) && exist(yamlPath,'file')==2
        txt = fileread(yamlPath);
        tk = regexp(txt,'resolution:\s*([-\d.]+)','tokens','once'); if ~isempty(tk); res=str2double(tk{1}); end
        tk = regexp(txt,'origin:\s*\[([^\]]+)\]','tokens','once');
        if ~isempty(tk); v=str2double(strsplit(tk{1},',')); if numel(v)>=2; origin=v(1:2); end; end
    end
    [H,W] = size(img);
    M.img=img; M.res=res; M.origin=origin;
    M.xdata=[origin(1) origin(1)+W*res];
    M.ydata=[origin(2)+H*res origin(2)];
end

function drawMap(M)
    if isempty(M); return; end
    imagesc(M.xdata, M.ydata, M.img); set(gca,'YDir','normal');
    colormap(gca, gray); hold(gca,'on');
end

function savePNG(fig, fp)
%SAVEPNG  Simpan figure ke PNG (exportgraphics bila ada, jika tidak pakai print).
    try
        exportgraphics(fig, fp, 'Resolution', 200);
    catch
        try
            print(fig, fp, '-dpng', '-r200');
        catch ME
            warning('Gagal simpan %s (%s)', fp, ME.message);
        end
    end
    fprintf('  tersimpan: %s\n', fp);
end
