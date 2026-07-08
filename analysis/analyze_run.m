function analyze_run(runDir, outDir)
%% ANALYZE_RUN  Visualisasi poin 1-8 evaluasi metrik multi-robot (haqqi_ta)
%
%  Menghasilkan:
%    Lintasan + PETA + WAYPOINT (global path vs aktual)        -> 01_lintasan.png
%    Video animasi lintasan (real-time, durasi = asli)         -> 01b_lintasan_anim.mp4
%    Video pembentukan global path (Dijkstra)                  -> 01c_global_path_anim.mp4
%    Waktu tiba                                                -> 02a_waktu_tiba.png
%    Presisi posisi akhir                                      -> 02b_presisi.png
%    Cross-track error vs waktu (+ pita fault)                 -> 03_crosstrack.png
%    Jarak antar-robot vs d_emergency                          -> 04_jarak_antar_robot.png
%    Progress lintasan p(t) + TITIK WAYPOINT                   -> 05a_progress.png
%    Konvergensi progress antar-robot                          -> 05b_konvergensi.png
%    Kecepatan: batas konsensus (vmax_cons)                    -> 06a_vmax_konsensus.png
%    Kecepatan aktual                                          -> 06b_kecepatan_aktual.png
%    Interval priority-stop                                    -> 06c_priority_stop.png
%    Kecepatan GABUNGAN ketiga robot (1 grafik)                -> 06d_kecepatan_gabungan.png
%    Local path (DWA) + PETA                                   -> 07_local_path.png
%    Video proses DWA menemukan local path (real-time)         -> 07b_dwa_local_path_anim.mp4
%    Waypoint error (jarak terdekat lintasan ke waypoint)      -> 08_waypoint_error.png
%
%  PEMAKAIAN:
%    analyze_run
%    analyze_run('path/ke/run')
%    analyze_run('path/ke/run','folder_out')

    %% ===================== KONFIGURASI =====================
    if nargin < 1 || isempty(runDir)
        runDir = '4.convoy_cons_fault_01_20260628_105913';   % <-- ganti ke folder run kamu
    end
    if nargin < 2 || isempty(outDir)
        outDir = fullfile(runDir, 'figs_matlab');
    end
    if ~exist(outDir,'dir'); mkdir(outDir); end

    SCENARIO     = 'convoy';     % 'split' | 'crossing' | 'convoy' | 'merge'
    MAKE_VIDEO        = true;    % video animasi lintasan (real-time, durasi = durasi asli)
    MAKE_DWA_VIDEO    = false;    % video proses DWA menemukan local path (real-time)
    MAKE_GLOBAL_VIDEO = false;    % video pembentukan global path (Dijkstra)
    VIDEO_FPS         = 5;      % frame/detik untuk semua video real-time
    PROGRESS_SOURCE = 'crosstrack';  % 'crosstrack' = progres geometrik (robust, disarankan) | 'consensus' = p mentah dari consensus_log

    MAP_PGM        = 'yahboom_map_lss_carto.pgm';
    MAP_YAML       = 'yahboom_map_lss_carto.yaml';
    MAP_RESOLUTION = 0.05;
    MAP_ORIGIN     = [-3.16 -2.82];

    D_EMERGENCY = 0.30;
    GOAL_RADIUS = 0.15;

    % Teks polos (tanpa TeX) supaya underscore/karakter khusus aman & tak error
    set(groot,'defaultTextInterpreter','none');
    set(groot,'defaultLegendInterpreter','none');
    set(groot,'defaultAxesTickLabelInterpreter','none');

    robots = {'robot1','robot2','robot3'};
    fprintf('Run: %s | skenario: %s\n', runDir, SCENARIO);

    %% ===================== MUAT DATA =====================
    pose = tryRead(fullfile(runDir,'pose_log.csv'));
    path = tryRead(fullfile(runDir,'path_log.csv'));
    cons = tryRead(fullfile(runDir,'consensus_log.csv'));
    ct   = tryRead(fullfile(runDir,'crosstrack_log.csv'));
    vel  = tryRead(fullfile(runDir,'velocity_log.csv'));
    ir   = tryRead(fullfile(runDir,'interrobot_log.csv'));
    goal = tryRead(fullfile(runDir,'goal_result.csv'));

    t0 = computeT0(runDir, {'velocity_log.csv','consensus_log.csv', ...
        'interrobot_log.csv','crosstrack_log.csv','pose_log.csv'});
    fprintf('t0 = %.3f s (offset waktu dinormalkan ke 0)\n', t0);

    P   = faultIntervals(runDir, t0, vel);
    S   = scenarioWaypoints(SCENARIO);
    M   = loadMap(MAP_PGM, MAP_YAML, MAP_RESOLUTION, MAP_ORIGIN);
    if isempty(M); fprintf('Peta tidak dimuat (lanjut tanpa latar peta).\n'); end
    if isempty(P); fprintf('Tidak ada pulsa fault terbaca.\n'); else; fprintf('%d pulsa fault terbaca.\n', size(P,1)); end

    %% ============ FIG 1: LINTASAN + PETA + WAYPOINT ============
    f1 = figure('Name','1. Lintasan','Color','w','Position',[60 60 820 680]); ax=gca; hold(ax,'on');
    drawMap(M);
    h = []; lbl = {};
    for i = 1:numel(robots)
        r = robots{i}; c = colOf(r);
        if ~isempty(path)
            m = strcmp(string(path.robot), r);
            if any(m)
                gp = sortrows(path(m,:),'point_index');
                hh = plot(gp.x, gp.y, '--', 'Color', c, 'LineWidth',1.5);
                h(end+1)=hh; lbl{end+1}=[r ' global path']; %#ok<AGROW>
            end
        end
        if ~isempty(pose)
            m = strcmp(string(pose.robot), r);
            if any(m)
                pp = pose(m,:);
                hh = plot(pp.x, pp.y, '-', 'Color', c, 'LineWidth',2.0);
                h(end+1)=hh; lbl{end+1}=[r ' aktual']; %#ok<AGROW>
                plot(pp.x(1), pp.y(1), 'o','MarkerSize',9,'MarkerEdgeColor',c,'MarkerFaceColor','w','LineWidth',1.6,'HandleVisibility','off');
            end
        end
        drawWaypoints(S, r, c);
    end
    axis(ax,'equal'); grid(ax,'on'); box(ax,'on');
    xlabel('x (m)'); ylabel('y (m)');
    title(sprintf('Lintasan (%s): global path (putus-putus) vs aktual (solid); o=start, kotak=W, bintang=goal', SCENARIO));
    % Keterangan bentuk marker (start / waypoint / goal)
    hs=plot(nan,nan,'o','MarkerSize',9,'MarkerEdgeColor','k','MarkerFaceColor','w','LineWidth',1.4);
    hw=plot(nan,nan,'s','MarkerSize',9,'MarkerEdgeColor','k','MarkerFaceColor','none','LineWidth',1.4);
    hg=plot(nan,nan,'p','MarkerSize',14,'MarkerEdgeColor','k','MarkerFaceColor','k');
    h(end+1)=hs; lbl{end+1}='lingkaran (o) = start';
    h(end+1)=hw; lbl{end+1}='kotak = waypoint';
    h(end+1)=hg; lbl{end+1}='bintang = goal';
    if ~isempty(h); legend(h, lbl, 'Location','eastoutside','FontSize',8); end
    savePNG(f1, fullfile(outDir,'01_lintasan.png'));

    %% ============ FIG 1b: VIDEO ANIMASI ============
    if MAKE_VIDEO && ~isempty(pose)
        makeTrajVideo(fullfile(outDir,'01b_lintasan_anim.mp4'), S, M, robots, pose, t0, P, SCENARIO, VIDEO_FPS);
    end

    %% ============ FIG 2a: WAKTU TIBA ============
    f2a = figure('Name','Waktu tiba','Color','w','Position',[80 80 560 440]); hold on; grid on; box on;
    if ~isempty(goal)
        names = string(goal.robot);
        arr   = toNum(goal.arrival_time_from_start_s);
        for k=1:numel(arr)
            bar(k, max(arr(k),0),'FaceColor',colOf(char(names(k))));
            if isnan(arr(k)); text(k,0,'DNF','HorizontalAlignment','center','VerticalAlignment','bottom','FontWeight','bold','Color','r');
            else; text(k,arr(k),sprintf('%.1f s',arr(k)),'HorizontalAlignment','center','VerticalAlignment','bottom'); end
        end
        set(gca,'XTick',1:numel(names),'XTickLabel',names); ylabel('waktu tiba (s)');
        title(sprintf('Waktu tiba (spread = %.2f s)', max(arr)-min(arr)));
    end
    savePNG(f2a, fullfile(outDir,'02a_waktu_tiba.png'));

    %% ============ FIG 2b: PRESISI POSISI AKHIR ============
    f2b = figure('Name','Presisi posisi akhir','Color','w','Position',[110 90 560 440]); hold on; grid on; box on;
    if ~isempty(goal)
        names = string(goal.robot);
        prec  = toNum(goal.goal_precision_m);
        for k=1:numel(prec)
            bar(k, prec(k),'FaceColor',colOf(char(names(k))));
            text(k,prec(k),sprintf('%.3f m',prec(k)),'HorizontalAlignment','center','VerticalAlignment','bottom');
        end
        yline(GOAL_RADIUS,'--k',sprintf('radius goal %.2f m',GOAL_RADIUS));
        set(gca,'XTick',1:numel(names),'XTickLabel',names); ylabel('presisi akhir (m)'); title('Presisi posisi akhir');
    end
    savePNG(f2b, fullfile(outDir,'02b_presisi.png'));

    %% ============ FIG 3: CROSS-TRACK ============
    f3 = figure('Name','3. Cross-track','Color','w','Position',[100 100 940 470]); hold on; grid on; box on;
    if ~isempty(ct)
        for i=1:numel(robots)
            r=robots{i}; m=strcmp(string(ct.robot),r);
            if any(m); plot(toNum(ct.timestamp_s(m))-t0, ct.crosstrack_error_m(m),'-','Color',colOf(r),'LineWidth',1.5,'DisplayName',r); end
        end
    end
    legend('Location','best'); shadeFaults(P);
    xlabel('waktu (s)'); ylabel('cross-track error (m)'); title('Cross-track error terhadap waktu');
    savePNG(f3, fullfile(outDir,'03_crosstrack.png'));

    %% ============ FIG 4: JARAK ANTAR-ROBOT ============
    f4 = figure('Name','4. Jarak antar-robot','Color','w','Position',[120 120 940 470]); hold on; grid on; box on;
    if ~isempty(ir)
        t = toNum(ir.timestamp_s)-t0;
        plot(t, ir.dist_r1_r2,'-','Color',[0.58 0.40 0.74],'LineWidth',1.4,'DisplayName','r1-r2');
        plot(t, ir.dist_r1_r3,'-','Color',[1.00 0.50 0.05],'LineWidth',1.4,'DisplayName','r1-r3');
        plot(t, ir.dist_r2_r3,'-','Color',[0.09 0.75 0.81],'LineWidth',1.4,'DisplayName','r2-r3');
        yline(D_EMERGENCY,'--r',sprintf('d_emergency = %.2f m',D_EMERGENCY),'LineWidth',1.3,'HandleVisibility','off');
        [mn,idx]=min(ir.min_dist);
        plot(t(idx),mn,'rv','MarkerFaceColor','r','MarkerSize',10,'DisplayName',sprintf('min %.3f m @ %.1fs',mn,t(idx)));
        legend('Location','best');
    end
    shadeFaults(P);
    xlabel('waktu (s)'); ylabel('jarak antar-robot (m)'); title('Jarak antar-robot (closest approach)');
    savePNG(f4, fullfile(outDir,'04_jarak_antar_robot.png'));

    %% ============ FIG 5: PROGRESS + TITIK WAYPOINT & GOAL ============
    % Catatan: 'consensus' p_robotX bisa macet/meloncat bila estimator node
    % tidak update (mis. robot1 stuck di 0 lalu loncat). 'crosstrack'
    % (path_progress geometrik) lebih robust dan mencerminkan gerak nyata.
    Tg=[]; Pmat=[]; tmaxp=0;
    for i=1:numel(robots)
        [tt,~]=progressSeries(PROGRESS_SOURCE,robots{i},cons,ct,t0);
        if ~isempty(tt); tmaxp=max(tmaxp,max(tt)); end
    end
    if tmaxp>0
        Tg=linspace(0,tmaxp,300)'; Pmat=nan(numel(Tg),numel(robots));
        for i=1:numel(robots)
            [tt,pp]=progressSeries(PROGRESS_SOURCE,robots{i},cons,ct,t0);
            if numel(tt)>1; Pmat(:,i)=interp1(tt,pp,Tg,'linear',NaN); end
        end
    end
    f5a = figure('Name','Progress lintasan','Color','w','Position',[140 140 1000 460]); hold on; grid on; box on; legH=[]; legL={};
    for i=1:numel(robots)
        r=robots{i}; c=colOf(r);
        [tt,pp]=progressSeries(PROGRESS_SOURCE,r,cons,ct,t0);
        if isempty(tt); continue; end
        hh=plot(tt,pp,'-','Color',c,'LineWidth',1.7); legH(end+1)=hh; legL{end+1}=['progress ' r]; %#ok<AGROW>
        if ~isempty(pose) && isfield(S,r)
            po=pose(strcmp(string(pose.robot),r),:);
            [tp,~]=wpPassInfo(po,t0,S.(r).wp); nwp=numel(tp);
            for j=1:nwp
                pv=interp1(tt,pp,tp(j),'linear',NaN);
                if j<nwp
                    plot(tp(j),pv,'o','MarkerFaceColor',c,'MarkerEdgeColor','k','MarkerSize',7,'HandleVisibility','off');
                else
                    plot(tp(j),pv,'p','MarkerFaceColor',c,'MarkerEdgeColor','k','MarkerSize',15,'HandleVisibility','off');
                end
            end
        end
    end
    if strcmpi(PROGRESS_SOURCE,'consensus') && ~isempty(cons) && ismember('p_bar',cons.Properties.VariableNames)
        hb=plot(toNum(cons.timestamp_s)-t0, cons.p_bar,'--k','LineWidth',1.3); legH(end+1)=hb; legL{end+1}='p_bar (rata-rata)';
    elseif ~isempty(Pmat)
        hb=plot(Tg,mean(Pmat,2,'omitnan'),'--k','LineWidth',1.3); legH(end+1)=hb; legL{end+1}='p_bar (rata-rata)';
    end
    ho=plot(nan,nan,'o','MarkerFaceColor',[.45 .45 .45],'MarkerEdgeColor','k','MarkerSize',7); legH(end+1)=ho; legL{end+1}='o = lewat waypoint';
    hgg=plot(nan,nan,'p','MarkerFaceColor',[.45 .45 .45],'MarkerEdgeColor','k','MarkerSize',13); legH(end+1)=hgg; legL{end+1}='bintang = capai goal';
    ylim([0 1.1]); ylabel('progress p (0..1)'); legend(legH,legL,'Location','eastoutside','FontSize',8);
    title(sprintf('Progress lintasan p(t)  [sumber: %s]  -  o = waypoint, bintang = goal', PROGRESS_SOURCE)); shadeFaults(P);
    savePNG(f5a, fullfile(outDir,'05a_progress.png'));

    f5b = figure('Name','Konvergensi progress','Color','w','Position',[160 160 1000 420]); hold on; grid on; box on;
    if strcmpi(PROGRESS_SOURCE,'consensus') && ~isempty(cons) && ismember('max_deviation',cons.Properties.VariableNames)
        plot(toNum(cons.timestamp_s)-t0, cons.max_deviation,'-','Color',[0.3 0.3 0.3],'LineWidth',1.5,'DisplayName','max deviation');
    elseif ~isempty(Pmat)
        dev=max(abs(Pmat-mean(Pmat,2,'omitnan')),[],2);
        plot(Tg,dev,'-','Color',[0.3 0.3 0.3],'LineWidth',1.5,'DisplayName','max |p_i - p_bar|');
    end
    xlabel('waktu (s)'); ylabel('max |p_i - p_bar|'); legend('Location','best');
    title('Konvergensi progress antar-robot (merah = fault)'); shadeFaults(P);
    savePNG(f5b, fullfile(outDir,'05b_konvergensi.png'));

    %% ============ FIG 6a: VMAX KONSENSUS ============
    f6a = figure('Name','Kecepatan: batas konsensus','Color','w','Position',[160 160 960 420]); hold on; grid on; box on;
    if ~isempty(vel)
        for i=1:numel(robots)
            r=robots{i}; m=strcmp(string(vel.robot),r);
            if any(m)&&ismember('vmax_consensus',vel.Properties.VariableNames)
                plot(toNum(vel.timestamp_s(m))-t0, toNum(vel.vmax_consensus(m)),'-','Color',colOf(r),'LineWidth',1.4,'DisplayName',r);
            end
        end
        xlabel('waktu (s)'); ylabel('vmax_consensus (m/s)'); legend('Location','best'); title('Kecepatan: batas konsensus'); shadeFaults(P);
    end
    savePNG(f6a, fullfile(outDir,'06a_vmax_konsensus.png'));

    %% ============ FIG 6b: KECEPATAN AKTUAL ============
    f6b = figure('Name','Kecepatan aktual','Color','w','Position',[180 180 960 420]); hold on; grid on; box on;
    if ~isempty(vel)
        for i=1:numel(robots)
            r=robots{i}; m=strcmp(string(vel.robot),r);
            if any(m); plot(toNum(vel.timestamp_s(m))-t0, toNum(vel.speed(m)),'-','Color',colOf(r),'LineWidth',1.4,'DisplayName',r); end
        end
        xlabel('waktu (s)'); ylabel('kecepatan aktual (m/s)'); legend('Location','best'); title('Kecepatan aktual'); shadeFaults(P);
    end
    savePNG(f6b, fullfile(outDir,'06b_kecepatan_aktual.png'));

    %% ============ FIG 6c: INTERVAL PRIORITY-STOP ============
    f6c = figure('Name','Priority-stop','Color','w','Position',[200 200 960 420]); hold on; grid on; box on;
    if ~isempty(vel)
        for i=1:numel(robots)
            r=robots{i}; ints=pstopIntervals(vel,r,t0); ylev=numel(robots)-i+1;
            for j=1:size(ints,1)
                rectangle('Position',[ints(j,1) ylev-0.35 max(ints(j,2)-ints(j,1),1e-3) 0.7],'FaceColor',colOf(r),'EdgeColor','none');
            end
        end
        ylim([0 numel(robots)+1]); set(gca,'YTick',1:numel(robots),'YTickLabel',fliplr(robots));
        xlabel('waktu (s)'); ylabel('priority-stop'); title('Interval priority-stop (batang = robot ditahan)'); shadeFaults(P);
    end
    savePNG(f6c, fullfile(outDir,'06c_priority_stop.png'));

    %% ============ FIG 6d: KECEPATAN GABUNGAN (1 grafik) ============
    f6d = figure('Name','Kecepatan gabungan','Color','w','Position',[180 180 1000 560]); hold on; grid on; box on;
    if ~isempty(vel)
        vmaxsp = 0.6;
        for i=1:numel(robots)
            r=robots{i}; m=strcmp(string(vel.robot),r);
            if any(m)
                tt=toNum(vel.timestamp_s(m))-t0;
                plot(tt, toNum(vel.speed(m)),'-','Color',colOf(r),'LineWidth',1.5,'DisplayName',[r ' aktual']);
                if ismember('vmax_consensus',vel.Properties.VariableNames)
                    plot(tt, toNum(vel.vmax_consensus(m)),'--','Color',colOf(r),'LineWidth',1.1,'DisplayName',[r ' vmax_cons']);
                end
                vmaxsp=max(vmaxsp, max(toNum(vel.speed(m)))*1.1);
            end
        end
        ylim([-0.13 vmaxsp]);
        shadeFaults(P);
        bandh=[-0.03 -0.06 -0.09];
        for i=1:numel(robots)
            r=robots{i}; ints=pstopIntervals(vel,r,t0);
            for j=1:size(ints,1)
                rectangle('Position',[ints(j,1) bandh(i)-0.012 max(ints(j,2)-ints(j,1),1e-3) 0.024],'FaceColor',colOf(r),'EdgeColor','none');
            end
        end
        yline(0,'-','Color',[0.6 0.6 0.6],'HandleVisibility','off');
        text(0,-0.105,' pita priority-stop (bawah)','FontSize',8,'Color',[0.4 0.4 0.4]);
        legend('Location','northeastoutside','FontSize',8);
    end
    xlabel('waktu (s)'); ylabel('kecepatan (m/s)');
    title('Kecepatan gabungan ketiga robot - aktual (solid) + vmax_cons (putus) + fault + priority-stop');
    savePNG(f6d, fullfile(outDir,'06d_kecepatan_gabungan.png'));

    %% ============ FIG 7: LOCAL PATH (DWA) + PETA ============
    f7 = figure('Name','7. Local path','Color','w','Position',[200 200 820 680]); ax7=gca; hold(ax7,'on');
    drawMap(M);
    lp = tryRead(fullfile(runDir,'local_plan_log.csv'));
    if ~isempty(lp) && height(lp)>0 && all(ismember({'x','y'},lp.Properties.VariableNames))
        h7=[]; lbl7={};
        if ismember('robot',lp.Properties.VariableNames)
            for i=1:numel(robots)
                r=robots{i}; m=strcmp(string(lp.robot),r);
                if any(m); hh=plot(lp.x(m),lp.y(m),'.','Color',colOf(r),'MarkerSize',6); h7(end+1)=hh; lbl7{end+1}=r; end %#ok<AGROW>
            end
            if ~isempty(h7); legend(h7,lbl7,'Location','eastoutside'); end
        else
            plot(lp.x, lp.y,'.','MarkerSize',6);
        end
        axis(ax7,'equal'); grid on; box on; xlabel('x (m)'); ylabel('y (m)');
        title('Local path (DWA) + peta');
    else
        if ~isempty(M); axis(ax7,'equal'); end
        yl=ylim; xl=xlim;
        text(mean(xl),mean(yl),{'local_plan_log.csv kosong (hanya header).', ...
            'Topik /ns/local_plan belum dijembatani via UDP dari robot ke PC,', ...
            'sehingga trajektori DWA tak terekam.'},'HorizontalAlignment','center','FontSize',10,'BackgroundColor',[1 1 1]);
        title('Local path (DWA) - log kosong');
        xlabel('x (m)'); ylabel('y (m)'); box on;
    end
    savePNG(f7, fullfile(outDir,'07_local_path.png'));

    %% ============ VIDEO PROSES DWA (LOCAL PATH) ============
    if MAKE_DWA_VIDEO
        makeDWAVideo(fullfile(outDir,'07b_dwa_local_path_anim.mp4'), M, robots, lp, path, pose, t0, VIDEO_FPS, SCENARIO);
    end

    %% ============ FIG 8: WAYPOINT ERROR ============
    f8 = figure('Name','8. Waypoint error','Color','w','Position',[220 220 940 470]); hold on; grid on; box on;
    if ~isempty(pose) && ~isempty(fieldnames(S))
        maxN=0; for i=1:numel(robots); if isfield(S,robots{i}); maxN=max(maxN,size(S.(robots{i}).wp,1)); end; end
        E=nan(numel(robots),maxN);
        for i=1:numel(robots)
            r=robots{i};
            if isfield(S,r)
                po=pose(strcmp(string(pose.robot),r),:);
                [~,dmin]=wpPassInfo(po,t0,S.(r).wp);
                E(i,1:numel(dmin))=dmin(:)';
            end
        end
        hb=bar(E');
        for i=1:numel(robots); if i<=numel(hb); set(hb(i),'FaceColor',colOf(robots{i}),'DisplayName',robots{i}); end; end
        xtl = cell(1,maxN);
        for j=1:maxN; if j<maxN; xtl{j}=sprintf('W%d',j); else; xtl{j}='G'; end; end
        set(gca,'XTick',1:maxN,'XTickLabel',xtl);
        ylabel('jarak terdekat lintasan ke waypoint (m)'); legend('Location','best');
        title('Waypoint error - seberapa dekat lintasan aktual melewati tiap waypoint');
    end
    savePNG(f8, fullfile(outDir,'08_waypoint_error.png'));

    %% ============ VIDEO PEMBENTUKAN GLOBAL PATH (DIJKSTRA) ============
    if MAKE_GLOBAL_VIDEO && ~isempty(path)
        makeGlobalPathVideo(fullfile(outDir,'01c_global_path_anim.mp4'), M, robots, path, VIDEO_FPS, SCENARIO);
    end

    fprintf('Selesai. Semua figure tersimpan di: %s\n', outDir);
end

%% ============================ FUNGSI BANTU ============================
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

function b = toLogical(x)
    if islogical(x); b=x; return; end
    if isnumeric(x); b=x~=0; return; end
    s=lower(strtrim(string(x))); b=(s=="true")|(s=="1")|(s=="1.0")|(s=="yes");
end

function t0 = computeT0(runDir, files)
    t0=inf;
    for i=1:numel(files)
        T=tryRead(fullfile(runDir,files{i}));
        if ~isempty(T)&&ismember('timestamp_s',T.Properties.VariableNames)
            v=toNum(T.timestamp_s); v=v(~isnan(v));
            if ~isempty(v); t0=min(t0,min(v)); end
        end
    end
    if ~isfinite(t0); t0=0; end
end

function P = faultIntervals(runDir, t0, vel)
    P=zeros(0,2);
    T=tryRead(fullfile(runDir,'fault_event_log.csv'));
    if ~isempty(T)&&height(T)>0&&ismember('event_type',T.Properties.VariableNames)
        et=string(T.event_type); ts=toNum(T.timestamp_s);
        sIdx=find(et=="START"); eIdx=find(et=="END");
        for i=1:numel(sIdx)
            s=ts(sIdx(i))-t0;
            if i<=numel(eIdx); e=ts(eIdx(i))-t0; else; e=s+2; end
            P(end+1,:)=[s e]; %#ok<AGROW>
        end
    end
    if isempty(P)&&nargin>2&&~isempty(vel)&&ismember('fault_active',vel.Properties.VariableNames)
        fa=toLogical(vel.fault_active); t=toNum(vel.timestamp_s)-t0;
        [t,si]=sort(t); fa=fa(si); on=false; s=0;
        for i=1:numel(fa)
            if fa(i)&&~on; on=true; s=t(i); elseif ~fa(i)&&on; on=false; P(end+1,:)=[s t(i)]; end %#ok<AGROW>
        end
        if on; P(end+1,:)=[s t(end)]; end
    end
end

function shadeFaults(P)
%SHADEFAULTS  Pita merah TEBAL + garis vertikal merah + label F1/F2/F3.
    if isempty(P); return; end
    yl=ylim;
    for k=1:size(P,1)
        x0=P(k,1); x1=P(k,2);
        patch([x0 x1 x1 x0],[yl(1) yl(1) yl(2) yl(2)],[1 0.3 0.3], ...
            'FaceAlpha',0.22,'EdgeColor','none','HandleVisibility','off');
        plot([x0 x0],yl,'-r','LineWidth',1.0,'HandleVisibility','off');
        plot([x1 x1],yl,'-r','LineWidth',1.0,'HandleVisibility','off');
        text((x0+x1)/2, yl(2), sprintf(' F%d',k),'Color','r','FontWeight','bold', ...
            'HorizontalAlignment','center','VerticalAlignment','top','FontSize',9);
    end
    ylim(yl);
end

function ints = pstopIntervals(vel, r, t0)
    ints=zeros(0,2);
    m=strcmp(string(vel.robot),r); if ~any(m); return; end
    d=vel(m,:); t=toNum(d.timestamp_s)-t0; [t,si]=sort(t);
    if ~ismember('priority_stop',d.Properties.VariableNames); return; end
    ps=toLogical(d.priority_stop); ps=ps(si); on=false; s=0;
    for i=1:numel(ps)
        if ps(i)&&~on; on=true; s=t(i); elseif ~ps(i)&&on; on=false; ints(end+1,:)=[s t(i)]; end %#ok<AGROW>
    end
    if on; ints(end+1,:)=[s t(end)]; end
end

function [tt, pp] = progressSeries(src, r, cons, ct, t0)
% Ambil deret progress 0..1 untuk robot r dari sumber terpilih.
    tt=[]; pp=[];
    if strcmpi(src,'crosstrack') && ~isempty(ct) && ismember('path_progress',ct.Properties.VariableNames)
        m=strcmp(string(ct.robot),r);
        if any(m); tt=toNum(ct.timestamp_s(m))-t0; pp=toNum(ct.path_progress(m)); [tt,si]=sort(tt); pp=pp(si); end
    elseif ~isempty(cons)
        pcol=['p_' r];
        if ismember(pcol,cons.Properties.VariableNames); tt=toNum(cons.timestamp_s)-t0; pp=toNum(cons.(pcol)); [tt,si]=sort(tt); pp=pp(si); end
    end
end

function [tpass, dmin] = wpPassInfo(po, t0, wp)
    tpass=nan(size(wp,1),1); dmin=nan(size(wp,1),1);
    if isempty(po)||isempty(wp); return; end
    px=po.x; py=po.y; pt=toNum(po.timestamp_s)-t0;
    for j=1:size(wp,1)
        d=hypot(px-wp(j,1), py-wp(j,2));
        [dmin(j),mi]=min(d); tpass(j)=pt(mi);
    end
end

function drawWaypoints(S, r, c)
    if ~isfield(S,r); return; end
    wp=S.(r).wp; n=size(wp,1);
    for j=1:n
        if j<n
            plot(wp(j,1),wp(j,2),'s','MarkerSize',9,'MarkerEdgeColor',c,'MarkerFaceColor','none','LineWidth',1.4,'HandleVisibility','off');
            text(wp(j,1),wp(j,2),sprintf('  W%d',j),'Color',c,'FontSize',7,'HandleVisibility','off');
        else
            plot(wp(j,1),wp(j,2),'p','MarkerSize',16,'MarkerFaceColor',c,'MarkerEdgeColor','k','LineWidth',1.0,'HandleVisibility','off');
        end
    end
end

function c = colOf(r)
    switch r
        case 'robot1'; c=[0.12 0.47 0.71];
        case 'robot2'; c=[0.84 0.15 0.16];
        case 'robot3'; c=[0.17 0.63 0.17];
        otherwise;     c=[0 0 0];
    end
end

function M = loadMap(pgmPath, yamlPath, resFallback, originFallback)
    M=[];
    if isempty(pgmPath)||exist(pgmPath,'file')~=2
        if ~isempty(pgmPath); warning('Peta .pgm tak ditemukan: %s', pgmPath); end
        return;
    end
    try; img=imread(pgmPath); catch ME; warning('Gagal baca pgm (%s)',ME.message); return; end
    res=resFallback; origin=originFallback;
    if ~isempty(yamlPath)&&exist(yamlPath,'file')==2
        txt=fileread(yamlPath);
        tk=regexp(txt,'resolution:\s*([-\d.]+)','tokens','once'); if ~isempty(tk); res=str2double(tk{1}); end
        tk=regexp(txt,'origin:\s*\[([^\]]+)\]','tokens','once');
        if ~isempty(tk); v=str2double(strsplit(tk{1},',')); if numel(v)>=2; origin=v(1:2); end; end
    end
    [H,W]=size(img);
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
        exportgraphics(fig, fp, 'Resolution', 150);
    catch
        try
            print(fig, fp, '-dpng', '-r150');
        catch ME
            warning('Gagal simpan %s (%s)', fp, ME.message);
        end
    end
end

function makeTrajVideo(outFile, S, M, robots, pose, t0, P, scn, fps)
    if nargin<9 || isempty(fps); fps=15; end
    fprintf('Membuat video animasi lintasan (real-time)...\n');
    pt=toNum(pose.timestamp_s)-t0; tmax=max(pt);
    nF=max(round(tmax*fps),2); tg=linspace(0,tmax,nF);   % real-time: durasi video = durasi rekaman asli
    try; vw=VideoWriter(outFile,'MPEG-4');
    catch; outFile=strrep(outFile,'.mp4','.avi'); vw=VideoWriter(outFile,'Motion JPEG AVI'); end
    vw.FrameRate=fps; open(vw);
    fig=figure('Color','w','Position',[100 100 820 680],'Visible','off');
    xall=pose.x; yall=pose.y; mx=0.5;
    xl=[min(xall)-mx max(xall)+mx]; yl=[min(yall)-mx max(yall)+mx];
    if ~isempty(M); xl=[min(xl(1),M.xdata(1)) max(xl(2),M.xdata(2))]; yl=[min(yl(1),M.ydata(2)) max(yl(2),M.ydata(1))]; end
    for k=1:nF
        clf(fig); ax=axes(fig); hold(ax,'on'); %#ok<LAXES>
        drawMap(M); tc=tg(k);
        for i=1:numel(robots)
            r=robots{i}; c=colOf(r); m=strcmp(string(pose.robot),r);
            if ~any(m); continue; end
            sx=pose.x(m); sy=pose.y(m); st=pt(m);
            drawWaypoints(S,r,c);
            sel=st<=tc;
            if any(sel); plot(sx(sel),sy(sel),'-','Color',c,'LineWidth',2); plot(sx(find(sel,1,'last')),sy(find(sel,1,'last')),'o','MarkerFaceColor',c,'MarkerEdgeColor','k','MarkerSize',11); end
        end
        faultNow=false; for q=1:size(P,1); if tc>=P(q,1)&&tc<=P(q,2); faultNow=true; end; end
        ttl=sprintf('Animasi lintasan (%s) - t = %.1f s', scn, tc);
        if faultNow; ttl=[ttl '   [FAULT robot2 AKTIF]']; end
        title(ax,ttl,'Color',ternary(faultNow,[0.8 0 0],[0 0 0]));
        axis(ax,'equal'); xlim(ax,xl); ylim(ax,yl); grid(ax,'on'); box(ax,'on'); xlabel('x (m)'); ylabel('y (m)');
        writeVideo(vw, getframe(fig));
    end
    close(vw); close(fig);
    fprintf('Video tersimpan: %s\n', outFile);
end

function y = ternary(c,a,b); if c; y=a; else; y=b; end; end

function S = scenarioWaypoints(name)
% Waypoint (x,y) tiap robot dari scenarios.yaml; baris TERAKHIR = goal.
    S=struct();
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
            S=struct();
    end
end

%% ===================== VIDEO DWA LOCAL PATH =====================
function makeDWAVideo(outFile, M, robots, lp, gpath, pose, t0, fps, scn)
%MAKEDWAVIDEO  Animasi REAL-TIME proses DWA: local plan (lookahead) yang
% diperbarui tiap siklus saat robot bergerak, digambar di atas peta + global
% path referensi. Catatan: local_plan_log mencatat trajektori HASIL pilihan
% DWA per siklus (output), bukan sampling kandidat velocity/skor internal
% (tidak ada di log). Jadi video memperlihatkan local path yang terus
% direncanakan-ulang & bergeser maju mengikuti gerak robot.
    if isempty(lp) || height(lp)==0 || ~all(ismember({'x','y','timestamp_s','robot','plan_seq'}, lp.Properties.VariableNames))
        fprintf('Lewati video DWA: local_plan_log.csv kosong/format tak sesuai.\n'); return;
    end
    if nargin<8 || isempty(fps); fps=15; end
    fprintf('Membuat video DWA local path (real-time)...\n');
    lpt = toNum(lp.timestamp_s)-t0;
    tmax = max(lpt);
    nF = max(round(tmax*fps),2); tg = linspace(0,tmax,nF);   % real-time
    try; vw=VideoWriter(outFile,'MPEG-4');
    catch; outFile=strrep(outFile,'.mp4','.avi'); vw=VideoWriter(outFile,'Motion JPEG AVI'); end
    vw.FrameRate=fps; open(vw);
    fig=figure('Color','w','Position',[100 100 860 700],'Visible','off');
    % batas tampilan
    xall=lp.x; yall=lp.y; mx=0.5;
    xl=[min(xall)-mx max(xall)+mx]; yl=[min(yall)-mx max(yall)+mx];
    if ~isempty(M); xl=[min(xl(1),M.xdata(1)) max(xl(2),M.xdata(2))]; yl=[min(yl(1),M.ydata(2)) max(yl(2),M.ydata(1))]; end
    haveP = ~isempty(pose);
    if haveP; ptime=toNum(pose.timestamp_s)-t0; end
    for k=1:nF
        clf(fig); ax=axes(fig); hold(ax,'on'); %#ok<LAXES>
        drawMap(M); tc=tg(k);
        for i=1:numel(robots)
            r=robots{i}; c=colOf(r); clt=c+(1-c)*0.6;
            % global path referensi (warna pudar)
            if ~isempty(gpath)
                mg=strcmp(string(gpath.robot),r);
                if any(mg); gp=sortrows(gpath(mg,:),'point_index'); plot(gp.x,gp.y,'--','Color',clt,'LineWidth',1.0,'HandleVisibility','off'); end
            end
            % local plan terbaru pada/atau sebelum tc (siklus DWA terkini)
            ml = strcmp(string(lp.robot),r) & lpt<=tc;
            if any(ml)
                sub = lp(ml,:);
                lastSeq = max(sub.plan_seq);
                cur = sortrows(sub(sub.plan_seq==lastSeq,:),'point_index');
                plot(cur.x,cur.y,'-','Color',c,'LineWidth',2.6,'HandleVisibility','off');
                plot(cur.x,cur.y,'.','Color',c,'MarkerSize',9,'HandleVisibility','off');
                plot(cur.x(end),cur.y(end),'d','MarkerFaceColor',c,'MarkerEdgeColor','k','MarkerSize',8,'HandleVisibility','off');
            end
            % posisi robot terkini
            if haveP
                mp = strcmp(string(pose.robot),r) & ptime<=tc;
                if any(mp); idx=find(mp,1,'last'); plot(pose.x(idx),pose.y(idx),'o','MarkerFaceColor',c,'MarkerEdgeColor','k','MarkerSize',11,'HandleVisibility','off'); end
            end
        end
        axis(ax,'equal'); xlim(ax,xl); ylim(ax,yl); grid(ax,'on'); box(ax,'on');
        xlabel(ax,'x (m)'); ylabel(ax,'y (m)');
        title(ax,sprintf('Proses DWA (%s): garis tebal = local path (lookahead) terkini, putus = global path  |  t = %.1f s', scn, tc));
        writeVideo(vw, getframe(fig));
    end
    close(vw); close(fig);
    fprintf('Video DWA tersimpan: %s\n', outFile);
end

%% ===================== VIDEO PEMBENTUKAN GLOBAL PATH (DIJKSTRA) =====================
function makeGlobalPathVideo(outFile, M, robots, gpath, fps, scn)
%MAKEGLOBALPATHVIDEO  Animasi pembentukan global path (hasil Dijkstra) yang
% digambar bertahap dari start -> goal untuk ketiga robot. Catatan: path_log
% berisi RANGKAIAN titik hasil Dijkstra (output planner); ekspansi node
% internal Dijkstra tidak tercatat, jadi animasi menelusuri jalur hasilnya.
    if isempty(gpath) || height(gpath)==0; fprintf('Lewati video global path: path_log kosong.\n'); return; end
    if nargin<5 || isempty(fps); fps=15; end
    fprintf('Membuat video pembentukan global path (Dijkstra)...\n');
    G=struct(); maxN=0;
    for i=1:numel(robots)
        r=robots{i}; m=strcmp(string(gpath.robot),r);
        if any(m); gp=sortrows(gpath(m,:),'point_index'); G.(r)=[gp.x gp.y]; maxN=max(maxN,size(G.(r),1)); else; G.(r)=[]; end
    end
    if maxN<2; fprintf('Lewati video global path: titik tidak cukup.\n'); return; end
    DUR=8; nF=max(round(DUR*fps),2);     % durasi animasi pembentukan ~8 s
    try; vw=VideoWriter(outFile,'MPEG-4');
    catch; outFile=strrep(outFile,'.mp4','.avi'); vw=VideoWriter(outFile,'Motion JPEG AVI'); end
    vw.FrameRate=fps; open(vw);
    fig=figure('Color','w','Position',[100 100 860 700],'Visible','off');
    allx=gpath.x; ally=gpath.y; mx=0.5;
    xl=[min(allx)-mx max(allx)+mx]; yl=[min(ally)-mx max(ally)+mx];
    if ~isempty(M); xl=[min(xl(1),M.xdata(1)) max(xl(2),M.xdata(2))]; yl=[min(yl(1),M.ydata(2)) max(yl(2),M.ydata(1))]; end
    for k=1:nF
        frac=k/nF; clf(fig); ax=axes(fig); hold(ax,'on'); %#ok<LAXES>
        drawMap(M);
        for i=1:numel(robots)
            r=robots{i}; c=colOf(r); Pg=G.(r);
            if isempty(Pg); continue; end
            n=size(Pg,1); ne=max(2,round(frac*n));
            plot(Pg(:,1),Pg(:,2),':','Color',c+(1-c)*0.7,'LineWidth',1.0,'HandleVisibility','off');   % target samar
            plot(Pg(1:ne,1),Pg(1:ne,2),'-','Color',c,'LineWidth',2.4,'HandleVisibility','off');         % jalur tumbuh
            plot(Pg(1,1),Pg(1,2),'o','MarkerFaceColor','w','MarkerEdgeColor',c,'MarkerSize',9,'LineWidth',1.6,'HandleVisibility','off');
            plot(Pg(ne,1),Pg(ne,2),'.','Color',c,'MarkerSize',20,'HandleVisibility','off');             % ujung frontier
            if frac>=1; plot(Pg(end,1),Pg(end,2),'p','MarkerFaceColor',c,'MarkerEdgeColor','k','MarkerSize',15,'HandleVisibility','off'); end
        end
        axis(ax,'equal'); xlim(ax,xl); ylim(ax,yl); grid(ax,'on'); box(ax,'on');
        xlabel(ax,'x (m)'); ylabel(ax,'y (m)');
        title(ax,sprintf('Pembentukan global path Dijkstra (%s) - %d%%', scn, round(frac*100)));
        writeVideo(vw, getframe(fig));
    end
    for k=1:round(fps*1.0); writeVideo(vw, getframe(fig)); end   % tahan 1 s di akhir
    close(vw); close(fig);
    fprintf('Video global path tersimpan: %s\n', outFile);
end
