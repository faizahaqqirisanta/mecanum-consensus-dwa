#!/usr/bin/env python3
"""[FIX-VARQUANT] Agregasi variabilitas terminal lintas banyak trial.

Memindai satu/lebih direktori untuk file goal_result.csv (rekursif), lalu untuk
tiap robot menghitung success-rate & sebaran error akhir (mean/std/min/max/median).
Pakai settled_precision_m bila ada; fallback ke final_error_m / goal_precision_m.

Contoh:
  python3 aggregate_trials.py results/ results2/
  python3 aggregate_trials.py results/ --out ringkasan_variabilitas.csv
"""
import argparse, csv, glob, math, os, statistics as st

ROBOTS = ['robot1', 'robot2', 'robot3']
TRUE_SET = {'yes', 'true', '1', 'ok', 'success'}


def _f(v):
    try:
        x = float(v)
        return x if math.isfinite(x) else None
    except (TypeError, ValueError):
        return None


def find_csvs(paths, pattern):
    files = []
    if pattern:
        files += glob.glob(pattern, recursive=True)
    for p in paths:
        if os.path.isfile(p):
            files.append(p)
        elif os.path.isdir(p):
            files += glob.glob(os.path.join(p, '**', 'goal_result.csv'), recursive=True)
    seen, out = set(), []
    for f in files:
        rp = os.path.realpath(f)
        if rp not in seen:
            seen.add(rp); out.append(f)
    return sorted(out)


def pick_error(row):
    for key in ('settled_precision_m', 'final_error_m', 'goal_precision_m'):
        if key in row:
            val = _f(row[key])
            if val is not None:
                return val, key
    return None, None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('paths', nargs='*', default=[])
    ap.add_argument('--glob', dest='pattern', default=None)
    ap.add_argument('--out', default=None)
    ap.add_argument('--tol', type=float, default=0.15)
    args = ap.parse_args()

    csvs = find_csvs(args.paths, args.pattern)
    if not csvs:
        print('Tidak ada goal_result.csv ditemukan.'); return
    print(f'Trial ditemukan: {len(csvs)}')
    for c in csvs:
        print(f'  - {c}')

    acc = {r: {'err': [], 'ok': [], 'ctmax': [], 'src': set()} for r in ROBOTS}
    for path in csvs:
        with open(path, newline='') as fh:
            for row in csv.DictReader(fh):
                r = (row.get('robot') or '').strip()
                if r not in acc:
                    continue
                err, src = pick_error(row)
                if err is not None:
                    acc[r]['err'].append(err)
                    if src:
                        acc[r]['src'].add(src)
                sval = (row.get('position_success') or row.get('state_success') or '').strip().lower()
                if sval:
                    acc[r]['ok'].append(sval in TRUE_SET)
                elif err is not None:
                    acc[r]['ok'].append(err <= args.tol)
                ctm = _f(row.get('crosstrack_max_m'))
                if ctm is not None:
                    acc[r]['ctmax'].append(ctm)

    def stat_row(r):
        e = acc[r]['err']; ok = acc[r]['ok']
        n = len(e)
        sr = (100.0 * sum(ok) / len(ok)) if ok else float('nan')
        mean = st.mean(e) if e else float('nan')
        sd = st.pstdev(e) if len(e) > 1 else 0.0
        med = st.median(e) if e else float('nan')
        lo = min(e) if e else float('nan')
        hi = max(e) if e else float('nan')
        ct = st.mean(acc[r]['ctmax']) if acc[r]['ctmax'] else float('nan')
        src = '+'.join(sorted(acc[r]['src'])) or 'n/a'
        return n, sr, mean, sd, med, lo, hi, ct, src

    hdr = f"{'robot':8} {'n':>3} {'succ%':>6} {'mean':>8} {'std':>8} {'median':>8} {'min':>8} {'max':>8} {'ctmax':>8}  src"
    print('\n' + '=' * len(hdr)); print('VARIABILITAS TERMINAL'); print('=' * len(hdr))
    print(hdr); print('-' * len(hdr))
    rows_out = []
    for r in ROBOTS:
        n, sr, mean, sd, med, lo, hi, ct, src = stat_row(r)
        print(f'{r:8} {n:3d} {sr:6.1f} {mean:8.4f} {sd:8.4f} {med:8.4f} {lo:8.4f} {hi:8.4f} {ct:8.4f}  {src}')
        rows_out.append({'robot': r, 'n_trials': n, 'success_rate_pct': f'{sr:.2f}',
                         'err_mean_m': f'{mean:.5f}', 'err_std_m': f'{sd:.5f}',
                         'err_median_m': f'{med:.5f}', 'err_min_m': f'{lo:.5f}',
                         'err_max_m': f'{hi:.5f}', 'crosstrack_max_mean_m': f'{ct:.5f}',
                         'error_source': src})
    if args.out and rows_out:
        with open(args.out, 'w', newline='') as fh:
            w = csv.DictWriter(fh, fieldnames=list(rows_out[0].keys()))
            w.writeheader(); w.writerows(rows_out)
        print(f'\nRingkasan ditulis: {args.out}')


if __name__ == '__main__':
    main()
