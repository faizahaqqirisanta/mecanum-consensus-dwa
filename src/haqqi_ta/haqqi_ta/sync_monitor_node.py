#!/usr/bin/env python3
"""
sync_monitor_node.py — haqqi_ta
Monitor ringkas sinkronisasi arrival semua robot dalam satu terminal.

Jalankan di PC Master (domain 44):
  ros2 run haqqi_ta sync_monitor_node

Output diperbarui setiap 0.5 detik, clear terminal tiap update.

Topic yang dimonitor (dari consensus_node + DWA debug publisher):
  /robotX/mission_remaining_length  — Float32
  /robotX/mission_total_length      — Float32
  /robotX/vmax_consensus            — Float32
  /robotX/goal_reached              — Bool
  /robotX/priority_stop             — Bool
  /robotX/fault_active              — Bool
  /robotX/dwa_vmax_eff              — Float32 (dari modified_dwa_node MOD-13, fisik)
  /robotX/dwa_speed_mag             — Float32 (dari modified_dwa_node MOD-13, fisik)
  /robotX/dwa_mode                  — String  (dari modified_dwa_node MOD-13, fisik)
"""

import os
import sys
import time
import math
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, String

ROBOTS = ['robot1', 'robot2', 'robot3']
SHORT  = {'robot1': 'r1', 'robot2': 'r2', 'robot3': 'r3'}


class SyncMonitorNode(Node):

    def __init__(self):
        super().__init__('sync_monitor_node')

        self._t_start = self.get_clock().now()

        # [M4] Status detektor kegagalan agen (dari /coordination_debug)
        self._detection_enabled = None   # None=belum ada data, True/False
        self._coord_t_last      = None   # waktu update terakhir coordination_debug

        # ── Per-robot data store ──────────────────────────────────────────
        self._data = {
            ns: {
                'rem':        None,   # mission_remaining_length (m)
                'total':      None,   # mission_total_length (m)
                'vcons':      None,   # vmax_consensus
                'goal':       None,   # Bool
                'pstop':      None,   # Bool
                'fault':      None,   # Bool
                'veff':       None,   # dwa_vmax_eff
                'mag':        None,   # dwa_speed_mag
                'mode':       None,   # dwa_mode
                'failed':     None,   # [M4] detektor: robot ditandai gagal (/coordination_debug)
                't_last':     None,   # last update time (for staleness)
            }
            for ns in ROBOTS
        }

        # ── Subscriptions ─────────────────────────────────────────────────
        for ns in ROBOTS:
            d = self._data[ns]

            def mk_float_cb(d_ref, key):
                def cb(msg): d_ref[key] = msg.data; d_ref['t_last'] = time.monotonic()
                return cb

            def mk_bool_cb(d_ref, key):
                def cb(msg): d_ref[key] = msg.data; d_ref['t_last'] = time.monotonic()
                return cb

            def mk_str_cb(d_ref, key):
                def cb(msg): d_ref[key] = msg.data; d_ref['t_last'] = time.monotonic()
                return cb

            self.create_subscription(Float32, f'/{ns}/mission_remaining_length',
                                     mk_float_cb(d, 'rem'),   10)
            self.create_subscription(Float32, f'/{ns}/mission_total_length',
                                     mk_float_cb(d, 'total'), 10)
            self.create_subscription(Float32, f'/{ns}/vmax_consensus',
                                     mk_float_cb(d, 'vcons'), 10)
            self.create_subscription(Bool,    f'/{ns}/goal_reached',
                                     mk_bool_cb(d, 'goal'),   10)
            self.create_subscription(Bool,    f'/{ns}/priority_stop',
                                     mk_bool_cb(d, 'pstop'),  10)
            self.create_subscription(Bool,    f'/{ns}/fault_active',
                                     mk_bool_cb(d, 'fault'),  10)
            self.create_subscription(Float32, f'/{ns}/dwa_vmax_eff',
                                     mk_float_cb(d, 'veff'),  10)
            self.create_subscription(Float32, f'/{ns}/dwa_speed_mag',
                                     mk_float_cb(d, 'mag'),   10)
            self.create_subscription(String,  f'/{ns}/dwa_mode',
                                     mk_str_cb(d, 'mode'),    10)

        # ── Status deteksi agen gagal [M4] dari consensus_node ─────────────
        # Dipublikasikan sbg satu JSON di /coordination_debug (bukan per-robot).
        self.create_subscription(String, '/coordination_debug',
                                 self._coord_debug_cb, 10)

        # ── Display timer ─────────────────────────────────────────────────
        self.create_timer(0.5, self._render)

    # ─────────────────────────────────────────────────────────────────────
    # COORDINATION DEBUG (status deteksi)
    # ─────────────────────────────────────────────────────────────────────

    def _coord_debug_cb(self, msg):
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        failed = payload.get('failed', {}) or {}
        for ns in ROBOTS:
            fv = failed.get(ns, None)
            self._data[ns]['failed'] = None if fv is None else bool(fv)
        de = payload.get('detection_enabled', None)
        self._detection_enabled = None if de is None else bool(de)
        self._coord_t_last = time.monotonic()

    # ─────────────────────────────────────────────────────────────────────
    # RENDER
    # ─────────────────────────────────────────────────────────────────────

    def _render(self):
        now_ros  = self.get_clock().now()
        elapsed  = (now_ros - self._t_start).nanoseconds * 1e-9

        # Compute mission_p per robot
        p_vals = {}
        for ns in ROBOTS:
            d = self._data[ns]
            rem   = d['rem']
            total = d['total']
            if rem is not None and total is not None and total > 0.05:
                p_vals[ns] = max(0.0, min(1.0, 1.0 - rem / total))
            else:
                p_vals[ns] = None

        # Summary stats
        valid_p  = [v for v in p_vals.values() if v is not None]
        p_avg    = sum(valid_p) / len(valid_p) if valid_p else None
        p_spread = (max(valid_p) - min(valid_p)) if len(valid_p) > 1 else None

        leader = lagger = None
        if len(valid_p) > 1:
            leader = max(ROBOTS, key=lambda ns: p_vals[ns] if p_vals[ns] is not None else -1)
            lagger = min(ROBOTS, key=lambda ns: p_vals[ns] if p_vals[ns] is not None else 2)

        # ── Build output ──────────────────────────────────────────────────
        lines = []
        lines.append('')
        lines.append(f'  ╔══════════════════════════════════════════════════════════════════╗')
        lines.append(f'  ║   SYNC MONITOR  |  t = {elapsed:6.1f}s                              ║')
        lines.append(f'  ╚══════════════════════════════════════════════════════════════════╝')
        lines.append('')

        # Summary line
        p_avg_str    = f'{p_avg:.3f}'    if p_avg    is not None else '  --- '
        spread_str   = f'{p_spread:.3f}' if p_spread is not None else '  --- '
        leader_str   = SHORT.get(leader, '---') if leader else '---'
        lagger_str   = SHORT.get(lagger, '---') if lagger else '---'
        lines.append(f'  p_avg={p_avg_str}  spread={spread_str}  '
                     f'leader={leader_str}  lagger={lagger_str}')

        # [M4] Status detektor kegagalan agen
        if self._detection_enabled is None:
            det_str = '---'
        else:
            det_str = 'ON' if self._detection_enabled else 'OFF'
        failed_robots = [SHORT[ns] for ns in ROBOTS if self._data[ns]['failed']]
        failed_str = ', '.join(failed_robots) if failed_robots else 'none'
        lines.append(f'  detection={det_str}  failed=[{failed_str}]')
        lines.append('')

        # Table header
        lines.append(f'  {"robot":<6} {"m_p":>6} {"remM":>6} '
                     f'{"vcons":>6} {"veff":>6} {"mag":>6} '
                     f'{"goal":>5} {"pstop":>5} {"fault":>5} {"fail":>5} {"mode":<7}')
        lines.append('  ' + '─' * 74)

        for ns in ROBOTS:
            d = self._data[ns]
            p   = p_vals[ns]
            rem = d['rem']

            p_str    = f'{p:.3f}'          if p           is not None else ' --- '
            rem_str  = f'{rem:.2f}'         if rem         is not None else ' --- '
            vc_str   = f'{d["vcons"]:.3f}' if d['vcons']  is not None else ' --- '
            ve_str   = f'{d["veff"]:.3f}'  if d['veff']   is not None else ' --- '
            mag_str  = f'{d["mag"]:.3f}'   if d['mag']    is not None else ' --- '
            goal_str = 'YES' if d['goal']  else ('no'  if d['goal']  is not None else '---')
            ps_str   = 'YES' if d['pstop'] else ('no'  if d['pstop'] is not None else '---')
            fa_str   = 'YES' if d['fault'] else ('no'  if d['fault'] is not None else '---')
            fail_str = 'FAIL' if d['failed'] else ('ok'  if d['failed'] is not None else '---')
            mode_str = (d['mode'] or '---')[:7]

            # Highlight robot that is leading or lagging
            tag = ''
            if leader and ns == leader and p_spread is not None and p_spread > 0.05:
                tag = ' ◀ lead'
            elif lagger and ns == lagger and p_spread is not None and p_spread > 0.05:
                tag = ' ▶ lag '

            lines.append(f'  {SHORT[ns]:<6} {p_str:>6} {rem_str:>6} '
                         f'{vc_str:>6} {ve_str:>6} {mag_str:>6} '
                         f'{goal_str:>5} {ps_str:>5} {fa_str:>5} {fail_str:>5} {mode_str:<7}{tag}')

        lines.append('')

        # Staleness warning
        now_mono = time.monotonic()
        stale = [ns for ns in ROBOTS
                 if self._data[ns]['t_last'] is not None
                 and now_mono - self._data[ns]['t_last'] > 3.0]
        if stale:
            lines.append(f'  ⚠ STALE data: {", ".join(stale)} (>3s since last update)')
        no_data = [ns for ns in ROBOTS if self._data[ns]['t_last'] is None]
        if no_data:
            lines.append(f'  · Waiting for: {", ".join(no_data)}')

        lines.append('')

        # Clear and print
        os.system('clear')
        print('\n'.join(lines), flush=True)


# ═══════════════════════════════════════════════════════════════════════════
# ENTRY POINT
# ═══════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = SyncMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
