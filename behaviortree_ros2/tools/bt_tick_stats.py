#!/usr/bin/env python3
import re
import sys
import math
import time
import argparse
from collections import defaultdict

PAT = re.compile(r"\[(?P<node>[^\]]+)\]\s+tick\(\)\s+took\s+(?P<ms>[0-9]*\.?[0-9]+)\s*ms")

def percentile(sorted_vals, p: float) -> float:
    if not sorted_vals:
        return float("nan")
    if p <= 0:
        return sorted_vals[0]
    if p >= 100:
        return sorted_vals[-1]
    k = int(math.ceil((p / 100.0) * len(sorted_vals))) - 1
    k = max(0, min(k, len(sorted_vals) - 1))
    return sorted_vals[k]

def compute_stats(values):
    n = len(values)
    vals = sorted(values)
    mean = sum(vals) / n
    mn, mx = vals[0], vals[-1]
    if n > 1:
        var = sum((x - mean) ** 2 for x in vals) / (n - 1)
        stdev = math.sqrt(var)
    else:
        stdev = 0.0
    return {
        "count": n,
        "avg": mean,
        "min": mn,
        "p50": percentile(vals, 50),
        "p90": percentile(vals, 90),
        "p95": percentile(vals, 95),
        "p99": percentile(vals, 99),
        "max": mx,
        "stdev": stdev,
    }

def print_report(data, top=0, title=None):
    if not data:
        return
    if title:
        print(title)

    header = f"{'Node':28} {'n':>8} {'avg':>10} {'min':>10} {'p50':>10} {'p90':>10} {'p95':>10} {'p99':>10} {'max':>10} {'stdev':>10}"
    print(header)
    print("-" * len(header))

    # sort by count desc
    for node in sorted(data.keys(), key=lambda k: len(data[k]), reverse=True):
        st = compute_stats(data[node])
        print(
            f"{node:28} "
            f"{st['count']:8d} "
            f"{st['avg']:10.3f} "
            f"{st['min']:10.3f} "
            f"{st['p50']:10.3f} "
            f"{st['p90']:10.3f} "
            f"{st['p95']:10.3f} "
            f"{st['p99']:10.3f} "
            f"{st['max']:10.3f} "
            f"{st['stdev']:10.3f}"
        )

    if top > 0:
        all_samples = []
        for node, vals in data.items():
            for v in vals:
                all_samples.append((v, node))
        all_samples.sort(reverse=True)
        print(f"\nTop {top} slowest ticks:")
        for v, node in all_samples[:top]:
            print(f"  {v:8.3f} ms  {node}")
    print("", flush=True)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--every", type=float, default=5.0, help="Print summary every N seconds (0 disables periodic prints)")
    ap.add_argument("--top", type=int, default=5, help="Show top N slowest samples (0 disables)")
    ap.add_argument("--since", default=None, help='Pass-through hint; use with docker logs --since. (Not used by parser)')
    args = ap.parse_args()

    data = defaultdict(list)
    last_print = time.monotonic()

    def maybe_print(force=False, reason=""):
        nonlocal last_print
        if not data:
            return
        now = time.monotonic()
        if force or (args.every > 0 and (now - last_print) >= args.every):
            ts = time.strftime("%Y-%m-%d %H:%M:%S")
            title = f"\n=== BT tick stats ({ts}) {reason} ==="
            print_report(data, top=args.top, title=title)
            last_print = now

    try:
        for line in sys.stdin:
            m = PAT.search(line)
            if m:
                node = m.group("node")
                ms = float(m.group("ms"))
                data[node].append(ms)
            maybe_print()
    except KeyboardInterrupt:
        # Print what we have so far
        maybe_print(force=True, reason="(final, interrupted)")
        return 0

    # EOF path
    maybe_print(force=True, reason="(final)")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
