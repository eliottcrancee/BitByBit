"""Agrège un profil samply (format Firefox Profiler) : self-time par fonction."""
import gzip
import json
from collections import defaultdict
from pathlib import Path

path = Path(__file__).parent / "profile_bench.json.gz"
path = Path(__file__).parent / "profile_bench.json.gz"
data = json.loads(gzip.decompress(path.read_bytes()))

# On prend le thread de rust.exe avec le plus d'échantillons
best = None
for th in data.get("threads", []):
    if "rust.exe" not in str(th.get("processName", "")):
        continue
    n = int(th["samples"].get("length", 0))
    if best is None or n > best[0]:
        best = (n, th)

if best is None:
    raise SystemExit("Thread de rust.exe introuvable")

n, th = best
strings = th["stringArray"]
funcs = th["funcTable"]
frames = th["frameTable"]
stacks = th["stackTable"]

def func_name(i: int) -> str:
    return strings[funcs["name"][i]]

def stack_funcs(stack_idx: int) -> list[str]:
    out = []
    while stack_idx is not None and stack_idx >= 0:
        f = frames["func"][stacks["frame"][stack_idx]]
        out.append(func_name(f))
        stack_idx = stacks["prefix"][stack_idx]
    return out

deltas = th["samples"]["timeDeltas"]
total = sum(deltas)

self_time: dict[str, int] = defaultdict(int)
incl_time: dict[str, int] = defaultdict(int)
for i in range(n):
    dt = deltas[i]
    if dt <= 0:
        continue
    fl = stack_funcs(th["samples"]["stack"][i])
    if not fl:
        continue
    leaf = fl[0].split(" (")[0]  # retire le suffixe d'adresse
    self_time[leaf] += dt
    seen = set()
    for fnm in fl:
        base = fnm.split(" (")[0]
        if base not in seen:
            seen.add(base)
            incl_time[base] += dt

print(f"\nDurée totale profilée : {total/1e6:.2f} s, {n} échantillons\n")
print(f"{'SELF %':>7} {'SELF ms':>9}  fonction")
for name, t in sorted(self_time.items(), key=lambda x: -x[1])[:25]:
    print(f"{100*t/total:6.1f}% {t/1000:9.1f}  {name}")

print("\n--- Temps inclusif (top 15) ---")
for name, t in sorted(incl_time.items(), key=lambda x: -x[1])[:15]:
    print(f"{100*t/total:6.1f}% {t/1000:9.1f}  {name}")
