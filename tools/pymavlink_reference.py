"""Reference statistics for a .tlog from pymavlink (ardupilotmega/all dialect, robust parsing)."""
import sys, json, collections, os
from pymavlink import mavutil
path = sys.argv[1]
m = mavutil.mavlink_connection(path, dialect='all', robust_parsing=True)
by_id = collections.Counter(); by_src = collections.Counter(); bad = 0; first = last = None; n = 0
seq = {}; lost = collections.Counter(); dup = collections.Counter(); ooo = collections.Counter()
while True:
    msg = m.recv_msg()
    if msg is None:
        break
    if msg.get_type() == 'BAD_DATA':
        bad += 1; continue
    n += 1
    src = f"{msg.get_srcSystem()}/{msg.get_srcComponent()}"
    by_id[msg.get_msgId()] += 1; by_src[src] += 1
    s = msg.get_seq()
    if src in seq:
        step = (s - seq[src]) % 256
        if step == 0: dup[src] += 1
        elif step <= 128: lost[src] += step - 1; seq[src] = s
        else: ooo[src] += 1
    else:
        seq[src] = s
    ts = int(round(msg._timestamp * 1e6))
    first = ts if first is None else first; last = ts
print(json.dumps({"file": os.path.basename(path), "messages": n, "pymavlink_bad_data": bad,
  "first_us": first, "last_us": last, "by_msgid": {str(k): v for k, v in sorted(by_id.items())},
  "by_source": dict(sorted(by_src.items())), "lost": dict(sorted(lost.items())), "duplicates": dict(sorted(dup.items())),
  "out_of_order": dict(sorted(ooo.items()))}, indent=1))
