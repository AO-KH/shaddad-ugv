#!/bin/bash
# run_batch.sh <cap_seconds> <log> "<matlab command>"
# Runs one MATLAB command in its own batch MATLAB with the Shaddad project open, once no other batch MATLAB is running
# (one heavy MATLAB at a time on this 16 GB PC). The user's own MATLAB, shared or not, is never waited for or touched.
# After <cap_seconds> the run is stopped: only the MATLAB whose command line carries this call's marker is killed.
# A relative <log> is taken from the project root. Run it in the background for anything longer than a few minutes.
# Example: bash tools/agentic/run_batch.sh 3600 work/agentic/logs/t_sensors.log "shaddad_run_tests('t_sensors')"
# 2026-11-15: a lock directory (work/agentic/.batch_lock, made with the atomic mkdir) now guards the start, because two
# callers waiting in the same 20 s poll could both see "no batch MATLAB" and start together. A lock whose owner shell is
# gone, or that is older than its cap plus 10 min, is stale and is removed.
cap=$1; log=$2; cmd=$3
if [ -z "$cmd" ]; then echo "usage: run_batch.sh <cap_seconds> <log> \"<matlab command>\""; exit 2; fi
root="$(cd "$(dirname "$0")/../.." && pwd)"
rootw="$(cygpath -w "$root")"
cd "$root" || exit 2
mkdir -p "$(dirname "$log")" "$root/work/agentic"
marker="agentbatch_$$_$RANDOM"
lock="$root/work/agentic/.batch_lock"
count="(Get-CimInstance Win32_Process -Filter \"Name='MATLAB.exe'\" | Where-Object CommandLine -match '-batch' | Measure-Object).Count"
while true; do
  if mkdir "$lock" 2>/dev/null; then
    echo "$$ $(date +%s) $cap" > "$lock/owner"
    if [ "$(powershell -NoProfile -Command "$count")" = "0" ]; then break; fi
    rm -rf "$lock"                      # a batch MATLAB from elsewhere is still running: give the lock back
  elif [ -f "$lock/owner" ]; then
    read -r opid ostart ocap < "$lock/owner"
    now=$(date +%s)
    if [ -n "$opid" ] && ! kill -0 "$opid" 2>/dev/null; then
      echo "$(date +%T) removing a stale batch lock (owner $opid gone)"; rm -rf "$lock"; continue
    fi
    if [ -n "$ostart" ] && [ -n "$ocap" ] && [ $((now - ostart)) -gt $((ocap + 600)) ]; then
      echo "$(date +%T) removing a stale batch lock (older than its cap)"; rm -rf "$lock"; continue
    fi
  fi
  echo "$(date +%T) waiting for another batch MATLAB"; sleep 20
done
trap 'rm -rf "$lock"' EXIT
echo "$(date +%T) start: $cmd"
"/c/Program Files/MATLAB/R2026b/bin/matlab.exe" -batch "cd('$rootw'); openProject('Shaddad_UGV.prj'); $cmd % $marker" > "$log" 2>&1 &
pid=$!
t=0
while kill -0 $pid 2>/dev/null; do
  sleep 5; t=$((t+5))
  if [ $t -ge "$cap" ]; then
    echo "CAP: $cap s reached, stopping $marker"
    powershell -NoProfile -Command "Get-CimInstance Win32_Process -Filter \"Name='MATLAB.exe'\" | Where-Object { \$_.CommandLine -match '$marker' } | ForEach-Object { Stop-Process -Id \$_.ProcessId -Force }"
    sleep 3
    break
  fi
done
wait $pid 2>/dev/null
rc=$?
echo "$(date +%T) done after about $t s, exit code $rc (log $log)"
