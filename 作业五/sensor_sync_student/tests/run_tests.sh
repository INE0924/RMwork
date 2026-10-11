#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."
cmake -S . -B build >/dev/null
cmake --build build -j >/dev/null
./build/sensor_sync config/sensor_sync.conf >/dev/null
diff -u data/expected_result.txt data/result.txt
printf 'PASS: public sample\n'

tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT

write_config() {
  local capacity=${1:-1} max_delta=${2:-200}
  cat > "$tmp/sensor_sync.conf" <<CONFIG
queue_capacity=$capacity
max_delta_us=$max_delta
camera_file=camera.txt
imu_file=imu.txt
output_file=actual.txt
CONFIG
}

run_ok() {
  timeout 5 ./build/sensor_sync "$tmp/sensor_sync.conf" >/dev/null
  diff -u "$tmp/expected.txt" "$tmp/actual.txt"
}

run_fail() {
  if timeout 5 ./build/sensor_sync "$tmp/sensor_sync.conf" >"$tmp/stdout.txt" 2>"$tmp/stderr.txt"; then
    echo "FAIL: expected a nonzero exit code" >&2
    exit 1
  fi
  if ! grep -q 'sensor_sync error:' "$tmp/stderr.txt"; then
    echo "FAIL: missing diagnostic error" >&2
    cat "$tmp/stderr.txt" >&2
    exit 1
  fi
}

# Ties: earlier timestamp wins. Same timestamp: smaller sequence wins.
# Two cameras at the same time must be output in sequence order.
write_config 1 200
cat >"$tmp/camera.txt" <<'CAMERA'
9 900
3 1000
1 1000
5 1100
7 5000
CAMERA
cat >"$tmp/imu.txt" <<'IMU'
4 800
8 1000
3 1000
9 1200
2 1200
IMU
cat >"$tmp/expected.txt" <<'EXPECTED'
CAMERA 9 IMU 4 DELTA 100
CAMERA 1 IMU 3 DELTA 0
CAMERA 3 IMU 3 DELTA 0
CAMERA 5 IMU 3 DELTA 100
CAMERA 7 UNMATCHED
EXPECTED
run_ok
printf 'PASS: tie-breaking, sorting, reuse and queue capacity 1\n'

# Exact-match-only when max_delta_us=0.
write_config 1 0
printf '10 250\n11 260\n' >"$tmp/camera.txt"
printf '4 250\n' >"$tmp/imu.txt"
printf 'CAMERA 10 IMU 4 DELTA 0\nCAMERA 11 UNMATCHED\n' >"$tmp/expected.txt"
run_ok
printf 'PASS: zero tolerance\n'

# Full unsigned 64-bit timestamp, and a one-microsecond difference.
write_config 1 1
printf '8 18446744073709551615\n' >"$tmp/camera.txt"
printf '7 18446744073709551614\n' >"$tmp/imu.txt"
printf 'CAMERA 8 IMU 7 DELTA 1\n' >"$tmp/expected.txt"
run_ok
printf 'PASS: uint64 timestamp boundary\n'

# Empty IMU file is valid: all cameras are unmatched.
write_config 1 200
printf '0 0\n1 20\n' >"$tmp/camera.txt"
: >"$tmp/imu.txt"
printf 'CAMERA 0 UNMATCHED\nCAMERA 1 UNMATCHED\n' >"$tmp/expected.txt"
run_ok
printf 'PASS: empty IMU input\n'

# Format errors, timestamp regression, failed opens, and invalid config.
write_config 1 200
printf '0 100\n\n' >"$tmp/camera.txt"
printf '0 100\n' >"$tmp/imu.txt"
run_fail
printf 'PASS: blank input line rejected\n'

printf '0 100\n1 99\n' >"$tmp/camera.txt"
run_fail
printf 'PASS: non-monotonic timestamp rejected\n'

printf '0 100 extra\n' >"$tmp/camera.txt"
run_fail
printf 'PASS: extra field rejected\n'

printf '%s\n' '-1 100' >"$tmp/camera.txt"
run_fail
printf 'PASS: negative sequence rejected\n'

printf '0 100\n' >"$tmp/camera.txt"
rm "$tmp/imu.txt"
run_fail
printf 'PASS: missing input file exits without deadlock\n'

write_config 0 200
run_fail
printf 'PASS: invalid queue capacity rejected\n'

# Stress a capacity-1 queue: both producers must make progress.
write_config 1 0
awk 'BEGIN {for (i=0; i<1500; ++i) print i, i*100}' >"$tmp/camera.txt"
cp "$tmp/camera.txt" "$tmp/imu.txt"
timeout 10 ./build/sensor_sync "$tmp/sensor_sync.conf" >/dev/null
if [[ $(wc -l <"$tmp/actual.txt") -ne 1500 ]] ||
   ! grep -q '^CAMERA 1499 IMU 1499 DELTA 0$' "$tmp/actual.txt"; then
  echo "FAIL: stress run had missing or incorrect data" >&2
  exit 1
fi
printf 'PASS: concurrent producer/consumer stress test\n'

echo 'All tests passed.'
