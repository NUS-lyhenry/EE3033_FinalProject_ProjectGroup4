#!/usr/bin/env bash
# Build the matching upstream usb_cam library with its microsecond conversion fixed.
# Run with an existing official 0.8.1 checkout. Does not replace apt-owned files.
set -euo pipefail
source_dir="${1:?Provide the official usb_cam 0.8.1 source directory}"
output_dir="${2:?Provide the output directory for the fixed library}"
python3 - "$source_dir" <<'PY'
from pathlib import Path
import sys
root = Path(sys.argv[1])
if '<version>0.8.1</version>' not in (root/'package.xml').read_text():
    raise SystemExit('This fix is limited to usb_cam 0.8.1')
header = root/'include/usb_cam/utils.hpp'
text = header.read_text()
bad = 'epoch_time.tv_sec * 1000000 + epoch_time.tv_usec / 1000.0;'
good = 'epoch_time.tv_sec * 1000000 + epoch_time.tv_usec;'
if bad not in text and good not in text:
    raise SystemExit('Unrecognized timebase implementation; inspect before building')
header.write_text(text.replace(bad, good))
PY
mkdir -p "$output_dir/lib"
# The ROS node and its settings are reused unchanged; only the camera library changes.
read -r -a camera_flags <<< "$(pkg-config --cflags --libs opencv4 libavcodec libavutil libswscale)"
c++ -std=c++17 -O2 -fPIC -shared "$source_dir/src/usb_cam.cpp" \
    -I"$source_dir/include" "${camera_flags[@]}" -o "$output_dir/lib/libusb_cam.so"
cat > "$output_dir/check_clock.cpp" <<'CPP'
#include <cmath>
#include <iostream>
#include "usb_cam/utils.hpp"
int main() {
  const auto shift = usb_cam::utils::get_epoch_time_shift_us();
  timespec mono{}, real{};
  clock_gettime(CLOCK_MONOTONIC, &mono);
  clock_gettime(CLOCK_REALTIME, &real);
  const double corrected = mono.tv_sec + mono.tv_nsec*1e-9 + shift*1e-6;
  const double reference = real.tv_sec + real.tv_nsec*1e-9;
  const double error = std::abs(corrected-reference);
  std::cout << "clock conversion error seconds=" << error << std::endl;
  return error < .005 ? 0 : 1;
}
CPP
c++ -std=c++17 -O2 -I"$source_dir/include" "$output_dir/check_clock.cpp" -o "$output_dir/check_clock"
"$output_dir/check_clock"
printf 'Fixed library: %s/lib/libusb_cam.so\n' "$output_dir"
