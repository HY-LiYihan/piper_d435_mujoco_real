# Hardware setup

Install `piper-control[real,camera]`, configure the CAN interface using the scripts in `vendor/piper_sdk`, and connect a D435/D435i. The default camera stream is aligned 1280x720 RGB/depth at 30 FPS.

Use `piper doctor` before connecting. The real backend defaults to `can0`; override with `--can-name`. Motion is available after connection, so test with the arm clear and keep the physical emergency stop accessible.
