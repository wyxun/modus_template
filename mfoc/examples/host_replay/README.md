# Standalone Motor replay

From this directory, run `make` with a host GCC and GNU Make. Use
`mingw32-make` on Windows. The default runs one FLOAT POSITION build.

```sh
make NUMERIC=FIXED BACKEND=POSITION IDENTIFY=1
make NUMERIC=FLOAT BACKEND=SMO
```

`BACKEND` accepts `NONE`, `POSITION`, `SMO`, `HFI`, `FLUX`, and
`EKF`. Selecting a backend enables the required provider. The
`POSITION`, `SMO`, `HFI`, `FLUX`, and `EKF` flags also allow compiling
secondary providers. `IDENTIFY=1` runs the Rs experiment through Motor, and
`MODEL_UPDATE=1` exercises applying a model while stopped. EKF needs FLOAT.
The default trig implementation is `TRIG=LUT`. `TRIG=LIBM` needs FLOAT, and
`TRIG=EXTERNAL` uses the direct Port implementation supplied by this example.
The same preprocessor settings apply to the example and every library
translation unit.

The single-threaded Port simulates a star-connected resistive load for one
interval per submitted PWM frame. It supplies fresh current, a stationary
position for POSITION builds, a clock, and PWM gate state. POSITION additionally
compares interleaved Motor instances with an isolated replay. The other
profiles exercise forced rotation or HFI acquisition on this simulated load.
Use the observer tests and board validation for lock and handover behavior.
The host-only external trig binding uses libm to simulate a target adapter.

The Makefile uses the same `mfoc.mk` source manifest as the board build.
The full link uses standard C memory and math routines, the host C runtime,
and compiler runtime helpers.
