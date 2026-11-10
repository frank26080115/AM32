# Noise simulation

## Motor-interference profiles

```sh
python tests/noise_sim.py --profile weapon-startup --seed 42
python tests/noise_sim.py --profile ground-bounce-startup --seed 42
python tests/noise_sim.py --profile path/to/custom.json --noise-rms-v 0.2 --seed 42
```

The six bundled profiles and tuning guidance are in
[`tests/noise_profiles/README.md`](../tests/noise_profiles/README.md). Without
`--profile`, all original Gaussian settings and random streams are preserved.
Named motor profiles use 0.1 us resolution; explicit CLI settings override the
profile's simulation settings. `--noise-rms-v` controls only Gaussian background
RMS, not the combined disturbance. Custom profiles inherit omitted fields from
weapon-startup defaults; unknown fields are rejected to catch configuration typos.

The core model injects opposite-polarity spikes at PWM rising/falling edges,
with duty-controlled spacing. Each triangular spike is followed by a damped
sinusoidal ring. Startup strength decays exponentially; electrical frequency
ramps linearly during spin-up, and its integrated phase modulates switching
amplitude. A separately decaying signed receiver-ground shift is subtracted
from the received signal. Motor components are zero before startup. Background,
switching phase, electrical phase and receiver phase use separate seeded streams.
Changing component amplitudes does not move switching events or receiver samples.

No normalization is applied to the combined waveform. Nominal ring amplitude
is the initial exponential-envelope amplitude; its actual first oscillation
peak is smaller because it decays. Overlapping tails add linearly and are
truncated after twelve decay constants. Spikes and rings are evaluated at their
actual age on the waveform grid, without rounding event phase to grid indices.
Spikes/ringing/ground arrays use float32 storage to reduce memory, while Gaussian
noise, filtering, and the combined disturbance use float64.

Resolution checks require at least ten samples per ring cycle and four across
a spike, as well as resolution of the PWM edge separation. Unresolved profiles
fail with a required maximum step instead of being silently aliased. Existing
clean-baseline reception checks still apply. Faster simulation resolution changes
white-noise bandwidth; a profile comparison against the original Gaussian default
therefore changes more than just interference type unless resolution is matched.

JSON and CSV summaries include the complete effective profile, random switching
and electrical phases, and measured RMS/absolute peak/mean for individual
components and the combined disturbance. Positive `ground_shift_v` is displacement
to subtract, not an additive noise component. The optional NPZ export includes
these component arrays. An additional disturbance PNG shows startup/electrical
envelopes, receiver ground displacement, and the first 300 us around motor startup.
Full binary export is always enabled. A two-second 100 ns run retains hundreds
of megabytes of arrays; longer high-resolution runs can require several gigabytes.

Profiles are illustrative GPIO-referred stress cases, not calibrated predictions
of a specific robot. They do not model supply clamps, motor dynamics, rare random
impulses, distinct commutation bursts, or CPU delays. Receiver behavior and the
perfect DShot integrity-check assumption remain unchanged. No parameter has been
tuned to force a particular protocol ranking.

## Original Gaussian mode

Run from the repository root:

```sh
python -m pip install -r tests/requirements-noise-sim.txt
python tests/noise_sim.py --seed 42
python tests/noise_sim.py --duration-s 10 --step-us 0.1 --noise-rms-v 2 --filter-tau-us 0 --seed 42 --save-waveforms
python -m unittest discover -s tests -p test_noise_sim.py -v
```

The default run lasts two seconds with 1 us resolution, 1 V RMS Gaussian white
noise, a 5 us low-pass time constant, estimated Schmitt thresholds of 1.75 V
rising / 1.55 V falling, and a 10 us PWM false-command tolerance. Thresholds,
duration, resolution, noise, filtering and tolerance are CLI parameters;
`--help` lists their names. Time steps must be multiples of 0.1 us. The script
rejects settings that cannot receive the clean baseline, including phase-dependent
failures from a coarse waveform grid. Try `--step-us 0.1` if this occurs; it does
not silently select a more favorable receiver phase. A zero filter time
constant bypasses filtering. An omitted seed is generated and recorded.

The noise is zero-mean and normalized to the requested measured RMS. All five
methods use the same noise timeline. PWM sends 1500 us pulses at 50 or 333 Hz;
DShot sends `0xAAAA` with no CRC calculation. DSHOT4/8/16 use 20/40/80 kHz
receiver sampling, five-sample bit periods, 30%/70% high times, and a five-sample
low pause. Streams start after 1 ms of idle. Receiver sampling uses a shared
random fractional phase and zero-order hold of the voltage grid. Separate random
streams generate noise and phase, so changing noise amplitude or duration does
not change receiver phase for a given seed. Incomplete
transmissions at the end are excluded from expected update counts.

Filtering uses `alpha = 1 - exp(-dt/tau)` and `y += alpha * (x-y)` from an
initial output of zero. GPIO transitions high at V_IH and low at V_IL, retaining
its state in between. These thresholds are an estimated GPIO model, not measured
F051 thresholds or its guaranteed datasheet input limits. Voltages are not
clipped to the supply rails; protection diodes and electrical damage are outside
the model.

PWM reception accepts widths strictly between 800 and 2200 us. An accepted
pulse is matched to an unmatched expected pulse by maximum temporal overlap.
Unmatched accepted pulses are spurious. Matched widths are compared to the
identically filtered, quantized clean baseline, avoiding false attribution of
filter delay to noise. Every nonzero width change is counted, but only changes
greater than the configured tolerance cause a false command. The last accepted
command remains active until another accepted pulse replaces it. No PWM ramping,
motor dynamics, calibration, arming or signal-loss shutdown is simulated.

DShot reception ports `slowDshotSamplePin()` from the firmware: five low samples
for idle recovery, contiguous high prefixes, low sample five, optional extra
low sample six, and synchronization on every bit. An ideal integrity oracle
rejects any incorrect word or packet alignment; a valid packet immediately
returns to waiting for the next packet. This deliberately assumes perfect
corruption rejection. A real four-bit CRC can miss some corrupted packets;
these results do not establish that false DShot commands are impossible.

Reports distinguish expected transmissions from receiver candidates. A damaged
DShot frame can generate several rejected candidates but counts as only one
failed expected update. PWM matched updates include accepted but incorrect
commands; see false-command counts separately. Overdue time starts at the clean
reference completion of a missing update and lasts until the next accepted
update, or the run ends. Overlapping missed-update intervals are merged. False
command time begins when a bad PWM command is accepted and ends with a correct
accepted command. These durations may overlap and must not be summed. Before
any command is received, false-command time is zero; missed-update time still
counts. Small completion-time jitter on successfully received updates is not
counted as a missing update.

`average_signal_loss_s` is the mean duration of continuous overdue-update
episodes, not the mean per missed packet. Overlapping or touching intervals
count as one episode; `signal_loss_episodes` reports their count. No loss gives
an average of zero. Loss still active when the run ends contributes its observed
duration, so that episode is truncated rather than extrapolated. The console
prints this average in milliseconds; JSON and CSV use seconds.

Every output filename uses the single UTC timestamp captured at startup. The
default root is `obj/noise_sim/`, already ignored by Git. Each run gets its own
UTC-timestamped subdirectory; existing runs are not overwritten. Outputs include
JSON and CSV summaries, event CSV, comparison PNG, a waveform PNG showing the
25 ms beginning 2 ms before the earliest detected signal error across all five
methods (clamped to zero), and a complete held-command timeline PNG. If no error
occurs, the waveform view begins at zero. Errors include rejected candidates,
missing updates, false commands, and any nonzero matched PWM width change,
including changes below the false-command tolerance. Error time is the receiver
detection time; a red vertical line marks the first error. All panels share the
same absolute time window. Compressed NPZ arrays of time, noise, and every
method's clean/noisy/filtered/binary waveform are now always saved at original
simulation precision. `--save-waveforms` remains accepted as a compatibility
flag but is no longer required. Full export keeps all arrays in memory and can
become large.

Each run also includes a Plotly `*_waveforms.html` page covering the entire
simulation, with all five methods on a shared time axis. Open it directly in a
current Chrome, Edge, Firefox, or Safari browser; no server or internet access
is needed. Wheel to zoom time, drag to pan time, scroll vertically between methods, use
the bottom range slider, or enter precise start/end times. The **Full run** and
**First error** buttons switch between the complete timeline and the existing
25 ms pre-error window. Clean/GPIO traces use steps at individual-sample zoom.
Voltage axes never zoom or rescale: all five panels share a fixed range covering
the full simulation, including noisy-voltage extrema.

The viewer loads compressed chunks from its adjacent `waveform_data/` folder
on demand. Keep that folder with the HTML when copying a run. Wide views show
minimum/maximum envelopes to preserve narrow spikes; zooming requests finer
levels down to every simulation sample. The status line identifies the active
resolution. Display voltage data uses float32 for compactness; the NPZ archive
retains original precision. The viewer caches a bounded number of chunks rather
than loading the entire simulation into the browser. Viewer assets duplicate
some waveform data and increase disk use, particularly at 100 ns resolution.

Fixed-RMS independent noise has a bandwidth set by the simulation step. Changing
resolution therefore changes the noise delivered through the filter even when
its physical time constant remains fixed. Compare methods within the same run;
do not interpret a resolution change as a controlled physical-noise experiment.
This model omits correlated motor interference, GPIO clamps, interrupt delays,
and missed CPU samples. Desk simulations cannot establish noise immunity of a
real ESC installation. Use multiple seeds and measured interference to guide
hardware testing.

The regression tests cross-check the Python state machine against the existing
production-C harness on Windows with clang/lld. Those cross-checks are skipped
elsewhere; the simulator itself does not require a C compiler.
