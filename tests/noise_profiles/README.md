# GPIO-referred motor interference profiles

These are illustrative stress cases, not measured interference from your robot.
Select a name with `python tests/noise_sim.py --profile NAME --seed 42`, or copy
a JSON file and pass its path. Every file has a description and all parameters.

| Name | Mechanism | Tune first |
|---|---|---|
| white-baseline | Original Gaussian noise, motor components disabled | Background RMS, receiver filter |
| steady-switching | PWM-edge spikes/ringing and electrical-cycle modulation | Spike amplitude, PWM frequency, modulation depth |
| weapon-startup | Strong initial switching interference and ground displacement | Startup multiplier and decay, ground startup voltage |
| ground-bounce-startup | Brief 2 V ground displacement, weak switching interference | Signed ground voltage and decay |
| long-ringing-startup | Strong 10 us ring tails overlapping neighboring edges | Ring amplitude, decay, frequency |
| fast-switching-startup | Weapon startup with 48 kHz switching | PWM frequency and duty |

Voltages describe what couples onto the GPIO, not motor phase voltage. Spike
amplitudes are triangular pulse peaks before startup/electrical modulation.
`ring_peak_v` is the initial exponential-envelope amplitude; damping means the
observed sinusoid peak is lower. Positive ground displacement is subtracted
from received voltage; negative displacement is added. Decay values are time
constants (not total event durations). Ground decay is independent of electrical
spin-up and switching-amplitude decay.

Motor profiles default to 100 ns resolution to resolve 1 us spikes and 200 kHz
ringing. White-baseline retains 1 us. No profile changes receiver thresholds,
filtering, or integrity checking unless its `simulation` group explicitly does
so. Explicit CLI options override profile simulation values. The default run
without a profile retains the original Gaussian implementation.

Custom profiles inherit omitted parameters from weapon-startup defaults.
Set spike/ring amplitudes to zero to disable them; set both ground voltages to
zero to disable ground shift. `simulation` can set duration, resolution, both
GPIO thresholds, receiver filter time constant, and PWM width tolerance.
`--noise-rms-v` overrides only background RMS, never total disturbance RMS.
