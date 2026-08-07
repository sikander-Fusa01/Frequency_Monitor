# Frequency Monitor IP (Verilog)

A  multi-clock-domain **Frequency Monitor** peripheral, implemented
from a functional micro-architecture specification. It measures and
continuously monitors the frequency of up to `NUM_CHANNELS` asynchronous
input clocks and exposes configuration/status/results through an **APB4**
register interface, with an interrupt output.

## Features

- **On-demand Measurement Mode** — single-shot or continuous frequency
  measurement of one selected channel via a programmable gate time.
- **Continuous Monitor Mode** — round-robin scheduler sweeps all enabled
  channels, comparing each against per-channel min/max thresholds and
  raising **Underflow / Overflow / Loss-of-Clock / Recovered** faults.
- Fully **synchronized** across three clock domains (`clk_in[]`, `ref_clk`,
  `bus_clk`) using generic level (`sync2ff`) and toggle-based pulse
  (`pulse_sync`) CDC primitives.
- Zero-wait-state **APB4** slave with complete address decode, RW1C sticky
  fault/IRQ bits, self-clearing START/ABORT bits, and configurable
  edge/level interrupt behavior.
- Parameterizable: `NUM_CHANNELS`, `REF_CLK_FREQ_HZ`, `SYNC_STAGES`,
  `RESULT_WIDTH`, `THRESH_WIDTH`, `ADDR_WIDTH`.

## Repository layout

```
freq_monitor/
├── rtl/                     Synthesizable RTL (pure Verilog-2001)
│   ├── sync2ff.v            Generic multi-stage level synchronizer
│   ├── pulse_sync.v         Toggle-based single-pulse CDC synchronizer
│   ├── ch_input_sync_edge.v Per-channel input sync + edge detector (ICS+ED)
│   ├── measurement_engine.v Measurement Engine (ME) FSM
│   ├── monitor_comparator.v Monitor Comparator (MC) round-robin FSM
│   ├── interrupt_gen.v      Interrupt Generator (IG) FSM
│   ├── apb_regs.v           APB4 register interface + fault/status logic
│   └── freq_monitor_top.v   Top-level integration
├── tb/
│   ├── tb_freq_monitor.v        Basic smoke-test bench
│   └── tb_freq_monitor_full.v   Full self-checking regression (81 checks)
├── docs/                    (place your spec / register-map docs here)
├── LICENSE
└── README.md
```


Both testbenches print a `PASS`/`FAIL` verdict per check plus an
EXPECTED-vs-ACTUAL comparison line, and a final scoreboard, e.g.:

```
[TEST 68] CH3 OVERFLOW sticky set        EXPECTED=0x00000001 ACTUAL=0x00000001  ==> PASS
...
TOTAL TESTS : 81
PASSED      : 81
FAILED      : 0

*** OVERALL RESULT : ALL TESTS PASSED ***
```

## Register map summary

| Offset  | Register            | Access | Notes                              |
|---------|----------------------|--------|-------------------------------------|
| 0x00    | CTRL                 | R/W    | GLOBAL_EN, MON_MODE, IRQ_GLOBAL_EN, IRQ_EDGE, MEASURE_CONTINUOUS |
| 0x04    | STATUS               | R      | MON_ACTIVE, MEASURE_BUSY, MEASURE_DONE, REF_CLK_OK, ANY_FAULT |
| 0x08    | IRQ_EN               | R/W    | Per-source interrupt enables        |
| 0x0C    | IRQ_STATUS           | RW1C   | Per-source pending interrupt flags  |
| 0x10    | IRQ_CLR              | WO     | Write-1 alternate clear path        |
| 0x14    | MON_PERIOD           | R/W    | Monitor gate time (ref_clk cycles)  |
| 0x18    | MEASURE_SEL          | R/W    | Channel selected for Measurement Mode |
| 0x1C    | MEASURE_GATE         | R/W    | Measurement gate time (ref_clk cycles) |
| 0x20    | MEASURE_CTRL         | mixed  | START/ABORT (self-clear WO), DONE/RESULT_VALID (RO) |
| 0x24    | MEASURE_RESULT       | R      | Raw edge count for last measurement |
| 0x28    | REVISION             | R      | Fixed 0x0001_0000                   |
| 0x2C    | NUM_CHANNELS         | R      | Number of implemented channels      |
| 0x100+i*0x10 | CH[i]_MON_MIN/MAX/LAST_RESULT/FAULT | mixed | Per-channel monitor config/status |
| 0x200   | MON_ENABLE           | R/W    | Per-channel monitor scan enable     |
| 0x204   | FAULT_SUMMARY        | R      | Live OR of all fault_active bits    |
| 0x208   | LOC_SUMMARY          | RW1C   | Sticky OR of Loss-of-Clock events   |
| 0x20C   | UNDERFLOW_SUMMARY    | RW1C   | Sticky OR of Underflow events       |
| 0x210   | OVERFLOW_SUMMARY     | RW1C   | Sticky OR of Overflow events        |
| 0x214   | RECOVERED_SUMMARY    | RW1C   | Sticky OR of Recovered events       |
| 0x218   | MON_CHANNEL_ACTIVE   | R      | Currently-scanned channel (0x1F = idle) |

## Known design decisions / limitations

- The Measurement Engine and Monitor Comparator use **separate** edge/gate
  counters rather than a single time-multiplexed counter, favoring clarity
  and robustness over area.
- Sticky (RW1C) fault/IRQ bits live entirely in the `bus_clk` domain, set by
  synchronized single-cycle event pulses from `ref_clk`.
- Multi-bit configuration buses (gate/period/thresholds/etc.) use simple
  2-flop synchronizers; software should hold configuration stable while the
  corresponding engine (`GLOBAL_EN` / `MON_MODE`) is active.
- `ref_clk` must run **substantially faster** than the fastest input clock
  being measured (a Nyquist-type oversampling requirement) — see the
  testbench comments for a worked example of a bug caused by violating this.

## License

MIT — see [LICENSE](LICENSE).
