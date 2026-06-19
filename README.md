# AXI4 2×1 Bus System (SystemVerilog)

A compact, self-checking **AXI4** example system in SystemVerilog: **two masters** share a single **slave** through a **2:1 round-robin interconnect**, with a self-checking testbench that drives traffic and verifies protocol behavior end-to-end.

It is meant as a readable, educational reference for how the five AXI4 channels, multiple-outstanding transactions, ID routing, and in-order responses fit together — not as a production crossbar.

---

## Contents

| File | Role |
|------|------|
| `tb_axi_bus_system.sv` | Top-level testbench: clock/reset, DUT instances, scoreboard, protocol monitors, watchdog. |
| `simple_axi_master.sv` | Behavioral AXI4 master — fixed pacing, multiple outstanding, in-order self-checker. |
| `realistic_axi_master.sv` | Behavioral AXI4 master — DMA-style command queues + buffered write data with randomized (but reproducible) pacing; same in-order self-checker. |
| `axi4_simple_2to1_bus_arbiter.sv` | 2-master / 1-slave round-robin interconnect with ID routing and order checking. |
| `simple_axi_slave.sv` | In-order AXI4 slave: returns `OKAY`, synthetic read data, optional response latency. |

The two masters are interchangeable (identical port lists). The testbench wires **master 0 = `simple_axi_master`** and **master 1 = `realistic_axi_master`**.

---

## System architecture

```
        ┌────────────────┐                                              ┌────────────────┐
        │   master 0     │                                              │                │
        │ simple_axi_... │                                              │                │
        └────────────────┘        ┌────────────────────────┐            │                │
                    AW/W/AR  ──►  │  2:1 round-robin       │──► AW/W/AR │   slave        │
                     B/R     ◄──  │  interconnect          │ ◄── B/R    │  (in-order     │
                                  │  • RR arbitration      │            │   responder)   │
        ┌────────────────┐        │  • ID widen {route,id} │            │                │
        │   master 1     │        |  • W routes by AW      │            │                │
        │ realistic_axi..│        |  • wr_q / rd_q order   │            │                │
        └────────────────┘        └────────────────────────┘            └────────────────┘
          ID_WIDTH bits                 ID_WIDTH+1 bits                   ID_WIDTH+1 bits
                                   (route bit prepended)
```

- **Masters** issue independent write (AW/W/B) and read (AR/R) transactions and self-check every response.
- The **interconnect** arbitrates between the two masters, tags each accepted transaction with a 1-bit **route** field (`S_*ID = {master_select, master_ID}`), forwards it to the slave, then routes the response (`B`/`R`) back to the originating master by that bit.
- The **slave** accepts commands in order, returns `OKAY` with deterministic read data, and re-echoes the (route-tagged) ID so the interconnect can route responses home.

---

## The five AXI4 channels

| Channel | Direction | Carries |
|---------|-----------|---------|
| **AW** — write address | master → slave | `AWID, AWADDR, AWLEN` + handshake |
| **W** — write data | master → slave | `WDATA, WLAST` + handshake (no `WID` in AXI4) |
| **B** — write response | slave → master | `BID, BRESP` |
| **AR** — read address | master → slave | `ARID, ARADDR, ARLEN` |
| **R** — read data | slave → master | `RID, RDATA, RLAST, RRESP` |

Every channel uses the AXI `VALID`/`READY` handshake: the payload transfers on the cycle both are high, and `VALID` must stay asserted with stable payload until `READY` arrives.

---

## Data flow

### Write transaction
1. **AW** — master presents `{AWID, AWADDR, AWLEN}`. The arbiter grants one master, prepends the route bit, and forwards it to the slave; it records the accepted command in an order FIFO (`wr_q`).
2. **W** — master streams `WDATA` beats, last beat marked `WLAST`. Because AXI4 has **no `WID`**, write data is bound to its address purely by **order** — the interconnect forwards W in AW-acceptance order.
3. **B** — after the burst, the slave returns `{BID, BRESP=OKAY}`. The interconnect routes it back to the originating master by the route bit and checks it came back in order.

### Read transaction
1. **AR** — master presents `{ARID, ARADDR, ARLEN}`; arbiter grants, route-tags, forwards, records in `rd_q`.
2. **R** — slave returns `RLEN+1` beats of `{RID, RDATA, RRESP}`, last beat marked `RLAST`. The interconnect routes each beat back to the originating master.

### Multiple outstanding
Masters do **not** wait for `B`/`R` before issuing the next `AW`/`AR`, so several transactions are in flight at once. The interconnect and slave bound this with per-channel outstanding limits (`MAX_OUTSTANDING_TRANSACTIONS`).

---

## Ordering model (important)

The whole system is **in-order**:
- Write responses (`B`) return in AW-acceptance order; read data (`R`) returns in AR-acceptance order.
- Within the interconnect, `W` follows AW order and `B`/`R` order is checked against the recorded `wr_q`/`rd_q`.

This is a legal AXI4 subset. It keeps the example small and lets the masters self-check with simple in-order expectations. **Out-of-order completion across IDs is not modeled.**

---

## The two masters

Both have identical ports and the same in-order self-checkers (they verify `BID`/`RID`, `BRESP`/`RRESP == OKAY`, and `WLAST`/`RLAST` placement, raising `protocol_error_*` outputs). They differ only in **how traffic is paced**:

- **`simple_axi_master`** — issues `NUM_TRANSACTIONS` fixed-length bursts with deterministic, optionally-spaced pacing (`ISSUE_GAP`, `START_DELAY`). Compact and easy to follow.
- **`realistic_axi_master`** — modeled like a DMA/CPU front-end: a **request generator** pushes commands into per-direction **command queues** over time; channel engines drain them; write data is staged in a **write-data FIFO** that lags the request by a fixed fill latency and randomized per-beat pacing; `BREADY`/`RREADY` apply randomized backpressure. All randomness comes from seeded **LFSRs**, so runs are **reproducible** (no `$urandom`).

Because content and order are identical, the two are drop-in interchangeable and the testbench checks the same way for either.

---

## Key parameters

Set at the top of `tb_axi_bus_system.sv`:

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `ID_WIDTH` | 4 | Master-side AXI ID width (slave side is `ID_WIDTH+1` with the route bit). |
| `ADDR_WIDTH` | 32 | Address bus width. |
| `DATA_WIDTH` | 128 | Data bus width (16 bytes/beat). |
| `NUM_TRANSACTIONS` | 16 | Write + read transactions issued **per master**. |
| `MAX_OUTSTANDING_TRANSACTIONS` | 16 | Per-channel outstanding depth in the interconnect/slave. |
| `BURST_BEATS` | 4 | Beats per burst (`AxLEN = BURST_BEATS-1`). |
| `DEBUG` | 1 | Enable per-transaction trace prints. |
| `DEMO` | 1 | Space out traffic for a readable trace (0 = back-to-back stress). |
| `SLAVE_RESP_LATENCY` | 3 (demo) | Cycles the slave waits before `B` / first `R` beat. |

---

## Running the simulation

### Vivado (xsim)
```sh
xvlog -sv simple_axi_master.sv realistic_axi_master.sv \
          axi4_simple_2to1_bus_arbiter.sv simple_axi_slave.sv \
          tb_axi_bus_system.sv
xelab tb_axi_bus_system -s tb_sim
xsim  tb_sim -runall
```
Or add the files to a Vivado project, set `tb_axi_bus_system` as the simulation top, and **Run All** (the run exceeds the default 1000 ns window, so use *Run All*, not *Run*).
A VCD waveform `wave_rr_axi4_id_outstanding.vcd` is dumped by default.

### Expected result
On success the testbench prints:
```
---- PASS: AXI4 ID + multiple outstanding scenario completed ----
```
and `$finish`es. Any protocol violation triggers a named `$fatal`; a stalled run is caught by the watchdog.

---

## What the testbench checks

1. **Throughput / outstanding** — both masters issue all transactions without waiting; the slave must see `2 × NUM_TRANSACTIONS` of each command.
2. **ID routing & order** — each master gets back its own `BID`/`RID`, in order, with the route bit correctly added and stripped.
3. **Bursts & responses** — `WLAST`/`RLAST` verified at every hop; `BRESP`/`RRESP` must be `OKAY`.
4. **Handshake stability** — payload must not change while `VALID` is high and `READY` is low.
5. **Liveness** — a workload-scaled watchdog `$fatal`s if the run never completes.

---

## Debug trace legend

With `DEBUG = 1`, each hop prints one line per transaction event:

```
[time] SRC CH dir txn=M<route>.<W|R><id>  <fields>
  SRC = M0 | M1 (master), ARB (arbiter), SLV (slave)
  CH  = AW / W / B (write path),  AR / R (read path)
  dir = >> request (master→slave),  << response (slave→master)
  txn = M<route>.<W|R><id>  (same key on every hop)
```
The `txn=` key is identical across master, arbiter, and slave, so a single transaction is traceable end-to-end, e.g. `grep 'txn=M0.W2'` (a write) or `grep 'txn=M1.R5'` (a read).

---

## Scope & limitations

- **In-order only** — no out-of-order completion across IDs.
- **Simulation models** — behavioral masters/slave/arbiter for verification and learning, not a synthesizable production interconnect.
- **No `WSTRB` / narrow / unaligned / WRAP bursts** — full-width `INCR` bursts only.
- **`BRESP`/`RRESP`** are always `OKAY` from this slave (error responses are decoded/printed but not generated).

---


