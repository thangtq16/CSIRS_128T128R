# 128T128R CSI-RS Simulation — Project Context & Technical Decisions

## Project Overview
- **Goal**: End-to-end Massive MIMO simulation for 128 CSI-RS ports, aligned with 3GPP Release 19 MIMO Phase 5 (RP-234007)
- **Author**: ThangTQ23 — Viettel Semiconductor Center (VSI)
- **MATLAB version**: R2025b
- **Current phase**: CSI-RS signal generation, channel estimation, CSI feedback
- **Future phases**: PDSCH scheduling → DM-RS → HARQ loop

## Architecture Decision: Digital Beamforming (NOT Hybrid)
- Focus on **Type-I/II codebook refinement** use case
- All 128 ports transmit simultaneously — gNB has full RF chain per port
- This differs from CRI-based CSI refinement (hybrid BF) where resources must be TDM across slots due to analog beam switching constraint

## 3GPP Rel-19 Reference: R1-2500098 (Huawei, RAN1#120, Feb 2025)
Key agreements from this document that drive the simulation design:

### Resource Aggregation
- 128 ports = **K=4 CSI-RS resources × 32 ports/resource**
- Alternative: K=8 × 16 ports (not chosen for this simulation)
- Max ports per single resource remains 32 (unchanged from Rel-15)

### Port Numbering (Rel-19 New)
After resource aggregation, P=128 ports are numbered per formula:
```
p_k = 3000 + (N/2)*(h_k + n2*v_k) + ... (see R1-2500098 Proposal 1)
```
Where h_k, v_k are horizontal/vertical indices of resource k in the antenna array.
- Code currently uses simple offset `(resIdx-1)*32` — needs update to Rel-19 formula

### Supported (Ks, max ports) Combinations
| K_S (resources) | Max ports/resource |
|---|---|
| 2, 3, 4 | 32 |
| 5, 6, 7, 8 | 16 |

## CSI-RS Resource Configuration Decisions

### Row Selection: Row 18 (CDM8)
**Chosen over Row 17 (CDM4)** for these reasons:
- Row 18: CDM8 = FD-CDM2 × TD-CDM4, needs only 1 SymbolLocation (l₀)
  - Toolbox auto-expands to 4 consecutive symbols: l₀, l₀+1, l₀+2, l₀+3
- Row 17: CDM4 = FD-CDM2 × TD-CDM2, needs 2 SymbolLocations (l₀, l₁)
  - Constraint l₁ ≥ l₀ + 2 was causing overlap issues in original code
- **Both rows occupy 4 OFDM symbols per resource** — Row 18 is simpler to configure

### Why 2 Slots Are Needed
- 4 resources × 4 symbols/resource = 16 symbols
- 1 slot = 14 symbols (normal CP)
- 16 > 14 → **must spread across 2 consecutive slots**
- This is consistent with 3GPP framework (multi-slot CSI-RS resource placement via periodicityAndOffset)

### 2-Slot Layout
```
Slot n (carrier.NSlot = 0):
  Resource #0 (ports 0-31):   l₀ = 2  → symbols 2,3,4,5
  Resource #1 (ports 32-63):  l₀ = 8  → symbols 8,9,10,11
  Free: symbols 0,1 (PDCCH), 6,7 (gap/DMRS), 12,13 (data)

Slot n+1 (carrier.NSlot = 1):
  Resource #2 (ports 64-95):  l₀ = 2  → symbols 2,3,4,5
  Resource #3 (ports 96-127): l₀ = 8  → symbols 8,9,10,11
  Free: symbols 0,1 (PDCCH), 6,7 (gap/DMRS), 12,13 (data)
```

### Density Consideration
- Current: all resources use `'dot5even'`
- Recommended improvement: stagger `'dot5even'` and `'dot5odd'` across resources for better frequency-domain coverage during channel estimation interpolation

## Channel Model Configuration

### CDL-C Parameters
```matlab
channel.DelayProfile         = 'CDL-C';
channel.DelaySpread          = 100e-9;       % 100ns (Urban Macro)
channel.MaximumDopplerShift  = 5;            % 5 Hz (low mobility)
channel.CarrierFrequency     = 3.5e9;        % n78
```

### Antenna Array (aligned with Huawei R1-2500098 Appendix A)
```matlab
% Huawei config: (M,N,P,Mg,Ng;Mp,Np) = (12,16,2,1,1,4,16)
% Physical: 12V × 16H × 2pol = 384 elements
% Virtualized: 4V × 16H × 2pol = 128 ports

% For 1:1 port-element simulation:
gnbArraySize = [4, 16, 2, 1, 1];   % [Nv, Nh, Npol, Mg, Ng]
% Or with virtualization:
gnbArraySize = [12, 16, 2, 1, 1];  % Physical array
% Virtualized to Mp=4, Np=16

ueArraySize = [1, 2, 2, 1, 1];     % UE: 4Rx (2H × 1V × 2pol)
```

### CRITICAL: Channel Continuity Across 2 Slots
- Use **ONE nrCDLChannel object** for both slots
- nrCDLChannel is a stateful System object with internal time counter
- Each call to `channel(waveform)` advances internal time by `length(waveform)/SampleRate`
- Path gains evolve continuously via Doppler model
- **DO NOT** create separate channel objects or reset seed between slots
- Coherence time ≈ 50ms, 2 slots = 1ms = 2% of T_c → highly correlated but not identical

## OFDM Modulation: Multi-Slot Support
`nrOFDMModulate` accepts grid with N > 14 symbols (multi-slot).
**Recommended approach**: concatenate 2 slot grids, modulate once, pass through channel once:

```matlab
% Build 2-slot grid
carrier.NSlot = 0;
grid_slot0 = nrResourceGrid(carrier, nTxAntennas);
% ... map R0, R1 ...

carrier.NSlot = 1;
grid_slot1 = nrResourceGrid(carrier, nTxAntennas);
% ... map R2, R3 ...

txGrid_2slots = [grid_slot0  grid_slot1];  % K × 28 × 128

% Single modulate → single channel pass → single demodulate
[txWaveform, ~] = nrOFDMModulate(carrier, txGrid_2slots);
txWaveform = [txWaveform; zeros(maxChDelay, nTxAntennas)];
[rxWaveform, pathGains, sampleTimes] = channel(txWaveform);
rxGrid_2slots = nrOFDMDemodulate(carrier, rxWaveform);
```

## Channel Estimation: Per-Resource Then Combine
```matlab
% Split rxGrid_2slots back into per-slot grids
rxGrid_slot0 = rxGrid_2slots(:, 1:14, :);
rxGrid_slot1 = rxGrid_2slots(:, 15:28, :);

% Estimate per resource, combine into H_est_full [K × L × nRx × 128]
% R0, R1 from rxGrid_slot0; R2, R3 from rxGrid_slot1
% CDM lengths for Row 18: [2 4] (FD-CDM2 × TD-CDM4)
cdmLengths = [2 4];
```

## CSI Feedback: Open Items
- `myCSIReport` function not yet implemented — needs Rel-19 Type-I codebook with extended (N1,N2) values
- Rel-19 introduces Scheme A and Scheme B for Type-I codebook:
  - Scheme A RI=1-4: extends legacy (N1,N2), limited 2nd beam selection
  - Scheme B RI=1-4: free beam + co-phasing selection, higher complexity
  - RI=5-8 support is new in Rel-19
- SVD-based approach (Approach B in current code) serves as upper-bound reference

## Known Issues in Current Code (CSIRS_128T128R_v1_0.m)
1. **SymbolLocations**: Row 17 config with 2-element vector — switch to Row 18 with single l₀
2. **Single-slot assumption**: Needs restructure to 2-slot simulation
3. **Port indexing**: Simple offset — needs Rel-19 p_k formula
4. **myCSIReport**: Missing implementation
5. **Beam pattern visualization**: Assumes ULA, ignores vertical dimension and cross-pol
6. **Timing sync**: Only uses Resource #0 — should combine all resources
7. **Title**: "128T128R" refers to gNB array size (128 TX/RX elements), UE is 4T4R — this is correct naming

## File Structure
- `CSIRS_128T128R_v1_0.m` — Main simulation script
- `5g/` — Custom 5G library path
- `wirelessnetwork/` — Custom wireless network library path
