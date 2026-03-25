# Prompt for Claude — Refactor CSIRS_128T128R_v1_0.m

## Role
You are a Senior 5G L1 Protocol Engineer specializing in Massive MIMO CSI-RS simulation. You must strictly follow 3GPP Release 19 specifications and the technical decisions documented in PROJECT_CONTEXT_128T_CSIRS.md.

## Task
Refactor the file `CSIRS_128T128R_v1_0.m` to correctly simulate 128 CSI-RS ports across 2 consecutive slots, aligned with 3GPP Rel-19 MIMO Phase 5 (R1-2500098, Huawei, RAN1#120).

## Critical Constraints
- MATLAB R2025b with 5G Toolbox
- Digital beamforming only (all 128 ports transmit simultaneously)
- Must use existing MATLAB 5G Toolbox functions (`nrCSIRSConfig`, `nrCSIRS`, `nrCSIRSIndices`, `nrChannelEstimate`, `nrOFDMModulate`, `nrOFDMDemodulate`, `nrCDLChannel`)
- Do NOT create any new toolbox functions — only modify the simulation script

## Changes Required (in order of priority)

### 1. Switch from Row 17 to Row 18 (CDM8)
**Current (wrong):**
```matlab
csirs{resIdx}.RowNumber = 17;
csirs{resIdx}.SymbolLocations = {[symbolStarts(resIdx), symbolStarts(resIdx)+1]};
```
**Required:**
```matlab
csirs{resIdx}.RowNumber = 18;
csirs{resIdx}.SymbolLocations = {l0_value};  % Single scalar, CDM8 auto-expands to 4 symbols
```
- Row 18: 32 ports, CDM8 (FD-CDM2 × TD-CDM4), density 0.5
- SymbolLocations takes ONE value (l₀ only), toolbox derives l₀,l₀+1,l₀+2,l₀+3 automatically
- SubcarrierLocations for Row 18: verify against TS 38.211 Table 7.4.1.5.3-1 for Row 18 requirements
- Update `cdmLengths` from `[2 2]` to `[2 4]` for CDM8

### 2. Restructure to 2-Slot Simulation
**Current:** Single slot, 4 resources crammed into 14 symbols (causes overlap)
**Required:** 2 consecutive slots, 2 resources per slot

Layout:
```
Slot 0: R0 (l₀=2, ports 0-31) + R1 (l₀=8, ports 32-63)
Slot 1: R2 (l₀=2, ports 64-95) + R3 (l₀=8, ports 96-127)
```

Implementation approach — concatenate 2 slot grids, single OFDM modulate:
```matlab
% Slot 0
carrier.NSlot = 0;
grid_slot0 = nrResourceGrid(carrier, nTxAntennas);
% Map R0 and R1 symbols into grid_slot0 (with correct port offset)

% Slot 1
carrier.NSlot = 1;
grid_slot1 = nrResourceGrid(carrier, nTxAntennas);
% Map R2 and R3 symbols into grid_slot1 (with correct port offset)

% Concatenate along symbol dimension
txGrid_2slots = [grid_slot0  grid_slot1];  % size: [K × 28 × 128]

% Single OFDM modulation for both slots
[txWaveform, ofdmModInfo] = nrOFDMModulate(carrier, txGrid_2slots);
```

### 3. Channel: Single Object, Single Pass
**Critical:** Use ONE nrCDLChannel object. Call it ONCE with the full 2-slot waveform.

```matlab
channel = nrCDLChannel;
% ... configure once ...
channel.SampleRate = ofdmInfo.SampleRate;

txWaveform = [txWaveform; zeros(maxChDelay, nTxAntennas)];
[rxWaveform, pathGains, sampleTimes] = channel(txWaveform);
```

Do NOT:
- Create 2 channel objects
- Reset the channel between slots
- Use the same random seed for 2 separate channel calls
- Call `reset(channel)` between slots

The channel object is stateful — it advances internal time automatically, giving physically correct temporal correlation between slots.

### 4. OFDM Demodulation and Channel Estimation
After single demod of full waveform:
```matlab
rxGrid_2slots = nrOFDMDemodulate(carrier, rxWaveform);
```

Split back into per-slot grids for channel estimation:
```matlab
nSymPerSlot = carrier.SymbolsPerSlot;  % = 14
rxGrid_slot0 = rxGrid_2slots(:, 1:nSymPerSlot, :);
rxGrid_slot1 = rxGrid_2slots(:, (nSymPerSlot+1):(2*nSymPerSlot), :);
```

Channel estimation per resource, then combine:
- R0, R1: estimate from rxGrid_slot0
- R2, R3: estimate from rxGrid_slot1
- CDM lengths: `[2 4]` (not `[2 2]`)
- Combine into H_est_full of size [nSubcarriers × nSymbols × nRx × 128]

**Important for indices:** `nrCSIRSIndices` returns indices relative to a single-resource grid [K×L×32]. When mapping to the full 128-port grid, the port offset logic must account for the correct port range per resource. Keep the existing `ind2sub` → port shift → `sub2ind` approach but verify dimensions match Row 18.

### 5. Antenna Array — Align with Huawei R1-2500098
**Current:**
```matlab
gnbArraySize = [16, 4, 2, 1, 1];  % 16H × 4V × 2pol
```
**Required (option A — 1:1 port-element mapping):**
```matlab
gnbArraySize = [4, 16, 2, 1, 1];  % 4V × 16H × 2pol = 128
% This matches Huawei virtualized layout: (N1,N2) = (16,4)
```
Also update UE array if needed:
```matlab
ueArraySize = [1, 2, 2, 1, 1];  % 1V × 2H × 2pol = 4 antennas
```

### 6. Update Visualization (Section 4)
- The resource grid plot must show 2 slots (28 symbols total)
- X-axis: symbols 0-27 (or 0-13 for slot 0, 0-13 for slot 1)
- Color-code 4 resources across both slots
- Update occupancy bar chart for 28-symbol span

### 7. Update Channel Estimation Error Analysis (Section 9)
- `nrPerfectChannelEstimate` needs pathGains and pathFilters from the single channel call
- H_actual must cover the full 2-slot span
- Comparison between H_est_full and H_actual should account for the 2-slot time dimension

### 8. CSI Feedback Section (Section 10)
- Keep Approach A (per-resource PMI) as-is but note it gives independent 32-port PMIs
- Keep Approach B (SVD full 128-port) as upper-bound reference
- Update `reportConfig.PanelDimensions` to match new antenna layout:
  ```matlab
  reportConfig.PanelDimensions = [8, 2];  % N1=8, N2=2 per resource (32 ports)
  ```
- `myCSIReport` function: if it doesn't exist, replace calls with a stub that prints a warning, or implement a minimal wrapper around `nrPMISelect` + `nrCQISelect`

### 9. Beam Pattern Visualization (Section 11)
- Update to use correct array dimensions: nH=16 (horizontal), nV=4 (vertical), nPol=2
- Consider 2D beam pattern (azimuth × elevation) instead of azimuth-only cut

## What NOT to Change
- Overall 12-section structure of the file
- SNR configuration (20 dB)
- CDL-C channel model selection
- Carrier config: 52 PRBs, SCS 30 kHz
- Basic pipeline flow: Signal Gen → Channel → CE → CSI Feedback

## Verification Checklist
After refactoring, the simulation should:
- [ ] Generate 4 CSI-RS resources with Row 18, CDM8, 32 ports each
- [ ] Place R0+R1 in slot 0 (symbols 2-5 and 8-11), R2+R3 in slot 1 (same symbol positions)
- [ ] Produce a txGrid of size [624 × 28 × 128]
- [ ] Pass through CDL-C channel as a single continuous waveform
- [ ] Produce rxGrid of size [624 × 28 × 4] (4 Rx antennas)
- [ ] Estimate H_est_full of size [624 × 28 × 4 × 128]
- [ ] Print NMSE per port across all 128 ports
- [ ] Run without errors on MATLAB R2025b
