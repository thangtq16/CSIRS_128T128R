%% Visualize CDL-B Channel Model — 128T4R (128T128R project)
%
%  Visualizes CDL-B channel characteristics for the 128T4R setup used in
%  CSIRS_128T128R_v2_0.m:
%
%    gNB (TX): [4V x 16H x 2pol x 1Mg x 1Ng] = 128 ports
%    UE  (RX): [1V x  2H x 2pol x 1Mg x 1Ng] =   4 ports
%
%  Channel parameters match the simulation:
%    - CDL-B, DS=100ns, fD=5Hz, fc=4.9GHz
%    - Element spacing: 0.5λ vertical/horizontal (standard URA)
%    - TX downtilt: 15 deg (typical macro gNB)
%
%  Reference: TS 38.901 Table 7.7.2-2 (CDL-B delay/power profile)

cdl = nrCDLChannel;
cdl.DelayProfile        = 'CDL-A';
cdl.DelaySpread         = 100e-9;      % 100 ns — Urban Micro / Indoor
cdl.MaximumDopplerShift = 5;           % 5 Hz — pedestrian @ 4.9 GHz
cdl.CarrierFrequency    = 4.9e9;       % 4.9 GHz

%% ── TX Array: gNB 128-port URA ──────────────────────────────────────────
%  Size = [M N P Mg Ng]: 4V × 16H × 2pol × 1Mg × 1Ng = 128 ports
gnbArraySize = [4, 16, 2, 1, 1];
cdl.TransmitAntennaArray.Size = gnbArraySize;

%  Element spacing: 0.5λ v/h, panel spacing = M*dv / N*dh (uniform URA)
lambda_v = 0.5;   % vertical element spacing (wavelengths)
lambda_h = 0.5;   % horizontal element spacing (wavelengths)
dg_v = lambda_v * gnbArraySize(1);   % = 0.5 * 4  = 2.0 λ (panel-to-panel vertical)
dg_h = lambda_h * gnbArraySize(2);   % = 0.5 * 16 = 8.0 λ (panel-to-panel horizontal)
cdl.TransmitAntennaArray.ElementSpacing = [lambda_v, lambda_h, dg_v, dg_h];

%  15 deg mechanical downtilt (standard macro gNB deployment)
cdl.TransmitArrayOrientation = [0; 15; 0];   % [bearing; downtilt; slant] degrees

%% ── RX Array: UE 4-port ─────────────────────────────────────────────────
%  Size = [M N P Mg Ng]: 1V × 2H × 2pol × 1Mg × 1Ng = 4 ports
ueArraySize = [1, 2, 2, 1, 1];
cdl.ReceiveAntennaArray.Size = ueArraySize;
cdl.ReceiveAntennaArray.ElementSpacing = [0.5, 0.5, 0.5, 0.5];

%% ── Display TX (gNB 128-port) ───────────────────────────────────────────
fprintf('=== %s Channel Visualization — 128T4R ===\n', cdl.DelayProfile);
fprintf('TX: gNB  %d ports  [%dV x %dH x %dpol]\n', ...
    prod(gnbArraySize), gnbArraySize(1), gnbArraySize(2), gnbArraySize(3));
fprintf('RX: UE   %d ports  [%dV x %dH x %dpol]\n', ...
    prod(ueArraySize), ueArraySize(1), ueArraySize(2), ueArraySize(3));
fprintf('Channel: %s, DS=%.0fns, fD=%.0fHz, fc=%.1fGHz\n\n', ...
    cdl.DelayProfile, cdl.DelaySpread*1e9, cdl.MaximumDopplerShift, cdl.CarrierFrequency/1e9);

figTx = displayChannel(cdl, 'LinkEnd', 'Tx');
figTx.Name = sprintf('%s — TX (gNB 128-port URA)', cdl.DelayProfile);
datacursormode on;

%% ── Display RX (UE 4-port) ──────────────────────────────────────────────
figRx = displayChannel(cdl, 'LinkEnd', 'Rx');
figRx.Name = sprintf('%s — RX (UE 4-port)', cdl.DelayProfile);
datacursormode on;
