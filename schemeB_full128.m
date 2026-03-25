%% schemeB_full128.m — Full 128-Port Type-I DFT Codebook (Scheme-B concept)
%
%  Implements codebook-constrained RI/PMI/CQI selection for the full
%  128-port channel using a DFT beam grid, following the Scheme-B
%  concept from R1-2500098 (Huawei, RAN1#120, Feb 2025).
%
%  Run AFTER CSIRS_128T128R_v1_0.m (uses H_wb, nVar_avg from workspace).
%
%  Panel geometry (aligned with channel config in CSIRS_128T128R_v1_0.m):
%    gnbArraySize = [Nv=4, Nh=16, Npol=2, 1, 1]  -> 128 ports
%    3GPP notation: N1=16 (horizontal), N2=4 (vertical)
%
%  MATLAB nrCDLChannel port ordering for [Nv, Nh, Npol]:
%    port_0based = npol*Nv*Nh + nh*Nv + nv   (nv varies fastest)
%    pol-0 ports: nh*Nv + nv  (indices 0..63)
%    pol-1 ports: 64 + nh*Nv + nv            (indices 64..127)
%
%  DFT beam for this ordering:
%    beam = kron(u_m1, u_m2)  where
%      u_m1 = Nh-element DFT vector (outer, nh index)
%      u_m2 = Nv-element DFT vector (inner, nv index)
%    beam[nh*Nv + nv] = u_m1[nh] * u_m2[nv]  ✓

fprintf('\n========================================\n');
fprintf(' Scheme-B: Full 128-Port DFT Codebook\n');
fprintf('========================================\n\n');

%% --- Panel and codebook parameters ---
Nh   = 16;   % Horizontal elements  (N1 in 3GPP)
Nv   = 4;    % Vertical elements    (N2 in 3GPP)
Npol = 2;    % Dual-polarization
assert(Nh*Nv*Npol == nTxAntennas, ...
    'Panel config (%d*%d*%d=%d) must match nTxAntennas (%d)', ...
    Nh, Nv, Npol, Nh*Nv*Npol, nTxAntennas);

O1 = 4;   % Oversampling horizontal (standard per TS 38.214)
O2 = 4;   % Oversampling vertical

nBeamH = O1 * Nh;   % = 64 horizontal beam directions
nBeamV = O2 * Nv;   % = 16 vertical beam directions
nBeams = nBeamH * nBeamV;  % = 1024 total beam candidates (single pol)

fprintf('Panel:    N1=%d (horiz) x N2=%d (vert) x %dpol = %d ports\n', ...
    Nh, Nv, Npol, nTxAntennas);
fprintf('Codebook: O1=%d, O2=%d → %d x %d = %d candidate beams\n\n', ...
    O1, O2, nBeamH, nBeamV, nBeams);

%% --- Build DFT codebook ---
% u_m1: Nh-element DFT vector for horizontal beam index m1
u_h = @(m1) exp(1j*2*pi*(0:Nh-1).'*m1/(O1*Nh)) / sqrt(Nh);
% u_m2: Nv-element DFT vector for vertical beam index m2
u_v = @(m2) exp(1j*2*pi*(0:Nv-1).'*m2/(O2*Nv)) / sqrt(Nv);

% B: single-pol codebook matrix [Nh*Nv=64 x nBeams=1024]
B = zeros(Nh*Nv, nBeams);
bi = 1;
for m1 = 0:(nBeamH - 1)
    for m2 = 0:(nBeamV - 1)
        % kron(u_h, u_v): outer=horizontal (nh), inner=vertical (nv)
        % -> B[nh*Nv + nv, bi] = u_h[nh] * u_v[nv]  aligns with MATLAB port ordering
        B(:, bi) = kron(u_h(m1), u_v(m2));
        bi = bi + 1;
    end
end

% Co-phasing factors phi (dual-pol, 3GPP Table 5.2.2.2.1-5)
co_phases = [1, 1j, -1, -1j];

%% --- Extract wideband single-pol channels ---
% H_wb [nRx=4 x nTx=128], pol-0: columns 1..64, pol-1: columns 65..128
H_p0 = H_wb(:, 1:Nh*Nv);        % [4 x 64]  pol-0
H_p1 = H_wb(:, Nh*Nv+1:end);    % [4 x 64]  pol-1

%% --- Beam gain per candidate (sum over both pols) ---
gain = sum(abs(H_p0 * B).^2, 1) + sum(abs(H_p1 * B).^2, 1);  % [1 x nBeams]
[~, sortedBeams] = sort(gain, 'descend');

%% --- Greedy orthogonal beam selection (Scheme-B constraint) ---
%
%  Scheme-B: each additional beam must be near-orthogonal to already
%  selected beams (inner product < threshold). This is the "free beam
%  selection with orthogonality constraint" from R1-2500098 Proposal 3.
%
maxLayers = min(nRxAntennas, 8);
orth_thresh = 0.3;   % Inner product threshold (adjustable)

sel_idx  = zeros(1, maxLayers);   % Beam indices (into B columns)
sel_beams = zeros(Nh*Nv, maxLayers);
nSel = 0;

for bi_cand = 1:nBeams
    w = B(:, sortedBeams(bi_cand));
    if nSel == 0
        max_corr = 0;
    else
        max_corr = max(abs(sel_beams(:, 1:nSel)' * w));
    end
    if max_corr < orth_thresh
        nSel = nSel + 1;
        sel_idx(nSel)      = sortedBeams(bi_cand);
        sel_beams(:, nSel) = w;
    end
    if nSel >= maxLayers
        break;
    end
end
sel_beams = sel_beams(:, 1:nSel);

fprintf('Greedy beam selection (orth_thresh=%.2f):\n', orth_thresh);
fprintf('  Candidate beams scanned: %d / %d\n', ...
    find(sortedBeams == sel_idx(nSel)), nBeams);
fprintf('  Beams selected: %d\n\n', nSel);

%% --- RI and co-phasing selection via capacity ---
%
%  Per-layer independent co-phasing (standard Type-I codebook):
%    Each layer l uses its own phi_l independently.
%    W_full[:,l] = [b_l; phi_l * b_l] / sqrt(2)   for l=1..ri
%
%  Search: for each RI, exhaustively search all phi combinations.
%  4^ri_t combinations total per RI (manageable for ri_t <= 4).
%
best_cap     = zeros(1, nSel);
best_phi_set = ones(nSel, nSel);   % best_phi_set(ri_t, :) stores phi indices for ri_t layers

for ri_t = 1:nSel
    cap_best     = -Inf;
    phi_best_row = ones(1, ri_t);
    % Generate all 4^ri_t combinations
    phi_combos = dec2base(0:(4^ri_t - 1), 4, ri_t) - '0' + 1;  % [4^ri_t x ri_t], values 1..4
    for ci = 1:size(phi_combos, 1)
        % Build precoder column by column with per-layer co-phase
        W_t = zeros(nTxAntennas, ri_t);
        for l = 1:ri_t
            phi_l = co_phases(phi_combos(ci, l));
            W_t(:, l) = [sel_beams(:, l); phi_l * sel_beams(:, l)] / sqrt(2);
        end
        H_e = H_wb * W_t;
        SNR_per_layer = 1 / (nVar_avg * ri_t);
        cap = real(log2(det(eye(nRxAntennas) + SNR_per_layer * (H_e * H_e'))));
        if cap > cap_best
            cap_best = cap;
            phi_best_row = phi_combos(ci, :);
        end
    end
    best_cap(ri_t) = cap_best;
    best_phi_set(ri_t, 1:ri_t) = phi_best_row;
end

% Select RI that maximizes capacity
[cap_max, ri_B] = max(best_cap);

%% --- Build final precoder (per-layer independent co-phasing) ---
W_B = zeros(nTxAntennas, ri_B);
phi_str = '';
for l = 1:ri_B
    phi_l = co_phases(best_phi_set(ri_B, l));
    W_B(:, l) = [sel_beams(:, l); phi_l * sel_beams(:, l)] / sqrt(2);
    phi_str = [phi_str, sprintf('%.0f%+.0fj ', real(phi_l), imag(phi_l))]; %#ok<AGROW>
end

%% --- Per-layer SINR and CQI (ZF receiver) ---
H_eff_B = H_wb * W_B;                  % [nRx x ri_B]
W_zf_B  = pinv(H_eff_B);               % [ri_B x nRx]

sinr_B = zeros(ri_B, 1);
for l = 1:ri_B
    sig  = abs(W_zf_B(l,:) * H_eff_B(:,l))^2;
    intf = sum(abs(W_zf_B(l,:) * H_eff_B).^2) - sig;
    sinr_B(l) = sig / (intf + nVar_avg * norm(W_zf_B(l,:))^2);
end
sinr_B_dB = 10*log10(sinr_B);

% CQI mapping (TS 38.214 Table 5.2.2.1-2, 64-QAM set)
cqi_tbl = [-6.7, -4.7, -2.3, 0.2, 2.4, 4.7, 6.9, 9.3, ...
           10.7, 12.2, 14.1, 15.6, 18.0, 20.3, 22.7];
cqi_B = arrayfun(@(s) max(sum(s >= cqi_tbl), 1), sinr_B_dB);

%% --- Compare Approach A vs Approach B vs Scheme-B ---
fprintf('=== Results Comparison ===\n\n');

fprintf('Approach A (per-resource, 32p each, Type-I [N1=8,N2=2]):\n');
fprintf('  RI per resource:  [%s]\n', num2str(ri_per_res, '%d '));
fprintf('  Min RI:           %d\n', min(ri_per_res));
fprintf('  Avg CQI:          %.1f\n\n', mean(cqi_per_res));

fprintf('Approach B (full 128p, unconstrained SVD):\n');
fprintf('  RI (SVD):         %d\n', ri_svd);
fprintf('  Singular values:  [%s]\n', num2str(singularValues(1:min(8,end)).', '%.2f '));
fprintf('  Per-layer SINR:   [%s] dB\n', num2str(sinr_dB.', '%.1f '));
fprintf('  Per-layer CQI:    [%s]\n\n', num2str(cqi_svd.', '%d '));

fprintf('Scheme-B (full 128p, DFT codebook [N1=%d,N2=%d], O1=%d,O2=%d):\n', ...
    Nh, Nv, O1, O2);
fprintf('  Beams available:  %d\n', nBeams);
fprintf('  Beams selected:   %d (orth_thresh=%.2f)\n', nSel, orth_thresh);
fprintf('  RI selected:      %d\n', ri_B);
fprintf('  Co-phase per-layer: [%s]\n', phi_str);
fprintf('  Per-layer SINR:   [%s] dB\n', num2str(sinr_B_dB.', '%.1f '));
fprintf('  Per-layer CQI:    [%s]\n', num2str(cqi_B.', '%d '));
fprintf('  Capacity:         %.2f bits/s/Hz (RI=%d)\n', cap_max, ri_B);
fprintf('  Capacity vs RI:   [%s]\n', num2str(best_cap, '%.2f '));

%% --- Codebook gain vs unconstrained SVD ---
% Upper bound: SVD capacity
H_wb_full = H_wb;
W_svd_full = V(:, 1:ri_svd);
H_eff_svd  = H_wb_full * W_svd_full;
snr_svd    = 1 / (nVar_avg * ri_svd);
cap_svd    = real(log2(det(eye(nRxAntennas) + snr_svd * (H_eff_svd * H_eff_svd'))));

fprintf('\nCodebook efficiency:\n');
fprintf('  SVD capacity (upper bound): %.2f bits/s/Hz\n', cap_svd);
fprintf('  Scheme-B capacity:          %.2f bits/s/Hz\n', cap_max);
fprintf('  Codebook loss:              %.2f dB (%.1f%%)\n', ...
    10*log10(cap_svd/cap_max), (1 - cap_max/cap_svd)*100);

%% --- Visualization ---
figure('Name', 'Scheme-B: Full 128-Port DFT Codebook', ...
    'Position', [100 100 1400 500]);

% (1) Capacity vs RI
subplot(1,3,1);
bar(1:nSel, best_cap);
hold on;
plot([ri_B ri_B], [0 max(best_cap)*1.1], 'r--', 'LineWidth', 1.5);
xlabel('RI (Number of Layers)');
ylabel('Capacity (bits/s/Hz)');
title('Capacity vs RI — Scheme-B');
legend('Capacity', sprintf('Selected RI=%d', ri_B));
grid on;

% (2) Beam gain pattern (horizontal cut: best beam per vertical index)
subplot(1,3,2);
gain_map = reshape(gain, nBeamV, nBeamH);   % [nBeamV x nBeamH]
gain_map_dB = 10*log10(gain_map / max(gain(:)));
imagesc(0:nBeamH-1, 0:nBeamV-1, gain_map_dB);
colorbar; colormap(gca, 'jet');
xlabel('Horizontal beam index m1'); ylabel('Vertical beam index m2');
title('Beam Gain Map (dB, normalized)');
clim([-30 0]);

% (3) Per-layer SINR comparison
subplot(1,3,3);
nL_svd = length(sinr_dB);
nL_B   = length(sinr_B_dB);
x_svd  = (1:nL_svd) - 0.15;
x_B    = (1:max(nL_svd,nL_B)) + 0.15;
bar(x_svd, sinr_dB, 0.3, 'FaceColor', [0.2 0.6 1]);
hold on;
bar((1:nL_B)+0.15, sinr_B_dB, 0.3, 'FaceColor', [1 0.5 0.2]);
xlabel('Layer'); ylabel('SINR (dB)');
title('Per-Layer SINR: SVD vs Scheme-B');
legend('SVD (unconstrained)', 'Scheme-B (DFT codebook)');
grid on;

sgtitle(sprintf('Scheme-B Full 128-Port Analysis (SNR=%ddB, CDL-C)', SNRdB));
