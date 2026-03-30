function fb = csirs_feedback(carrier, csirs, H_est_full, nVar_all, slotAssign)
%CSIRS_FEEDBACK  CSI feedback (RI/PMI/CQI/Cap) for Approaches B, C, D.
%
%  Approach B: Full 128-port SVD upper bound (ideal reference)
%  Approach C: Rel-19 Type I Single-Panel Mode A (TS 38.214 S5.2.2.2.1a)
%  Approach D: Rel-19 Type I Single-Panel Mode B (TS 38.214 S5.2.2.2.1a)
%
%  Inputs:
%    carrier      - nrCarrierConfig (NSlot will be modified internally)
%    csirs        - {1x4} nrCSIRSConfig objects (csirs{1} used for C/D)
%    H_est_full   - [K x 28 x nRx x 128] estimated channel
%    nVar_all     - [1x4] noise variance per resource
%    slotAssign   - [1x4] slot index per resource
%
%  Output:
%    fb - struct with fields:
%           .ri_B,  .cqi_B,  .cap_B,  .W_B   (SVD reference)
%           .ri_C,  .cqi_C,  .cap_C,  .W_C   (Mode A)
%           .ri_D,  .cqi_D,  .cap_D,  .W_D   (Mode B)
%         cqi_* is a per-layer vector; cap_* is scalar (bits/s/Hz)

nRxAntennas  = size(H_est_full, 3);
nTxAntennas  = size(H_est_full, 4);
nPortsPerRes = nTxAntennas / 4;
nSymPerSlot  = carrier.SymbolsPerSlot;

% ── Wideband channel matrix H_wb [nRx x nTx] ─────────────────────────────
% Average over each resource's own slot to avoid cross-slot dilution
H_wb = zeros(nRxAntennas, nTxAntennas);
for r = 1:4
    pS = (r-1)*nPortsPerRes + 1;  pE = r*nPortsPerRes;
    sS = slotAssign(r)*nSymPerSlot + 1;
    sE = (slotAssign(r)+1)*nSymPerSlot;
    H_wb(:, pS:pE) = squeeze(mean(H_est_full(:, sS:sE, :, pS:pE), [1 2]));
end

nVar_wb  = mean(nVar_all);
cqi_tbl  = [-6.7,-4.7,-2.3,0.2,2.4,4.7,6.9,9.3,10.7,12.2,14.1,15.6,18.0,20.3,22.7];
ri_max   = min(nRxAntennas, 4);

% Broadcast H_wb to [K x L x nRx x nTx] for nrPMIReport
carrier.NSlot = 0;
nSC  = carrier.NSizeGrid * 12;
nSym = carrier.SymbolsPerSlot;
H_4d = repmat(reshape(H_wb, 1, 1, nRxAntennas, nTxAntennas), [nSC nSym 1 1]);

% =========================================================================
%  APPROACH B: Full 128-Port SVD Upper Bound
% =========================================================================
[~, S_B, V_B]    = svd(H_wb, 'econ');
singVals_B       = diag(S_B);
snr_sv_B         = singVals_B.^2 / nVar_wb;
riThresh_B       = 0.1 * singVals_B(1);
ri_B             = min(sum(singVals_B > riThresh_B), nRxAntennas);
W_B              = V_B(:, 1:ri_B);

H_eff_B   = H_wb * W_B;
W_zf_B    = pinv(H_eff_B);
sinr_B    = zeros(ri_B, 1);
for lyr = 1:ri_B
    sig  = abs(W_zf_B(lyr,:) * H_eff_B(:,lyr))^2;
    intf = sum(abs(W_zf_B(lyr,:) * H_eff_B).^2) - sig;
    nse  = nVar_wb * norm(W_zf_B(lyr,:))^2;
    sinr_B(lyr) = sig / (intf + nse);
end
sinr_B_dB = 10*log10(sinr_B);
cqi_B     = arrayfun(@(s) max(sum(s >= cqi_tbl), 1), sinr_B_dB);
cap_B     = sum(log2(1 + snr_sv_B(1:ri_B) / ri_B));

fb.ri_B  = ri_B;
fb.cqi_B = cqi_B;    % [ri_B x 1]
fb.cap_B = cap_B;
fb.W_B   = W_B;

% =========================================================================
%  APPROACH C: Rel-19 Mode A
% =========================================================================
repCfg                    = nrCSIReportConfig;
repCfg.NSizeBWP           = carrier.NSizeGrid;
repCfg.NStartBWP          = 0;
repCfg.CodebookType       = 'typeI-SinglePanel-r19';
repCfg.PanelDimensions    = [1, 16, 4];
repCfg.PMIFormatIndicator = 'wideband';
repCfg.CodebookMode       = 1;

ri_C = 1;  best_rate_C = -Inf;
for ri_try = 1:ri_max
    try
        [~, info_try] = nr5g.internal.nrPMIReport( ...
            carrier, csirs{1}, repCfg, ri_try, H_4d, nVar_wb);
        rate_ = real(log2(det(eye(nRxAntennas) + ...
            (1/nVar_wb) * (H_wb*info_try.W*(H_wb*info_try.W)'))));
        if rate_ > best_rate_C;  best_rate_C = rate_;  ri_C = ri_try;  end
    catch
    end
end
[~, info_C] = nr5g.internal.nrPMIReport( ...
    carrier, csirs{1}, repCfg, ri_C, H_4d, nVar_wb);
W_C       = info_C.W;
cap_C     = real(log2(det(eye(nRxAntennas) + ...
    (1/nVar_wb) * (H_wb*W_C*(H_wb*W_C)'))));
sinr_C_dB = 10*log10(mean(info_C.SINRPerREPMI, 1, 'omitnan'));
cqi_C     = arrayfun(@(s) max(sum(s >= cqi_tbl), 1), sinr_C_dB);

fb.ri_C  = ri_C;
fb.cqi_C = cqi_C;    % [1 x ri_C]
fb.cap_C = cap_C;
fb.W_C   = W_C;

% =========================================================================
%  APPROACH D: Rel-19 Mode B
% =========================================================================
repCfg.CodebookMode = 2;

ri_D = 1;  best_rate_D = -Inf;
for ri_try = 1:ri_max
    try
        [~, info_try] = nr5g.internal.nrPMIReport( ...
            carrier, csirs{1}, repCfg, ri_try, H_4d, nVar_wb);
        rate_ = real(log2(det(eye(nRxAntennas) + ...
            (1/nVar_wb) * (H_wb*info_try.W*(H_wb*info_try.W)'))));
        if rate_ > best_rate_D;  best_rate_D = rate_;  ri_D = ri_try;  end
    catch
    end
end
[~, info_D] = nr5g.internal.nrPMIReport( ...
    carrier, csirs{1}, repCfg, ri_D, H_4d, nVar_wb);
W_D       = info_D.W;
cap_D     = real(log2(det(eye(nRxAntennas) + ...
    (1/nVar_wb) * (H_wb*W_D*(H_wb*W_D)'))));
sinr_D_dB = 10*log10(mean(info_D.SINRPerREPMI, 1, 'omitnan'));
cqi_D     = arrayfun(@(s) max(sum(s >= cqi_tbl), 1), sinr_D_dB);

fb.ri_D  = ri_D;
fb.cqi_D = cqi_D;    % [1 x ri_D]
fb.cap_D = cap_D;
fb.W_D   = W_D;

end
