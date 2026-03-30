function m = csirs_computeMetrics(fb, carrier)
%CSIRS_COMPUTEMETRICS  Derive RI, MCS, Throughput from CSI feedback struct.
%
%  CQI → MCS mapping follows TS 38.214 Table 5.1.3.1-2 (64QAM table).
%  Throughput is estimated as a per-slot link-level approximation:
%    TP = RI × ModOrder × CodeRate × N_RE_per_slot / T_slot
%
%  Inputs:
%    fb      - struct from csirs_feedback (ri_B/C/D, cqi_B/C/D, cap_B/C/D)
%    carrier - nrCarrierConfig (NSizeGrid, SubcarrierSpacing)
%
%  Output:
%    m - struct with fields (each is [1x3] vector, order: B C D):
%          .ri  - average RI per approach
%          .mcs - average MCS index per approach
%          .tp  - throughput in Mbps per approach

% ── CQI → [ModOrder, CodeRate/1024] (TS 38.214 Table 5.1.3.1-2) ─────────
%  Col: [CQI, ModOrder, CodeRate*1024]
cqiTable = [
     1,  2,   78;
     2,  2,  193;
     3,  2,  449;
     4,  4,  378;
     5,  4,  490;
     6,  4,  616;
     7,  6,  466;
     8,  6,  567;
     9,  6,  666;
    10,  6,  772;
    11,  6,  873;
    12,  6,  948;
    13,  8,  711;
    14,  8,  797;
    15,  8,  885;
];

% ── MCS lookup ───────────────────────────────────────────────────────────
%  MCS index (0-based) is taken as the row index in cqiTable that best
%  matches the CQI (CQI is already 1-based index into this table).
%  Average CQI across layers → single MCS per approach.
cqi2mcs = @(cqiVec) round(mean(cqiVec));  % avg layer CQI → MCS index

mcsIdx_B = cqi2mcs(fb.cqi_B);
mcsIdx_C = cqi2mcs(fb.cqi_C);
mcsIdx_D = cqi2mcs(fb.cqi_D);

mcsIdx_B = max(1, min(15, mcsIdx_B));
mcsIdx_C = max(1, min(15, mcsIdx_C));
mcsIdx_D = max(1, min(15, mcsIdx_D));

% ── Resource geometry ────────────────────────────────────────────────────
nPRB        = carrier.NSizeGrid;
nSC_per_PRB = 12;
nSym_PDSCH  = 12;              % ~12 DMRS-free symbols per slot (conservative)
slotDur_s   = 1e-3 / (carrier.SubcarrierSpacing / 15);  % e.g. 0.5ms @ 30kHz
nRE_slot    = nPRB * nSC_per_PRB * nSym_PDSCH;

% ── Throughput per approach ───────────────────────────────────────────────
%  TP (bps) = RI × ModOrder × (CodeRate/1024) × nRE_slot / slotDur_s
tp_fn = @(ri, mcsIdx) ...
    ri * cqiTable(mcsIdx, 2) * (cqiTable(mcsIdx, 3)/1024) * nRE_slot / slotDur_s;

tp_B = tp_fn(fb.ri_B, mcsIdx_B) / 1e6;   % Mbps
tp_C = tp_fn(fb.ri_C, mcsIdx_C) / 1e6;
tp_D = tp_fn(fb.ri_D, mcsIdx_D) / 1e6;

% ── Pack output [1x3]: order B C D ───────────────────────────────────────
m.ri  = [fb.ri_B,  fb.ri_C,  fb.ri_D];
m.mcs = [mcsIdx_B, mcsIdx_C, mcsIdx_D];
m.tp  = [tp_B,     tp_C,     tp_D];

end
