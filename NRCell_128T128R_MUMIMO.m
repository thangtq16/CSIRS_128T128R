%% NR Cell Performance — 128T128R Downlink MU-MIMO (Rel-19 eTypeII)
% Evaluates DL MU-MIMO system performance using abstract PHY (L2SM),
% 128-antenna gNB with eTypeII-r19 codebook (TS 38.214 §5.2.2.2.5a).
%
% Author: ThangTQ23 — VSI, 2026-04

%% 1. Simulation Setup

% Prepend local patched wirelessnetwork + 5g toolbox overrides so that
% 128T-capable nrGNB (allows NumTransmitAntennas up to 128) and the
% eTypeII-r19 PHY stack are found before the installed support packages.
projectRoot = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(projectRoot, 'wirelessnetwork')), '-begin');
addpath(genpath(fullfile(projectRoot, '5g')),              '-begin');

wirelessnetworkSupportPackageCheck

rng("default")
numFrameSimulation = 5;
networkSimulator   = wirelessNetworkSimulator.init;

%% 2. gNB Configuration
% 128T128R massive MIMO, TDD, 4.9 GHz n79, 10 MHz / 30 kHz SCS
% (Use channelBW = 100e6 for production; 10 MHz here for fast simulation.)

gNBPosition  = [0 0 25];       % [x y z] metres
duplexType   = "TDD";

carrierFreq  = 4.9e9;          % n79 band centre frequency (Hz)
channelBW    = 10e6;           % channel bandwidth (Hz)
scs          = 30e3;           % subcarrier spacing (Hz)

% Valid NRB per 3GPP TS 38.104 Table 5.3.2-1 for FR1 30 kHz SCS:
%   5 MHz→11, 10 MHz→24, 15 MHz→38, 20 MHz→51, 40 MHz→106, 100 MHz→273
numRB = 24;   % matches channelBW = 10 MHz, SCS = 30 kHz

gNB = nrGNB( ...
    Position             = gNBPosition, ...
    TransmitPower        = 34, ...
    CarrierFrequency     = carrierFreq, ...
    ChannelBandwidth     = channelBW, ...
    SubcarrierSpacing    = scs, ...
    NumResourceBlocks    = numRB, ...
    NumTransmitAntennas  = 128, ...
    NumReceiveAntennas   = 128, ...
    DuplexMode           = duplexType, ...
    ReceiveGain          = 11, ...
    SRSPeriodicityUE     = 20);

disp("gNB: " + duplexType + ", " + carrierFreq/1e9 + " GHz, " + ...
     channelBW/1e6 + " MHz, NRB=" + numRB + ", 128T128R");

%% 3. Scheduler Configuration
% CSI measurement source: "CSI-RS" (eTypeII-r19 feedback) or "SRS" (TDD reciprocity)

csiMeasurementSignalDLType = "CSI-RS";
allocationType             = 0;   % RAT-0 (RBG-based)

% CSI-RS mode fields: SemiOrthogonalityFactor, MinCQI (not MinSINR — that is SRS-only)
muMIMOConfiguration = struct( ...
    MaxNumUsersPaired       = 2, ...
    MaxNumLayers            = 8, ...
    MinNumRBs               = 2, ...
    SemiOrthogonalityFactor = 0.9, ...
    MinCQI                  = 3);

configureScheduler(gNB, ...
    ResourceAllocationType = allocationType, ...
    MaxNumUsersPerTTI      = 10, ...
    MUMIMOConfigDL         = muMIMOConfiguration, ...
    CSIMeasurementSignalDL = csiMeasurementSignalDLType);

%% 4. UE Deployment
% 10 UEs uniformly distributed at 500 m, azimuth ±60°

numUEs         = 10;
ueRxGain       = 0;
ueNumTxAnt     = 1;
ueNumRxAnt     = 4;

azDeg       = -60 + 120 .* rand(numUEs, 1);   % uniform in [-60, 60] deg
[xPos, yPos, zPos] = sph2cart(deg2rad(azDeg), zeros(numUEs,1), 500*ones(numUEs,1));
uePositions = [xPos, yPos, zPos] + gNBPosition;
ueNames     = "UE-" + (1:numUEs);

UEs = nrUE( ...
    Name                = ueNames, ...
    Position            = uePositions, ...
    ReceiveGain         = ueRxGain, ...
    NumTransmitAntennas = ueNumTxAnt, ...
    NumReceiveAntennas  = ueNumRxAnt);

connectUE(gNB, UEs, FullBufferTraffic = "DL", CSIReportPeriodicity = 10);

%% 5. Network Simulator — Add Nodes

addNodes(networkSimulator, gNB);
addNodes(networkSimulator, UEs);

%% 6. CDL Channel Model
% CDL-B, 100 ns delay spread, 5 Hz Doppler (frequency-selective, rank-friendly)

channelConfig = struct( ...
    DelayProfile        = "CDL-B", ...
    DelaySpread         = 100e-9, ...
    MaximumDopplerShift = 5);

channels = hNRCreateCDLChannels(channelConfig, gNB, UEs);
customChannelModel = hNRCustomChannelModel(channels);
addChannelModel(networkSimulator, @customChannelModel.applyChannelModel);

%% 7. Logging and Visualization

enableTraces = true;

if enableTraces
    simSchedulingLogger = helperNRSchedulingLogger(numFrameSimulation, gNB, UEs);
    simPhyLogger        = helperNRPhyLogger(numFrameSimulation, gNB, UEs);
end

numMetricPlotUpdates = 200;
metricsVisualizer = helperNRMetricsVisualizer(gNB, UEs, ...
    RefreshRate         = numMetricPlotUpdates, ...
    PlotSchedulerMetrics = true, ...
    PlotPhyMetrics      = false, ...
    PlotCDFMetrics      = true, ...
    LinkDirection       = 0);

simulationLogFile = "simulationLogs";

%% 8. Run Simulation

simulationTime = numFrameSimulation * 1e-2;   % seconds
run(networkSimulator, simulationTime);

%% 9. Results — KPI Summary
% Display cell throughput, spectral efficiency, and BLER ECDF.

displayPerformanceIndicators(metricsVisualizer);

%% 10. Save Traces

if enableTraces
    simulationLogs = cell(1,1);
    if gNB.DuplexMode == "FDD"
        logInfo = struct(DLTimeStepLogs=[], ULTimeStepLogs=[], ...
                         SchedulingAssignmentLogs=[], PhyReceptionLogs=[]);
        [logInfo.DLTimeStepLogs, logInfo.ULTimeStepLogs] = ...
            getSchedulingLogs(simSchedulingLogger);
    else  % TDD
        logInfo = struct(TimeStepLogs=[], SchedulingAssignmentLogs=[], ...
                         PhyReceptionLogs=[]);
        logInfo.TimeStepLogs = getSchedulingLogs(simSchedulingLogger);
    end
    logInfo.SchedulingAssignmentLogs = getGrantLogs(simSchedulingLogger);
    logInfo.PhyReceptionLogs         = getReceptionLogs(simPhyLogger);
    simulationLogs{1} = logInfo;
    save(simulationLogFile, "simulationLogs");
end

%% 11. MU-MIMO Pairing Analysis — UEs per RB Distribution
% Histogram shows how often multiple UEs share the same RB in DL slots.

if enableTraces
    avgNumUEsPerRB = calculateAvgUEsPerRBDL(logInfo, gNB.NumResourceBlocks, ...
                                             allocationType, duplexType);
    figure;
    histogram(avgNumUEsPerRB, BinWidth=0.1);
    title("Distribution of Average Number of UEs per RB in DL Slots");
    xlabel("Average Number of UEs per RB");
    ylabel("Number of Occurrences");
    grid on;
end

%% Local Function

function avgUEsPerRB = calculateAvgUEsPerRBDL(logInfo, numResourceBlocks, ratType, duplexMode)
    % Returns average number of UE nodes per RB for each DL slot.
    % Supports RAT-0 (RBG-based) and RAT-1 (contiguous-RB-based) allocation.

    if strcmp(duplexMode, 'TDD')
        timeStepLogs    = logInfo.TimeStepLogs;
        freqAllocations = timeStepLogs(:, 5);
    else
        timeStepLogs    = logInfo.DLTimeStepLogs;
        freqAllocations = timeStepLogs(:, 4);
    end

    numOfSlots = size(timeStepLogs, 1) - 1;

    if ~ratType
        % Derive per-RBG sizes per TS 38.214
        numRBG       = size(freqAllocations{2}, 2);
        P            = ceil(numResourceBlocks / numRBG);
        numRBsPerRBG = P * ones(1, numRBG);
        remainder    = mod(numResourceBlocks, P);
        if remainder > 0
            numRBsPerRBG(end) = remainder;
        end
    end

    avgUEsPerRB = zeros(1, numOfSlots);

    for slotIdx = 1:numOfSlots
        % Skip UL slots in TDD
        if strcmp(duplexMode, 'TDD') && ~strcmp(timeStepLogs{slotIdx+1, 4}, 'DL')
            continue;
        end

        freqAllocation = freqAllocations{slotIdx + 1};

        if ~ratType
            % RAT-0: weight each RBG by its RB count
            totalUniqueUEs = sum(arrayfun( ...
                @(i) nnz(freqAllocation(:,i) > 0) * numRBsPerRBG(i), ...
                1:length(numRBsPerRBG)));
            avgUEsPerRB(slotIdx) = totalUniqueUEs / numResourceBlocks;
        else
            % RAT-1: count UEs per contiguous RB allocation
            ueRBUsage = zeros(1, numResourceBlocks);
            for ueIdx = 1:size(freqAllocation, 1)
                startRB = freqAllocation(ueIdx, 1);
                numRBs  = freqAllocation(ueIdx, 2);
                ueRBUsage(startRB+1 : startRB+numRBs) = ...
                    ueRBUsage(startRB+1 : startRB+numRBs) + 1;
            end
            avgUEsPerRB(slotIdx) = mean(ueRBUsage(ueRBUsage > 0));
        end
    end

    % Remove zero entries (UL slots)
    avgUEsPerRB = avgUEsPerRB(avgUEsPerRB > 0);
end
