%% NR Cell Performance — 128T128R Downlink MU-MIMO (Rel-19 eTypeII)
% Evaluates DL MU-MIMO system performance using abstract PHY (L2SM),
% 128-antenna gNB with eTypeII-r19 codebook (TS 38.214 §5.2.2.2.5a).
%
% Author: ThangTQ23 — VSI, 2026-04

%% 1. Simulation Setup

wirelessnetworkSupportPackageCheck

rng("default")
numFrameSimulation = 5;
networkSimulator   = wirelessNetworkSimulator.init;

%% 2. gNB Configuration
% 128T128R massive MIMO, TDD, 3.5 GHz n78, 20 MHz / 30 kHz SCS

gNBPosition = [0 0 30];   % [x y z] metres
duplexType  = "TDD";

gNB = nrGNB(Position=gNBPosition, TransmitPower=34, ...
    SubcarrierSpacing=30000, CarrierFrequency=3.5e9, ...
    ChannelBandwidth=20e6, NumTransmitAntennas=128, NumReceiveAntennas=128, ...
    DuplexMode=duplexType, ReceiveGain=11, ...
    SRSPeriodicityUE=20, NumResourceBlocks=51);

%% 3. Scheduler Configuration
% CSI measurement source: "SRS" (TDD reciprocity) or "CSI-RS"
csiMeasurementSignalDLType = "SRS";
allocationType             = 0;   % RAT-0 (RBG-based)

muMIMOConfiguration = struct(MaxNumUsersPaired=2, MaxNumLayers=8, ...
                             MinNumRBs=2, MinSINR=10);

configureScheduler(gNB, ResourceAllocationType=allocationType, ...
    MaxNumUsersPerTTI=10, MUMIMOConfigDL=muMIMOConfiguration, ...
    CSIMeasurementSignalDL=csiMeasurementSignalDLType);

%% 4. UE Deployment
% 10 UEs uniformly distributed at 500 m, azimuth ±60°

numUEs        = 10;
ueRelPosition = [ones(numUEs,1)*500, (rand(numUEs,1)-0.5)*120, zeros(numUEs,1)];
[xPos, yPos, zPos] = sph2cart(deg2rad(ueRelPosition(:,2)), ...
                               deg2rad(ueRelPosition(:,3)), ...
                               ueRelPosition(:,1));
uePositions = [xPos yPos zPos] + gNBPosition;
ueNames     = "UE-" + (1:numUEs);

UEs = nrUE(Name=ueNames, Position=uePositions, ReceiveGain=0, ...
           NumTransmitAntennas=1, NumReceiveAntennas=4);

connectUE(gNB, UEs, FullBufferTraffic="DL", CSIReportPeriodicity=10);

%% 5. Network Simulator — Add Nodes

addNodes(networkSimulator, gNB);
addNodes(networkSimulator, UEs);

%% 6. CDL Channel Model
% CDL-B, 100 ns delay spread, 5 Hz Doppler (frequency-selective, rank-friendly)

channelConfig = struct(DelayProfile="CDL-B", DelaySpread=100e-9, ...
                       MaximumDopplerShift=5);
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
    RefreshRate=numMetricPlotUpdates, PlotSchedulerMetrics=true, ...
    PlotPhyMetrics=false, PlotCDFMetrics=true, LinkDirection=0);

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
% Histogram shows how often multiple UEs share the same RB in DL slots,
% which is a direct indicator of MU-MIMO pairing effectiveness.

if enableTraces
    avgNumUEsPerRB = calculateAvgUEsPerRBDL(logInfo, gNB.NumResourceBlocks, ...
                                             allocationType, duplexType);
    figure;
    theme("light");
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
