clear, clc, clf;

%% --- Shared Constants ---
mass = 1.2; 
reductionRatio = 12/60; 
spoolRadius = 20/1000; 
controlPeriod = 10 / 1000; 
gravityConstant = -9.81; 
systemInertiaAtShaft = mass * spoolRadius^2;
motorCount = 2;
kineticFrictionForce = 0.1 * gravityConstant * mass;
staticFrictionForce = 0.1 * gravityConstant * mass;
maxVoltage = 12; 
freeSpeed = (5900 * 2 * pi)/60; 
stallCurrent = 11; 
backEMFconstant = maxVoltage / freeSpeed; 
armatureResistance = maxVoltage / stallCurrent; 
targetPositionCm = 100;
simTimeLimit = 5.0;

%% --- Run Simulations ---
[timePID, posPID] = runSim(false);
[timeOpt, posOpt] = runSim(true);

%% --- Plotting Results ---
figure(1); hold on;
plot(timePID, posPID, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Standard PID');
plot(timeOpt, posOpt, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Optimal Regen');
yline(targetPositionCm, 'k:', 'Target');
xlabel('Time (s)'); ylabel('Position (cm)');
title('FTC Motor Control Comparison');
legend('Location', 'southeast');
grid on;
ylim([0 targetPositionCm * 1.2]);

%% --- Simulation Function ---
function [t_out, p_out] = runSim(isOptimal)
    % Import constants from caller workspace
    ws = 'caller';
    mass = evalin(ws, 'mass'); reductionRatio = evalin(ws, 'reductionRatio');
    spoolRadius = evalin(ws, 'spoolRadius'); controlPeriod = evalin(ws, 'controlPeriod');
    gravityConstant = evalin(ws, 'gravityConstant'); systemInertiaAtShaft = evalin(ws, 'systemInertiaAtShaft');
    motorCount = evalin(ws, 'motorCount'); kineticFrictionForce = evalin(ws, 'kineticFrictionForce');
    staticFrictionForce = evalin(ws, 'staticFrictionForce'); maxVoltage = evalin(ws, 'maxVoltage');
    backEMFconstant = evalin(ws, 'backEMFconstant'); armatureResistance = evalin(ws, 'armatureResistance');
    targetPositionCm = evalin(ws, 'targetPositionCm'); simTimeLimit = evalin(ws, 'simTimeLimit');

    % State Variables
    angPos = 0; angVel = 0; angAcc = 0;
    t = 0; prevError = 0; integralSum = 0;
    
    % Data storage
    t_out = []; p_out = [];
    
    % PID Gains
    p_gain = 0.003; i_gain = 0.001; d_gain = 0.00018; f_gain = 0.110;
    
    while t < simTimeLimit
        % Target in ticks
        targetTicks = 28 * ((targetPositionCm / (2 * pi * spoolRadius * 100)) / reductionRatio);
        currTicks = (angPos / (2 * pi)) * 28;
        errorTicks = targetTicks - currTicks;
        
        %% 1. Controller Logic
        if ~isOptimal
            % Standard PID + Feedforward
            deriv = (errorTicks - prevError) / controlPeriod;
            integralSum = clip(integralSum + errorTicks * controlPeriod, -1, 1);
            % Simplified PID logic for sim
            appliedVoltage = clip((errorTicks * p_gain + integralSum * i_gain + deriv * d_gain + f_gain) * maxVoltage, -12, 12);
            prevError = errorTicks;
        else
            % Optimal Control with Sprint/Brake Logic
            stoppingDistRad = (systemInertiaAtShaft * armatureResistance * (angVel/reductionRatio)) / (backEMFconstant^2);
            deltaTheta = (targetTicks - currTicks) * (2 * pi / 28);
            gravityTorque = gravityConstant * mass * spoolRadius;
            gravityV = (abs(gravityTorque) * reductionRatio * armatureResistance) / (backEMFconstant * motorCount);

            if abs(deltaTheta) > abs(stoppingDistRad)
                appliedVoltage = sign(deltaTheta) * maxVoltage;
            else
                appliedVoltage = (backEMFconstant * (angVel/reductionRatio)) + ...
                                 (deltaTheta * backEMFconstant^2) / (systemInertiaAtShaft * armatureResistance);
            end
            appliedVoltage = clip(appliedVoltage + gravityV, -12, 12);
        end

        %% 2. Physics Engine
        motorVel = angVel / reductionRatio;
        backEMF = backEMFconstant * motorVel;
        netV = appliedVoltage - backEMF;
        motorT = (backEMFconstant * (netV / armatureResistance) * motorCount) / reductionRatio;
        gravT = gravityConstant * mass * spoolRadius;
        
        % Friction model
        if abs(angVel) < 0.1
            fricT = sign(motorT + gravT) * staticFrictionForce * spoolRadius;
            netT = motorT + gravT + fricT;
            if abs(fricT) >= abs(motorT + gravT), netT = 0; angVel = 0; end
        else
            netT = motorT + gravT + sign(angVel) * kineticFrictionForce * spoolRadius;
        end
        
        angAcc = netT / systemInertiaAtShaft;
        angPos = angPos + controlPeriod * angVel + 0.5 * angAcc * controlPeriod^2;
        angVel = angVel + angAcc * controlPeriod;
        
        % Log Data
        t = t + controlPeriod;
        t_out(end+1) = t;
        p_out(end+1) = ((angPos / (2 * pi)) * reductionRatio) * 2 * pi * spoolRadius * 100;
    end
end

function out = clip(val, low, high)
    out = min(max(val, low), high);
end