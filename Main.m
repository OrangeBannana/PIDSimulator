clear; clc; clf;

% Setup Physical System
myMotor = Motor();
myMech  = Mechanism();
target  = 100; % cm

% Setup Controllers to Compare
ctrls = {
    PIDController(0.014, 0, 0.00082, 0.10, 'Tuned PID'), ...
    OptimalController(.7, 4.0, 'Optimal Regen'), ...
    PIDController(0.003, 0, 0.0002, 0.10, 'Re-Tuned PID'), ...
    PIDController(0.0115, 0, 0.000685, 0.10, 'Re-Tuned PID2'), ...
    QuadraticPIDController(0.017, 0, 0.05, 0.10, 'Quadratic PID')
};

% Run Comparison
hold on;
for i = 1:length(ctrls)
    res = runSim(myMotor, myMech, ctrls{i}, target);
    plot(res.t, res.p, 'DisplayName', ctrls{i}.Name, 'LineWidth', 2);
end

yline(target, 'k:'); grid on; legend();
ylabel('Position (cm)'); xlabel('Time (s)');
xlim([.6 1]);
ylim([target - 10 target + 10]);

function res = runSim(motor, mech, ctrl, targetCm)
    state.pos = 0; state.vel = 0; state.t = 0; state.prevV = 0;
    dt = 0.026;
    res.t = []; res.p = [];
    
    while state.t < 3.0
        [v, ~] = ctrl.calculate(state, mech, motor, targetCm);
        
        torqueMotor = motor.getTorque(v, state.vel);
        torqueLoad = (torqueMotor / mech.ratio) + mech.getGravityTorque();
        
        % Net Torque with Friction
        netT = torqueLoad - (sign(state.vel) * mech.kineticFric);
        
        acc = netT / mech.getInertia();
        state.pos = state.pos + dt * state.vel;
        state.vel = state.vel + acc * dt;
        state.t = state.t + dt;
        state.prevV = v;
        
        res.t(end+1) = state.t;
        res.p(end+1) = mech.getLinearPos(state.pos);
    end
end