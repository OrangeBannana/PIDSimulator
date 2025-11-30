clear, clc, clf;

p = 0.014;
i = 0.03;
d = 0.0006;
f = 0.07;

% System Constants

mass = 1.2; % Kg

reductionRatio = 12/60; % Input Teeth : Output Teeth

spoolRadius = 20/1000; % Spool Radius in Meters

controlPeriod = 26 / 1000; % Time between updates in seconds

gravityConstant = -9.81; % m/s^2

systemInertiaAtShaft = mass * spoolRadius^2;

motorCount = 2;

kineticFrictionForce = 0.5 * gravityConstant * mass;
staticFrictionForce = 1.5 * gravityConstant * mass;


% Motor Constants

maxVoltage = 12; % Volts

freeSpeed = (5900 * 2 * pi)/60; % No load speed rad/s
freeCurrent = 0.3; % A

stallTorque = 0.19; % N * m
stallCurrent = 11; % A

backEMFconstant = maxVoltage / freeSpeed; % V / Rad/s or N * m /A

armatureResistance = maxVoltage / stallCurrent; % Ohms

% Working Variables

angularPosition = 0.0;
angularVelocity = 0.0;
angularAcceleration = 0.0;

previousError = 0.0;

controller = Controller(p, i, d);

integralSum = 0.0;

% Situation Variables
targetPosition = 100; % cm
angularTargetPosition = 28 * ((targetPosition / (2 * pi * spoolRadius * 100)) / reductionRatio); % ticks

overallTime = tic;
timer = tic;

data = animatedline;
hold on;


while (true)
    timeStep = toc(timer);
    time = toc(overallTime);

    if timeStep >= controlPeriod

        %% Update system to real time
        
        % Calculate Angular Position & update angular velocity
        angularPosition = angularPosition + timeStep * angularVelocity + (1/2) * angularAcceleration * timeStep^2;
        
        angularVelocity = angularVelocity + angularAcceleration * timeStep;

        % Find linear position in cm
        position = ((angularPosition / (2 * pi)) * reductionRatio) * 2 * pi * spoolRadius * 100;
        
        %% Update Controller
        
        angularPositionTicks = (angularPosition / (2 * pi)) * 28;
        controller = controller.update(angularPositionTicks, angularTargetPosition); % Controller output is clamped [-1,1]
        
        errorDeriv = (controller.positionError - previousError) / timeStep;
        previousError = controller.positionError;

        %integralSum = integralSum + controller.positionError * timeStep;
        integralSum = integralSum + controller.positionError * timeStep;

        if (abs(integralSum) > 1)
            integralSum = 1 * sign(integralSum);
        end

        controlSum = controller.output + f + errorDeriv * d + i * integralSum;

        appliedPower = clip(controlSum, -1, 1);

        %integralSum = integralSum - (i * (controlSum - appliedPower) * timeStep);

        appliedVoltage = appliedPower * maxVoltage ;


        %% Calculate Motor Torque

        backEMFVoltage = backEMFconstant * angularVelocity;
        netDrivingVoltage = appliedVoltage - backEMFVoltage;

        armatureCurrent = netDrivingVoltage / armatureResistance;

        motorTorque = backEMFconstant * armatureCurrent;

        %% Calculate Net Torque

        motorTorqueAtShaft = (motorTorque * motorCount) / reductionRatio;

        gravityTorqueAtShaft = gravityConstant * mass * spoolRadius;
        
        if (abs(angularVelocity) < 0.1) 
            frictionTorqueAtShaft = sign(motorTorqueAtShaft + gravityTorqueAtShaft) * staticFrictionForce * spoolRadius;
            if (abs(frictionTorqueAtShaft) >= abs(motorTorqueAtShaft + gravityTorqueAtShaft))
                netTorqueAtShaft = 0;
                angularVelocity = 0;
            else
                frictionTorqueAtShaft = sign(angularVelocity) * kineticFrictionForce * spoolRadius;
                netTorqueAtShaft = motorTorqueAtShaft + gravityTorqueAtShaft + frictionTorqueAtShaft;
            end

        else
            frictionTorqueAtShaft = sign(angularVelocity) * kineticFrictionForce * spoolRadius;
            netTorqueAtShaft = motorTorqueAtShaft + gravityTorqueAtShaft + frictionTorqueAtShaft;
        end
        
        %% Calculate angular position, velocity, and acceleration

        angularAcceleration = netTorqueAtShaft / systemInertiaAtShaft;

        timer = tic;
        addpoints(data, time, position)
        xlim([time - 5, time])
        ylim([0 inf])
        drawnow limitrate;

        fprintf("\n\nNEXT TIME STEP\n")
        fprintf("Time Step: %.2f\n", timeStep)
        fprintf("Position: %.2f\n", position)
        fprintf("Angular Position: %.2f\n", angularPosition)
        fprintf("Angular Velocity: %.2f\n", angularVelocity)
        fprintf("Angular Acceleration: %.2f\n", angularAcceleration)
        fprintf("Controller pGain: %.2f\n", controller.pGain)
        fprintf("Controller p: %.2f\n", controller.p)
        fprintf("Controller Output: %.2f\n", controller.output)
        fprintf("Controller Position Error: %.2f\n", controller.positionError)
        fprintf("Overall Time: %.2f\n", time)
        fprintf("Net Torque: %.2f\n", netTorqueAtShaft)
        fprintf("Applied Voltage: %.2f\n", appliedVoltage)
        fprintf("Motor Torque: %.2f\n", motorTorque)
        fprintf("Motor Torque At Shaft: %.2f\n", motorTorqueAtShaft)
        fprintf("Gravity Torque At Shaft: %.2f\n", gravityTorqueAtShaft)


    end

    

end
