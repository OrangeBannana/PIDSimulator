classdef OptimalController < handle
    properties
        DecelFactor, SlewLimit, Name
    end
    
    methods
        function obj = OptimalController(df, slew, name)
            obj.DecelFactor = df; obj.SlewLimit = slew; obj.Name = name;
        end
        
        function [v, stopDistCm] = calculate(obj, state, mech, motor, targetCm)
            J = mech.getInertia();
            % Calculate max torque available at spool
            maxT = (motor.stallTorque * motor.count) / mech.ratio;
            decel = obj.DecelFactor * (maxT / J);
            
            % Kinematic stopping distance (Quadratic with velocity)
            stopDistRad = ((J * motor.getR()) / (motor.getKt() ^ 2)) * (state.vel * mech.ratio / motor.count)
            stopDistCm = (stopDistRad * mech.ratio) * mech.spoolR * 100;
            
            % Distance remaining
            targetRad = (targetCm/100) / (mech.spoolR * mech.ratio);
            distLeft = targetRad - state.pos
            
            % Gravity compensation voltage
            gravV = (abs(mech.getGravityTorque()) * mech.ratio * motor.getR()) / (motor.getKt() * motor.count);
            
            if abs(distLeft) > abs(stopDistRad)
                targetV = sign(distLeft) * 12;
            else
                % v = sqrt(2ad)
                targetV = distLeft * 0.2;
            end
            
            % Apply Slew
            v = targetV;
            v = min(max(v, -12), 12);
        end
    end
end