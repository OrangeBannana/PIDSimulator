classdef QuadraticPIDController < handle
    properties
        Kp, Ki, Kd, Kf, Name
        integral = 0; prevErr = 0;
    end
    
    methods
        function obj = QuadraticPIDController(p, i, d, f, name)
            obj.Kp = p; obj.Ki = i; obj.Kd = d; obj.Kf = f; obj.Name = name;
        end
        
        function [v, stopDist] = calculate(obj, state, mech, motor, targetCm)
            % Target in ticks
            targetTicks = 28 * ((targetCm / (2*pi*mech.spoolR*100)) / mech.ratio);
            currTicks = (state.pos / (2*pi)) * 28;
            err = targetTicks - currTicks;
            
            dt = 0.026; % Fixed loop time
            deriv = (err - obj.prevErr) / (dt);
            deriv = deriv / abs(err);
            obj.integral = obj.integral + err * dt;
            
            v = (err*obj.Kp + obj.integral*obj.Ki + deriv*obj.Kd + obj.Kf) * 12;
            v = min(max(v, -12), 12);
            stopDist = 0;
            obj.prevErr = err;
        end
    end
end