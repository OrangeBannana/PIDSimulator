classdef Motor
    properties
        count = 2
        stallTorque = 0.19      % Nm
        freeSpeedRPM = 5900     
        maxV = 12               
        stallCurrent = 11       
    end
    
    methods
        function kt = getKt(obj)
            kt = obj.maxV / ((obj.freeSpeedRPM * 2 * pi) / 60);
        end
        
        function r = getR(obj)
            r = obj.maxV / obj.stallCurrent;
        end
        
        function torque = getTorque(obj, voltage, omega)
            % T = Kt * (V - Ke*w) / R
            kt = obj.getKt();
            torque = (kt * (voltage - kt * omega) / obj.getR()) * obj.count;
        end
    end
end