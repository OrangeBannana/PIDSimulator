classdef Mechanism
    properties
        mass = 1.2              % kg
        ratio = 12/60           % 5:1 reduction
        spoolR = 0.02           % 20mm
        g = -9.81               
        staticFric = 1.5 * 9.81 * 1.2 * 0.02 % Static torque
        kineticFric = 0.5 * 9.81 * 1.2 * 0.02 % Kinetic torque
    end
    
    methods
        function J = getInertia(obj)
            J = obj.mass * obj.spoolR^2;
        end
        
        function t_grav = getGravityTorque(obj)
            t_grav = obj.g * obj.mass * obj.spoolR;
        end
        
        function posCm = getLinearPos(obj, angPos)
            posCm = ((angPos / (2*pi)) * obj.ratio) * 2*pi * obj.spoolR * 100;
        end
    end
end