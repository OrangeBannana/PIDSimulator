classdef Controller
    %% Class to define a system controller
    % It is meant to take some target input (float targetPosition) and a number of
    % constant values along with signal values, and it will output a vector
    % to achieve the target value

    properties
        pGain = 0.0, iGain = 0.0, dGain = 0.0;
        p = 0.0, i = 0.0, d = 0.0;

        targetPosition = 0.0;
        currentPosition = 0.0;
        positionError = 0.0;

        timeFormat = 'yyyy-MM-dd HH:mm:ss.SSS';
        lastUpdatedTime;
        currentTime;

        output = 0.0;
    end

    methods
        function obj = Controller(pGain, iGain, dGain)
            obj.pGain = pGain;
            obj.iGain = iGain;
            obj.dGain = dGain;
        end

        function obj = update(obj, currentPos, targetPos)

            obj.targetPosition = targetPos;
            obj.currentPosition = currentPos;
            obj.positionError = targetPos - currentPos;

            obj.p = obj.pGain * obj.positionError;
            obj.i = 0.00;
            obj.d = 0.00;

            obj.output = obj.p + obj.i + obj.d;
        end

    end
end