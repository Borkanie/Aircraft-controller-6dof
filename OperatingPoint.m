classdef OperatingPoint
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here

    properties
        PointOfOperation,
        KEmd,
        AEmd,
        BEmd,
        CEmd,
        Kp,
        Ki
    end

    methods
        function obj = OperatingPoint(states,inputs,winds)
            %UNTITLED2 Construct an instance of this class
            %   Detailed explanation goes here
            obj.Ki=1e-5;
            obj.Kp=1e-3;
            obj.PointOfOperation=PointOfEquilibrium(states,inputs,winds);
            [obj.AEmd,obj.BEmd,obj.CEmd,obj.KEmd]=ControlMinimal(obj.PointOfOperation);
        end
    end
end