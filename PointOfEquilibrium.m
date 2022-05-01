classdef PointOfEquilibrium
    %POINTOFEQUILIBRIUM Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        States,
        Inputs
    end
    
    methods
        function obj = PointOfEquilibrium(States,Inputs)
            %POINTOFEQUILIBRIUM Construct an instance of this class
            %   Detailed explanation goes here
            obj.States=States;
            obj.Inputs=Inputs;
        end
    end
end

