classdef PointOfEquilibrium
    %POINTOFEQUILIBRIUM Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        States,
        Inputs,
        Winds
    end
    
    methods
        function obj = PointOfEquilibrium(States,Inputs,Winds)
            %POINTOFEQUILIBRIUM Construct an instance of this class
            %   Detailed explanation goes here
            obj.States=States;
            obj.Inputs=Inputs;
            obj.Winds=Winds;
        end
    end
end

