classdef aircraft
    %A class that will symbolize the system of an aircraft with 6 DOF
    % You can call both linear and nonlinear systems
    
    properties
        g0;
        m;
        Aw;
        Ast;
        Af_fuselage;
        Af_wing;
        Afrontal;
        Alat;
        Arudder;
        Aaileron;
        Ahorizontal;
        ro;
        Iinv;
        I;
        Ix;
        Iz;
        Iy;
        Ixz;
        Tau;
        c1;
        c2;
        c3;
        c4;
        c5;
        c6;
        c7;
        c8;
        c9;
        l1;
        l2;
        d;
        b;
        c_bar;
        l3;
        A;
        B;
        C;
        D;
    end
    
    methods
        
        function obj = aircraft()
            %We create a new instance of an aircraft with default values
            %   It's small wing UAV
            
            %gravitational acceleration
            obj.g0=9.81;
            %mass
            obj.m=0.63;
            %wing arrea
            obj.Aw=0.7*0.12;
            %stabilizer areea
            obj.Ast=0.0699*0.08708;
            %fuselage frontal areea
            obj.Af_fuselage=0.05*0.05;
            %Wing fronat areea
            obj.Af_wing=0.105*0.55;
            %total frontal areea
            obj.Afrontal=obj.Af_fuselage+obj.Af_wing;
            %total lateral areea
            obj.Alat=0.44*0.05;
            %rudder lateral areea
            obj.Arudder=3/4*0.05*0.05;
            %Aileron horizontal areea
            obj.Aaileron=0.07*0.05;
            %total gorizotnal areea excluding aileron
            obj.Ahorizontal=obj.Ast+obj.Aw;
            %air density(consideret constant)
            obj.ro=1.225;
            %inertia amtrix
            obj.Ix=17/100;
            obj.Iz=55/100;
            obj.Iy=46/100;
            obj.Ixz=-2.4/100;
            obj.I=[obj.Ix,0,obj.Ixz;
                0,obj.Iy,0;
                obj.Ixz,0,obj.Iz];
            obj.Iinv=inv(obj.I);
            %inertia constants
            obj.Tau=obj.Ix*obj.Iz-obj.Ixz^2;
            obj.c1=((obj.Iy-obj.Iz)*obj.Iz-obj.Ixz^2)/obj.Tau;
            obj.c2=(obj.Ix-obj.Iy+obj.Iz)*obj.Ixz/obj.Tau;
            obj.c3=obj.Iz/obj.Tau;
            obj.c4=obj.Ixz/obj.Tau;
            obj.c5=(obj.Iz-obj.Ix)/obj.Iy;
            obj.c6=obj.Ixz/obj.Iy;
            obj.c7=1/obj.Iy;
            obj.c8=(obj.Ix*(obj.Ix-obj.Iy)+obj.Ixz^2)/obj.Tau;
            obj.c9=obj.Ix/obj.Tau;
            %length from aerodynamic center of wing to center of mass
            obj.l1=0.02;
            %length from aerodynamic center of stabilizer to center of mass
            obj.l2=-0.276;
            %CoG of motor center of wing to CoG of the aircraft
            obj.d=0;
            %wingspan
            obj.b=0.55;
            %mean gemetric chord of the wing
            %its the mean width of the wing
            obj.c_bar=0.0105;
            obj.l3=0.03;           
        end
        
        function Va=getAirspeed(obj,u,v,w)
            Va=sqrt(u^2+v^2+w^2);
        end
        
    end
end

