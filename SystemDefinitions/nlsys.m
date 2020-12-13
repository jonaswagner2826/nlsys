classdef nlsys
    %NLSYS This class describes non-linear dynamical systems for simulation
    %   Class developed to do similar things as the MATLAB control systems
    %   toolbox but for nonlinear state funactions and outputs
    
    
    properties
        % f - State_eq... linear or nonlinear function describing the
        % update of the state itself. Should be @(x,u)
        f
        % h - Output_equation... should be @(x,u)
        h
        % x - Current state of the system, should be of the correct size
        % to satisfy f(x,u)...
        x
        
        % System size
        % n - number of states
        n (1,1) int32
        % p - number of inputs
        p (1,1) int32
        % q - number of outputs
        q (1,1) int32
    end
    
    methods
        function sys = nlsys(f, h, x)
            % Constructor for nlsys... 
            % f is state eq... must be of correct size, shape etc.
            % h is output eq (optional)
            % x is the current state (optional)
            arguments
                f (:,1)
                h       = @(x,u) x;
                x (:,1) = -1;
            end
            sys.f = f;
            
            temp = f();
            sys.n = temp(1);
            sys.p = temp(2);
            sys.q = temp(3);
            
            sys.h = h;
            if x == -1
                x = zeros(sys.n,1);
            end
            sys.x = x;
        end
        
        
        % Standar System Operation
%         function  = StateUpdate
        
        
        
        
        
        % Data Export
        function [f, h, x, n, p, q] = nlsysdata(sys)
            % NLSYSDATA output all the system properties
            f = sys.f;
            h = sys.h;
            x = sys.x;
            n = sys.n;
            p = sys.p;
            q = sys.q;
        end
        
        
        
    end
end

