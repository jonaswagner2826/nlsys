classdef nlsds < nlsys
    %NLSDS Nonlinear Sampled Data System
    %   Used for simulating a Sampled Data System with nonlinear state
    %   equations, but with discrete measurments.
    
    properties
        T_sensor
    end
    
    methods
        function sys = nlsds(f,T_sensor,T_input,h,x)
            % NLSDS this is the constructor for nlSDS objects
            arguments
                % f is state eq (or nlsys or lti object)
                f
                % T_sensor is the sampling time
                T_sensor
                % T_input is the update time of the control signal
                % (optional) default = same as sensor
                T_input = -1;
                % h is output eq (optional) default = output just x
                h = -1;
                % x is the current state (optional) default = relaxed
                x = -1;
            end
            
            % Input validation
            if T_sensor <= 0
                error('T_sensor must be > 0')
            end
            if T_input == -1
                T_input = T_sensor;
            elseif T_input <= 0
                error('T_input must be > 0')
            end
            
            % Input a sys
            if isa(f, 'lti') || isa(f, 'double')
                f = nlsys(f);
            end
            if isa(f,'nlsys')
                sys_old = f;
                f = sys_old.f;
                if nargin < 3
                    h = sys_old.h;
                end
                if nargin < 4
                    x = sys_old.x;
                end
            end
            
            % Nlsys definition
            sys@nlsys(f,h,x,-1);
            sys.T_sensor = T_sensor;
            sys.T_input = T_input;
        end
        
        
        % SDS operations
        function y = sample(sys,u,x)
            % SAMPLE samples the system (bassically just y)
            arguments
                % sys is the nonlin sys
                sys
                % u is the input
                u
                % x is the current state
                x = sys.x
            end
            y = sys.h(x,u);
        end
        
%         function u = input_signal(sys,t,
            
            
            

        
    end
end

