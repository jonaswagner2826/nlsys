classdef nlfeedback < nlcomposite
    %NLFEEDBACK combines two nlsys objects into a closed-loop model:
    % Used for simulating a feedback system with either a nonlinear
    %   feedback loop or non-linear dynamics.
    % 
    % u --->O---->[ M1 ]----+---> y
    %      |               |         
    %      +-----[ M2 ]<---+
    % 
    % Negative feedback is assumed, so modify acoridingly
    properties
        u
    end
    
    methods
        function sys = nlfeedback(sys1, sys2, t, u)
            % FEEDBACK combines two nlsys objects into a closed-loop model:
            % 
            % u --->O---->[ M1 ]----+---> y
            %      |               |         
            %      +-----[ M2 ]<---+
            % 
            % Negative feedback is assumed, so modify acoridingly
            arguments
                % sys1 - system (ctrl & plant)
                sys1
                % sys2 - feedback (optional) default = unity
                sys2 = nlsys(1);
                % t - current time (optional) default = 0
                t = 0;
                % u - current input (optional) default = 0
                u = 0;
            end
            % Compatability
            if sys1.q ~= sys2.p || sys2.q ~= sys1.p
                error('sys1 and sys2 incompatible');
            end            
            
            sys@nlcomposite(sys1,sys2,t)
            sys.p = sys1.p;
            sys.q = sys1.q;
            if u ~= 0
                if size(u) ~= [sys.p,1]
                    warning('u incorrect size, set to zero')
                    u = 0;
                else
                    sys.u = u;
                end
            else
                sys.u = zeros(sys.p,1);
            end
        end
        
    end
    
    % Overload Methods
    methods
        function disp(sys)
            fprintf('Controller/Plant sys:\n')
            disp(sys.sys1)
            fprintf('\n Feedback sys: \n')
            disp(sys.sys2)
            fprintf('Feedback Nonlinear System\n')
        end
    end
    
    methods
%         function sys = plus(sys1,sys2)
            
%         end
%         sys = mtimes(sys1,sys2); shouldn't be needed... times is same
%         thing... just for different calls
%         function sys = times(sys1,sys2)
            
%         end
    end
    
    % Standard Operation
    methods
        function sys = update(sys,r,t,x)
            % UPDATE - return an updated system based on u and t...
            % and x (optional)            
            arguments
                % sys is the nlfeedback sys
                sys
                % r is the closed loop reference input
                r = 0;
                % t is the time past from last step
                t = -1;
                % x = [x1; x2] (optional)
                x = -1;
            end
            
            % Input Defaults
            if nargin < 2 || r == 0
                r = zeros(sys.p,1); % stabilization
            end
            if nargin < 3 || t == -1
                t = sys.Ts; % only take one time step
            end
            if nargin < 4 || x == -1
                x = sys.x;
            end
            
            % Input verification
            if size(r) ~= [sys.p,1]
                error('r is the wrong size')
            end
            if t <= 0
                error('t must be greater then zero')
            end
            if size(x) ~= [sys.n,1]
                error('x is incorrect size')
            end
            
            n1 = sys.sys1.n;
            n2 = sys.sys2.n;
            
            x1 = x(1:n1);
            x2 = x((n1+1):(n1+n2));
            
            sys1 = sys.sys1.update(sys.u,t,x1);
            x1 = sys1.x;
            y1 = sys.sys1.h(x1,sys.u);
            
            sys2 = sys.sys2.update(y1,t,x2);
            x2 = sys2.x;
            y2 = sys.sys2.h(x2,y1);
            
            u_local = r - y2;
            
            sys.sys1 = sys1;
            sys.sys2 = sys2;
            sys.x = [x1;x2];
            sys.u = u_local;
        end
        
        function [A,B,C,D] = linearize(sys,x_0,u_0)
            
        end
        
    end    
    
    
end

