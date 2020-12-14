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
        % System Constructor
        function sys = nlsys(f, h, x)
            % Constructor for nlsys... 
            arguments
                % f is state eq (or nlsys or lti object)
                f
                % h is output eq (optional) default = output just x
                h               = @h_default;
                % x is the current state (optional) default = relaxed
                x (:,1) double = -1;
            end
            
            % Conversion setups
            if IsNLSYS(f)% direct nlsys conversion
                if nargin < 2
                    h = f.h;
                end
                if nargin < 3 % not sure why this would ever be used...
                    x = f.x;
                end
                f = f.f;
            end
            
            if IsLTI(f)% conversions from lti sys
                [a,b,c,d] = ssdata(f);
                f = @(x,u) a*x + b*u;
                if nargin < 2
                    h = @(x,u) c*x + d*u;
                end
                n = size(a,1);
                p = size(b,2);
                q = size(c,1);
            end
            
            % State Equation
            sys.f = f;
            
            % System Size
            if exist('n','var') == 0
                temp = f();
                n = temp(1);
                p = temp(2);
            end
            sys.n = n; % Number of States
            sys.p = p; % Number of Inputs
            
            % Output Equation
            sys.h = h;
            
            % Output Size
            if nargin >= 2
                temp = h(); %temp(1) = n, temp(2) = p, temp(3) = q
                if temp(1) == -1
                    temp(1) = sys.n;
                end
                if temp(2) == -1
                    temp(2) = sys.p;
                end
                if temp(1) ~= sys.n || temp(2) ~= sys.p
                    error('h() is not compatible with f()')
                end
                q = temp(3);
            else
                if exist('q','var') == 0
                    q = sys.n;
                end
            end
            sys.q = q;% Number of outputs
            
            % System State
            if x == -1 %either no input given for x or none specified w/ -1
                x = zeros(sys.n,1);
            end
            if size(x,1) ~= sys.n
                error('x incorrect size')
            end
            sys.x = x;
            
            
            % Validization
            try
                x_test = sys.f(x,zeros(sys.p,1));
            catch
                error('f(x,0) does not work')
            end
            if size(x_test,1) ~= sys.n
                error('f(x,0) incorrect size')
            end
            try
                y_test = sys.h(x,zeros(sys.p,1));
            catch ME
                rethrow(ME)
                error('h(x,0) does not work')
            end
            if sys.q ~= -1 && size(y_test,1) ~= sys.q
                error('h(x,0) incorrect size')
            end
        end
        
%         % System Interconnectivity
%         function sys = series(sys1, sys2)
%             f = @(x,u)[sys1.f(x,u), eye(sys
%         end
%         
        % Standard System Operation
        function dx = dx(sys,u,x)
            % DX - returns state update eq (dx)
            arguments
                % sys is the nonlin sys
                sys
                % u is the input
                u
                % x is the current state
                x = sys.x
            end
            dx = sys.f(x,u);
        end
        
        function y = y(sys,u,x)
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
            
        function  sys = update(sys,u,t,x)
            % UPDATE - return an updated system based on u and t...
            % and x (optional)
            arguments
                % sys is the nonlin sys
                sys
                % u is the input
                u (:,1) double
                % t is the time step to be updated by (assumign CT)
                t           double
                % x is the current state (optional) default = sys.x
                x (:,1) double = sys.x
            end
            
            % Input Validation
            if size(u,1) ~= sys.p
                error('u incorrect size')
            end
            if size(x,1) ~= sys.n
                error('x incorrect size')
            end
            
            % Update Equation
            x = x + sys.dx(u,x) * t;
            
            % New sys definition
            sys = nlsys(sys.f,@h_default,x);
        end

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


function y = h_default(x,u)
    % H_DEFAULT default h function
    arguments
        x = 0;
        u = 0;
    end
    % Simplistic output eq
    y = x;
    
    % Exception n,p,and q... for verification steps
    if nargin ==0
        y = [-1;-1;-1];
    end
end

function isLTI = IsLTI(sys)
    % IsLTI tests if a system is lti
    try
        ss = IsSS(sys);
        tf = IsTF(sys);
        zpk = IsZPK(sys);
        if ss || tf || zpk
            isLTI = true;
        else
            isLTI = false;
        end
    catch
        isLTI = false;
    end
end

function isSS = IsSS(sys)
    % IsSS tests if a system is ss
    try
        isSS = strcmp(class(sys), 'ss');
    catch
        isSS = false;
    end
end

function isTF = IsTF(sys)
    % IsTF tests if a system is tf
    try
        isTF = strcmp(class(sys), 'tf');
    catch
        isTF = false;
    end
end

function isZPK = IsZPK(sys)
    % IsZPK tests if a system is zpk
    try
        isZPK = strcmp(class(sys), 'zpk');
    catch
        isZPK = false;
    end
end

function isNLSYS = IsNLSYS(sys)
    % IsNLYS tests if a system is nlsys
    try
        isNLSYS = strcmp(class(sys), 'nlsys');
    catch
        isNLSYS = false;
    end
end
