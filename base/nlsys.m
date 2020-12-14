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
                h               = -1;
                % x is the current state (optional) default = relaxed
                x (:,1) double = -1;
            end
            
            % Conversion setups
            if isa(f,'nlsys')% direct nlsys conversion
                if nargin < 2
                    h = f.h;
                end
                if nargin < 3 % not sure why this would ever be used...
                    x = f.x;
                end
                f = f.f;
            end
            
            if isa(f,'lti')% conversions from lti sys
                [a,b,c,d] = ssdata(f);
                f = f_lti_default(a,b);%@(x,u) a*x + b*u;
                if nargin < 2
                    h = h_lti_default(c,d);%@(x,u) c*x + d*u;
                    q = size(c,1);
                end
            end
            
            % State Equation
            sys.f = f;
            
            % System Size
            try
                temp = f();
            catch
                warning('size of f errors')
                n = -1;
                p = -1;
            end
            if exist('n','var') == 0
                n = temp(1);
                p = temp(2);
            end
            sys.n = n; % Number of States
            sys.p = p; % Number of Inputs
            
            % Output Equation
            if ~ isa(h,'function_handle')
                if isa(f,'nlsys')
                    h = f.h;
                else
                    h = h_default(n,p);
                end
            end
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
            elseif exist('q','var') == 0
                q = sys.n;
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
            catch
                error('h(x,0) does not work')
            end
            if size(y_test,1) ~= sys.q
                error('h(x,0) incorrect size')
            end
        end
        
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
            sys = nlsys(sys.f,sys.h,x);
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
    
    methods (Static)
        % Interconnected Systems
        function sys = series(sys1,sys2)
            % SERIES combines two nlsys objects in series
            sys = sys1; % obviously not correct... just a placeholder
            f1 = sys1.f;
            f2 = sys2.f;
            h1 = sys1.h;
            h2 = sys2.h;
            %not done... need to do math to figure out how these go
            %together
        end
        
        function sys = parrellel(sys1,sys2)
            % PARRELLEL combines two nlsys objects in parrellel
            %sys = sys1; % obviously not correct... just a placeholder
            f1 = sys1.f;
            f2 = sys2.f;
            h1 = sys1.h;
            h2 = sys2.h;
            f = nlsys.func_append(f1,f2);
            h = nlsys.func_append_h(h1,h2); % its isn't this simple... have to figure out how to modify h2 to grab from the secound set of states...
            x = [sys1.x; sys2.x];
            
            sys = nlsys(f,h,x);
%             syms x u
%             f = func_sum(sys1.f, sys2.f)
%             h = matlabFunction(sys1.h(x,u), sys2.h(x,u))%[sys1.h; sys2.h];
%             x = [sys1.x; sys2.x];
%             sys = nlsys(f,h,x);
        end
        
        function sys = feedback(sys1,sys2)
            % FEEDBACK combines two nlsys objects in parrellel
            sys = sys1; % obviously not correct... just a placeholder
        end
        
        function f = func_sum(varargin)
            % FUNC_SUM sums two varible functions together:
            % Ex: f1(x,u) + f2(x,u) + f3(x,u)
            f = @(x,u) varargin{1}(x,u);
            for i = 2:nargin
                f = @(x,u) f(x,u) + varargin{i}(x,u);
            end
            f = @(x,u) f(x,u);
        end
        
        function f = func_append(varargin)
            % FUNC_APPEND appends two varible functions virtically:
            % Ex: [f1(x,u); f2(x,u); f3(x,u)]
            f = @(x,u) varargin{1}(x,u);
            for i = 2:nargin
                f = @(x,u) [f(x,u); varargin{i}(x,u)];
            end
            f = @g;%(x,u) f(x,u);
            
            function y = g(x,u)
                y = f(x,u); % need to add another nargin term...
            end
       end
        
        function f = func_append_h(varargin)
            % FUNC_APPEND appends two varible functions virtically:
            % Ex: [f1(x,u); f2(x,u); f3(x,u)]
            f = @(x,u) varargin{1}(x,u);
            for i = 2:nargin
                f = @(x,u) [f(x,u), varargin{i}(x,u)];
            end
            f = @(x,u) f(x,u);
        end
        
        function func = func_conv(varargin)
            % FUNC_CONV convolves multiple single varible functions
            f = @(x) varargin{1}(x);
            for i = 2:nargin
                f = @(x) varargin{i}(f(x));
            end
            func = @g;
            function y = g(x)
                y = f(x);
            end
        end
    end
end


function h = h_default(varargin)
    h = @h_default_func;
    local_n = varargin{1};
    local_p = varargin{2};
    function y = h_default_func(varargin)
        % H_DEFAULT default h function
        if nargin == 0
            y = [local_n; local_p; local_n];
        else
            local_x = varargin{1};
            local_u = varargin{2}; % Unused
            y = local_x;
        end
    end
end

function f = f_lti_default(A,B)
    f = @lti_state_func;

    % Array sizes
    local_n = size(A,1); % Number of states
    local_p = size(B,2); % Number of inputs

    function y = lti_state_func(varargin)
        % LTI_STATE_FUNC function developed using state matrices
        if nargin == 0
            y = [local_n; local_p];
        else
            local_x = varargin{1};
            local_u = varargin{2};
            if size(local_x) ~= [local_n,1]
                error('x wrong size')
            end
            if size(local_u) ~= [local_p,1]
                error('u wrong size')
            end
            y = A * local_x + B * local_u;
        end
    end
end

function h = h_lti_default(C,D)
    h = @lti_output_func;

    % Array sizes
    local_n = size(C,1); % Number of states
    local_p = size(D,2); % Number of inputs

    function y = lti_output_func(varargin)
        % LTI_STATE_FUNC function developed using state matrices
        if nargin == 0
            y = [local_n; local_p; local_n];
        else
            local_x = varargin{1};
            local_u = varargin{2};
            if size(local_x) ~= [local_n,1]
                error('x wrong size')
            end
            if size(local_u) ~= [local_p,1]
                error('u wrong size')
            end
            y = C * local_x + D * local_u;
        end
    end
end


