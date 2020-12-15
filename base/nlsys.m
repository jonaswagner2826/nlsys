classdef nlsys
    %NLSYS This class describes non-linear dynamical systems for simulation
    %   Class developed to do similar things as the MATLAB control systems
    %   toolbox but for nonlinear state and output funactions
    
    properties
        % System Parameters
        % f - State_eq... linear or nonlinear function describing the
        % update of the state itself. Should be @(x,u)
        f
        % h - Output_equation... should be @(x,u)
        h
        % x - Current state of the system, should be of the correct size
        % to satisfy f(x,u)...
        x
        % Ts is the DT step size (-1 = CT) 
        Ts
        
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
        function sys = nlsys(f, h, x, Ts)
            % NLSYS this is the constructor for nlsys objects
            arguments
                % f is state eq (or nlsys or lti object)
                f       = 'empty';
                % h is output eq (optional) default = output just x
                h       = 'empty';
                % x is the current state (optional) default = relaxed
                x (:,1) = 0;
                % Ts is the DT step size (options) default = CT (Ts = -1)
                Ts      = -1;
            end
            
            if nargin == 0 % Empty Construction
                sys.Ts = -1; % default to CT
            else
                % Conversion setups
                if isa(f,'nlsys')% direct nlsys conversion
                    sys_old = f;
                    f = sys_old.f;
                    if nargin < 2
                        h = sys_old.h;
                    end
                    if nargin < 3 % not sure why this would ever be used...
                        x = sys_old.x;
                    end
                    if nargin < 4
                        Ts = sys_old.Ts;
                    end
                end

                if isa(f,'lti')% conversions from lti sys
                    [a,b,c,d] = ssdata(f);
                    f = f_lti_default(a,b);%@(x,u) a*x + b*u;
                    if nargin < 2
                        h = h_lti_default(c,d);%@(x,u) c*x + d*u;
                        q = size(c,1);
                    end
                end
                
                if isa(f,'double')
                    k = f;
                    f = f_lti_default(0,0);
                    h = h_lti_default(0,k);
                    q = size(k,1);
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
                if x == 0 %either no input given for x
                    x = zeros(sys.n,1);
                end
                if size(x,1) ~= sys.n
                    error('x incorrect size')
                end
                sys.x = x;
                
                % DT Step Size (default = -1 = CT)
                sys.Ts = Ts;

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
                    y_test = sys.h(x,0);
                catch
                    error('h(x,0) does not work')
                end
                if size(y_test,1) ~= sys.q
                    error('h(x,0) incorrect size')
                end
            end
        end
        
        % Standard System Operations
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
        
        function x = CT_Update(sys,u,t,x)
            arguments
                % sys is the nonlin sys
                sys nlsys
                % u is the input
                u
                % t is the time step
                t
                % x is the current state
                x = sys.x
            end            
            x = x + sys.dx(u,x) * t;
        end
        
        function x = DT_Update(sys,u,n,x)
            %DT_Update... does n foward steps
            arguments
                % sys is the nonlin sys
                sys nlsys
                % u is the input
                u
                % n is the number of time steps
                n (1,1) int32
                % x is the current state
                x = sys.x
            end
            x = sys.f(x,u);
            if n > 1
                for i = 2:n
                    x = sys.f(x,u);
                end
            end
        end
        
        function sys = update(sys,u,t,x)
            % UPDATE - return an updated system based on u and t...
            % and x (optional)
            arguments
                % sys is the nonlin sys
                sys nlsys
                % u is the input
                u (:,1)
                % t is the time step to be updated by (assuming CT)
                % if DT: result is x_{n+t}
                t = 0;
                % x is the current state (optional) default = sys.x
                x (:,1) = sys.x
            end
            % Acounting for symbolic explicitly
            if isa(t,'sym')
                if sys.Ts == -1
                    x = CT_Update(sys,u,t,x);
                    sys = nlsys(sys.f,sys.h,x);
                    return;
                else
                    error('DT not possible with symbolic t')
                end
            end
            
            % Input Validation
            if size(u,1) ~= sys.p
                error('u incorrect size')
            end
            if size(x,1) ~= sys.n
                error('x incorrect size')
            end
            if t <=0
                error('t must be greater then zero')
            end
            
            % Update Equation
            if sys.Ts == -1 % Standard CT update
                x = CT_Update(sys,u,t,x);
            else % DT update
                x = DT_Update(sys,u,t,x);
            end
            
            % New sys definition
            sys = nlsys(sys.f,sys.h,x);
        end
        
    end
    
    methods (Static)
        
        % Export to lti classes
        
        function [A,B,C,D] = linearize(sys, x_0, u_0)
            % LINEARIZE linearizes the system around x_0 and u_0
            arguments
                % sys is the nlsys object
                sys
                % x_0 is the equilibrium point... default is sys.x
                x_0 = 0;
                % u_0  is the equilibrium input... default is 0
                u_0 = 0;
            end
            [f, h, x, n, p, ~, ~] = nlsys.data_export(sys);
            
            if nargin < 2 || x_0 == -1
                x_0 = x;
            end
            if nargin < 3 || u_0 == -1
                u_0 = 0;
            end
            
            x_sym = sym('x',[n,1]);
            u_sym = sym('u',[p,1]);
            
            A = jacobian(f(x_sym,u_sym),x_sym);
            B = jacobian(f(x_sym,u_sym),u_sym);
            C = jacobian(h(x_sym,u_sym),x_sym);
            D = jacobian(h(x_sym,u_sym),u_sym);
            
            A = double(subs(subs(A,x_sym,x_0),u_sym,u_0));
            B = double(subs(subs(B,x_sym,x_0),u_sym,u_0));
            C = double(subs(subs(C,x_sym,x_0),u_sym,u_0));
            D = double(subs(subs(D,x_sym,x_0),u_sym,u_0));
        
        end
        
        function sys_new = ss(sys, x_0, u_0)
            % SS exports nlsys into an ss object
            arguments
                % sys is the nlsys object
                sys
                % x_0 is the equilibrium point... default is sys.x
                x_0 = -1;
                % u_0  is the equilibrium input... default is 0
                u_0 = -1;
            end
            if sys.Ts ~= -1
                error('function no work in DT')
            end

            [A,B,C,D] = nlsys.linearize(sys, x_0, u_0);
            sys_new = ss(A,B,C,D);
        end

        function sys_new = tf(sys, x_0, u_0)
            % TF exports nlsys into an tf object
            arguments
                % sys is the nlsys object
                sys
                % x_0 is the equilibrium point... default is sys.x
                x_0 = -1;
                % u_0  is the equilibrium input... default is 0
                u_0 = -1;
            end
            if sys.Ts ~= -1
                error('function no work in DT')
            end

            [A,B,C,D] = nlsys.linearize(sys, x_0, u_0);
            sys_new = tf(ss(A,B,C,D));
        end
        
        function sys_new = zpk(sys, x_0, u_0)
            % ZPK exports nlsys into an zpk object
            arguments
                % sys is the nlsys object
                sys
                % x_0 is the equilibrium point... default is sys.x
                x_0 = -1;
                % u_0  is the equilibrium input... default is 0
                u_0 = -1;
            end
            if sys.Ts ~= -1
                error('function no work in DT')
            end

            [A,B,C,D] = nlsys.linearize(sys, x_0, u_0);
            sys_new = zpk(ss(A,B,C,D));
        end
        
        % Interconnected Systems
        function sys = series(sys1,sys2)
            % SERIES combines two nlsys objects in series
            
            % System Parameters
            [f1, h1, x1, n1, p1, q1, Ts1] = nlsys.data_export(sys1);
            [f2, h2, x2, n2, p2, q2, Ts2] = nlsys.data_export(sys2);
            
            % Compatibility 
            if q1 ~= p2
                error('sys1 and sys2 incompatible');
            end
            if Ts1 ~= Ts2
                error('sys1 and sys2 Ts different');
            end

            % Sys definition
            new_x = [x1;x2];
            n = n1 + n2;
            p = p1;
            q = q2;
            Ts = Ts1;

            sys = nlsys.data_input(@new_f, @new_h, new_x, n, p, q, Ts);
            
            function f = new_f(x,u)
                % NEW_F function defining new series state function
                if nargin == 0
                    f = [n,p];
                else
                    x1 = x(1:n1);
                    x2 = x((n1+1):(n1+n2));
                    dx1 = f1(x1,u);
                    y1 = h1(x1,u);
                    dx2 = f2(x2,y1);
                    f = [dx1; dx2];
                end
            end
            
            function h = new_h(x,u)
                % NEW_H function defining new series output function
                if nargin == 0
                    h = [n,p,q];
                else
                    x1 = x(1:n1);
                    x2 = x((n1+1):(n1+n2));
                    y1 = h1(x1,u);
                    y2 = h2(x2,y1);
                    h = y2;
                end
            end
                        
        end
        
        function sys = parrellel(sys1,sys2)
            % PARRELLEL combines two nlsys objects in parrellel... assumes
            % that they have the same input and an un scaled sum outputs
            
            % System Parameters
            [f1, h1, x1, n1, p1, q1, Ts1] = nlsys.data_export(sys1);
            [f2, h2, x2, n2, p2, q2, Ts2] = nlsys.data_export(sys2);   
            
            % Compatibility 
            if p1 ~= p2 || q1 ~= q2 %sys1.p ~= sys2.p || sys1.q ~= sys2.q
                error('sys1 and sys2 incompatible');
            end
            if Ts1 ~= Ts2 %sys1.Ts ~= sys2.Ts
                error('sys1 and sys2 Ts different');
            end
            
            % Sys definition
            new_x = [x1;x2];
            n = n1 + n2;
            p = p1;
            q = q2;
            Ts = Ts1;

            sys = nlsys.data_input(@new_f, @new_h, new_x, n, p, q, Ts);            

            function f = new_f(x,u)
                % NEW_F function defining new parrellel state function
                if nargin == 0
                    f = [n,p];
                else
                    x1 = x(1:n1);
                    x2 = x((n1+1):(n1+n2));
                    dx1 = f1(x1,u);
                    dx2 = f2(x2,u);
                    f = [dx1; dx2];
                end
            end
            
            function h = new_h(x,u)
                % NEW_H function defining new series output function
                if nargin == 0
                    h = [n,p,q];
                else
                    x1 = x(1:n1);
                    x2 = x((n1+1):(n1+n2));
                    y1 = h1(x1,u);
                    y2 = h2(x2,u);
                    h = y1+y2;
                end
            end
        end
        
        % might impliment in seperate class (I don't like that idea though)
        % -------- Not done ----------------------
        function sys = feedback(sys1,sys2)
            % this doesn't work yet ****************************
            % FEEDBACK combines two nlsys objects into a closed-loop model:
%             
%            u --->O---->[ M1 ]----+---> y
%                  |               |         
%                  +-----[ M2 ]<---+
% 
            % Negative feedback is assumed, so modify acoridingly
            
            % Compatibility 
            if sys1.p ~= sys2.q || sys1.q ~= sys2.p
                error('sys1 and sys2 incompatible');
            end
            if sys1.Ts ~= sys2.Ts
                error('sys1 and sys2 Ts different');
            end
            
            
            % System Parameters
            f1 = sys1.f;
            f2 = sys2.f;
            h1 = sys1.h;
            h2 = sys2.h;
            n1 = sys1.n;
            n2 = sys2.n;
            
            % Sys definition
            sys = nlsys();
            sys.f = @new_f;
            sys.h = @new_h;
            sys.x = [sys1.x; sys2.x];
            
            n = sys1.n + sys2.n;
            p = sys1.p;
            q = sys2.q;
            
            sys.n = n;
            sys.p = p;
            sys.q = q;
            
            sys.Ts = sys1.Ts
            
            
            function f = new_f(x,u)
                % NEW_F function defining new series state function
                if nargin == 0
                    f = [n,p];
                else
                    x1 = x(1:n1);
                    x2 = x((n1+1):(n1+n2));
%                     y1 = h1(x1,u);
%                     dx1 = f1(x1,u);
%                     dx2 = f2(x2,y1);
%                     f = [dx1; dx2];
                end
                % i don't know what the correct way to do this is
            end
            
            function h = new_h(x,u)
                % NEW_H function defining new series output function
                if nargin == 0
                    h = [n,p,q];
                else
                    x1 = x(1:n1);
                    x2 = x((n1+1):(n1+n2));
%                     y1 = h1(x1,u);
%                     y2 = h2(x2,y1);
%                     h = y2;
                end
            end
        end
        % -----------------------------------------
         
        % Data Inport/Export
        function [f, h, x, n, p, q, Ts] = data_export(sys)
            % DATA_EXPORT output all the system properties
            f = sys.f;
            h = sys.h;
            x = sys.x;
            n = sys.n;
            p = sys.p;
            q = sys.q;
            Ts = sys.Ts;
        end
        
        function sys = data_input(f, h, x, n, p, q, Ts)
            % DATA_INPUT creates a new nlsys and inputs specific parameters
            sys = nlsys();
            sys.f = f;
            sys.h = h;
            sys.x = x;
            sys.n = n;
            sys.p = p;
            sys.q = q;
            sys.Ts = Ts;
        end
        
        % Anonomous Function Manipulation
        function f = func_sum(varargin)
            % FUNC_SUM sums two varible functions together:
            % Ex: f1(x,u) + f2(x,u) + f3(x,u)
            f = @(x,u) varargin{1}(x,u);
            for i = 2:nargin
                f = @(x,u) f(x,u) + varargin{i}(x,u);
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
        
        % -------- Not done ----------------------
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
            % FUNC_APPEND appends two varible functions horrizontally:
            % Ex: [f1(x,u), f2(x,u), f3(x,u)]
            f = @(x,u) varargin{1}(x,u);
            for i = 2:nargin
                f = @(x,u) [f(x,u), varargin{i}(x,u)];
            end
            f = @(x,u) f(x,u);
        end
        % -----------------------------------------
        
        
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


