classdef nlcomposite < nlsys
    %NLCOMPOSITE combines two nlsys objects into a single model.
    % Used for simulating more complicated nlsys models

    properties
        % sys1 - first nlsys
        sys1
        % sys2 - secound nlsys
        sys2
    end
    
    methods
        function sys = nlcomposite(sys1, sys2, t)
            % NLCOMPOSITe is the constructor for an nlcomposite
            arguments
                % sys1 - first system
                sys1
                % sys2 - secound system
                sys2
                % t - current time (optional) default = 0
                t = 0;
            end

            % Input validation
            if ~ isa(sys1,'nlsys')
                try
                    sys1 = nlsys(sys1);
                catch
                    error('sys1 not compatible')
                end
            end
            if ~ isa(sys2,'nlsys')
                try
                    sys2 = nlsys(sys2);
                catch
                    error('sys1 not compatible')
                end
            end
            
            if sys1.Ts ~= -1 || sys2.Ts ~= -1
                if sys1.Ts ~= sys2.Ts
                    sys.Ts = sys1.Ts * sys2.Ts;
                else
                    sys.Ts = sys1.Ts;
                end
                sys.Ts = -1;
            end
            
            % System definition
            sys.sys1 = sys1;
            sys.sys2 = sys2;
            sys.x = [sys1.x; sys2.x];
            sys.n = sys1.n + sys2.n;
            sys.p = sys1.p + sys2.p;
            sys.q = sys1.q + sys2.q;
            
            sys.f = @standard_error;
            sys.h = @standard_error;
        end
    end
    
    % Regular Overload Functions
    methods
        function disp(sys)
            fprintf('\n Sys1:\n')
            disp(sys.sys1)
            fprintf('\n Sys2:\n')
            disp(sys.sys2)
            fprintf('Composite Nonlinear System \n')
        end
    end
    
    % Composite overload functions
    methods
        function standard_error(varagin)
            error('does not work like this... composite')
        end
        
        
    end
    
    methods (Abstract)
        %shouldn't be needed... just calls for series and parrellel
%         sys = plus(sys1,sys2);
%         sys = mtimes(sys1,sys2); 
%         sys = times(sys1,sys2);
        % doesn't seem that easy to do these...
%         sys = series(sys,sys2);
%         sys = parrellel(sys1,sys2);
    end
    
    % Standard Operation
    methods (Abstract)
        sys = update(sys,u,t,x);
        
        [A,B,C,D] = linearize(sys,x_0,u_0);
        
    end
end

