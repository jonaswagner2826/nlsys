classdef nlsim
    %NLSIM - This class is used to simulate nlsys systems similar to lsim
    %in the MATLAB controls toolbox.
    properties
        % SYS - array of nlsys objects
        SYS (:,1)
    end
    
    
    methods
        function sys_sim = nlsim(sys, U, T, x_0)
            % NLSIM simulates the response of an nlsys given input U at over
            % the time T. It then outputs an array of nlsys objects that
            % contain the state of the system at each point of simulated T.
            % If it is a DT system, SYS will be adjusted to export thoose
            % time steps instead
            arguments
                % sys - nlsys object to be simulated
                sys
                % U - Input to the system at time t (transposed u...)
                U double
                % T - Time of input (and output if CT) to the system
                T (:,1) double
                % x_0 - Initial state of the system
                x_0 = 0;
            end

            % Simulation Setup
            if sys.Ts == -1
                T_sim = T;
            else
                error('Ts ~= -1 is not supported yet')
        %                 T_sim = T(1):t_step:T(end); % this may or may not work...
        %                 t_step = T_sim(3) - T_sim(2);
            end
            if T(1) ~= 0
                error('T must begin at 0')
            end

            % Simulation Initialize
            N = size(T_sim,1);
            if x_0 ~= 0
                try
                    sys.x = x_0;
                catch
                    error('issue with x_0 size')
                end
            end
            if sys.t ~= 0
                error('not setup for t ~= 0 yet')
            end
            SYS = nlsys.empty(0,1);
            SYS(1) = sys;
            t_sim = 0;

            for i = 2:N
                t_sim_old = t_sim;
                t_sim = T_sim(i);
                t_delta = t_sim - t_sim_old;
                u = interp1(T,U,t_sim)'; %interpreted based on input... u'
                SYS(i) = SYS(i-1).update(u,t_delta);
            end
            
            sys_sim.SYS = SYS;
            
        end
        
    end
    
    methods
        function X = X(sys)
            % X - export all states as an array
            X = cell2mat({sys.SYS(:).x});
        end
        
        function U = U(sys)
            % U - export all inpus as an array
            U = cell2mat({sys.SYS(:).u});
        end
        
        function T = T(sys)
            % T - export all states as an array
            T = cell2mat({sys.SYS(:).t});
        end
        
        function SS = ssModel(sys)
            % SSMODEL - exports all linearized state matrices at the point
            % of the simulation
            SS = zeros(size(sys,1),1);
            for i = 1:size(sys,1)
                SS(i) = sys.SYS(i).ss;
            end
        end
        
    end
end