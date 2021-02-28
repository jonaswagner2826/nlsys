classdef nlsim
    %NLSIM - This class is used to simulate nlsys systems similar to lsim
    %in the MATLAB controls toolbox.
    properties
        % SYS - array of nlsys objects
        SYS (:,1)
    end
    
    
    %% Constructor/Simulation
    methods
        function sys_sim = nlsim(sys, U, T, x_0, parms, solver)
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
                % parms - Parameters of nonlin eq
                parms = false
                % solver - solver method
                solver = 'default'
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
            
            if size(U,1) ~= size(T,1)
                U = U'; %array of U'
            end
            

            % Simulation Initialize
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
            
            if strcmp(solver, 'default')
                SYS = sim_default(sys, U, T, parms);
            elseif strcmp(solver, 'ode45')
                SYS = sim_ode45(sys, U, T, parms);
            else
                error('solution method does not exist')
            end
                
            
            sys_sim.SYS = SYS;
            

        end
    end
    
    %% Data Export
    methods
        function X = GetX(sys,state)
            % X - export all states as an array
            X = cell2mat({sys.SYS(:).x});
            if nargin > 1
                X = X(:,state);
            end
        end
    end
    methods (Static)
        function X = X(sys,state)
            % X - export all states as an array
            X = cell2mat({sys.SYS(:).x});
            if nargin > 1
                X = X(state,:);
            end
        end
        
        function U = U(sys,input)
            % U - export all inpus as an array
            try
                U = cell2mat({sys.SYS(:).u});
            catch
                try
                    U = cell2mat({sys.SYS(:).u}');
                catch
                    error('U issues... likely transpose issues')
                end
            end
            if nargin > 1
                U = U(input,:);
            end
        end
        
        function T = T(sys)
            % T - export all states as an array
            T = cell2mat({sys.SYS(:).t});
        end
        
%         idk whats wrong here...
        function Y = Y(sys,output)
            % Y - export all states as an array
            Y = cell2mat({sys.SYS(:).y});
            if nargin > 1
                Y = Y(output,:);
            end
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
    
    %% Ploting
    methods
        function [fig, axes] = plot(sys, states, inputs, outputs,fig,k,j)
            % Plots the States and Inputs of the system vs time
            arguments
                % SYS - nlsim object
                sys
                % STATES - states to plot (optional) default all            
                % 0 means plot none
                % -1 means all
                states int32 = -1;
                % INPUTS - inputs to plot (optional) default all
                % 0 means plot none
                % -1 means all
                inputs int32 = -1;
                % OUTPUS - outputs to plot (optional) defaults all
                % 0 means plot none
                % -1 means all
                outputs int32 = -1;
                % fig
                fig = -1;
                % k - subplot width total
                k = 1;
                % j - subplot width start
                j = 1;
            end
            
            % Plotting setup
            numAxes = 3 - nnz(~[states,inputs,outputs]);
            if fig == -1
                fig = figure();
                sgtitle(strcat('nlsim plot of',32,inputname(1)));
            end
            
            % Input conditioning
            if states == -1
                states = 1:(sys.SYS(1).n);
            end
            if inputs == -1
                inputs = 1:(sys.SYS(1).p);
            end
            if outputs == -1
                outputs = 1:(sys.SYS(1).q);
            end
            
            T = sys.T;
            if states ~= 0
                axes(j) = subplot(numAxes,k,j); j = j+k;
                hold on
                str_legend = [];
                for i = states
                    plot(T,sys.X(i));
                    str_legend = [str_legend,string(i)];
                end
                hold off;
                legend(str_legend);
                title('System States')
            end
            
            if inputs ~= 0
                axes(j) = subplot(numAxes,k,j); j = j+k;
                hold on
                str_legend = [];
                for i = inputs
                    plot(T,sys.U(i));
                    str_legend = [str_legend,string(i)];
                end
                hold off
                legend(str_legend);
                title('System Inputs')
            end
            
            if outputs ~= 0
                axes(j) = subplot(numAxes,k,j); j = j+k;
                hold on
                str_legend = [];
                for i = outputs
                    if size(T,2) ~= size(sys.Y(i))
                        T = T(1:end-1);
                    end
                    plot(T,sys.Y(i));
                    str_legend = [str_legend,string(i)];
                end
                hold off
                legend(str_legend);
                title('System Outputs')
            end
        end
        
        
        
        function [fig, ax] = phasePlot(sys,state_1,state_2,plotName,fig,ax)
            % Plots the States and Inputs of the system vs time
            arguments
                % SYS - nlsim object
                sys
                % STATE_1 - state on the X-axis (optional) default (1)
                state_1 = 1;
                % STATE_2 - state on the Y-axis (optional) default (2)
                state_2 = 2;
                % Axes Title
                plotName = 'Phase Plot';
                % fig
                fig = -1;
                % ax
                ax = -1;
            end
            
            % Plotting setup
            if fig == -1
               fig = figure();
            end
            if ax == -1
                ax = axes();
            end
            
            X1 = sys.X(sys,state_1);
            X2 = sys.X(sys,state_2);
            
            plot(X1,X2)
            xlabel(['x',num2str(state_1)])
            ylabel(['x',num2str(state_2)])
            title(plotName)
        end
        
        
    end
    
end

%% Simulation Functions
function SYS = sim_default(sys, U, T, parms)
    N = size(T,1);
    SYS = [sys];
    t_sim = 0;
    for i = 2:N
        t_sim_old = t_sim;
        t_sim = T(i);
        t_delta = t_sim - t_sim_old;
        u = interp1(T,U,t_sim)'; %interpreted based on input... u'
        if parms == false
            SYS(i) = SYS(i-1).update(u,t_delta);
        else
            SYS(i) = SYS(i-1).update(u,t_delta,parms);
        end
    end
end


function SYS = sim_ode45(sys, U, T, parms)
    f = sys.f;
    x_0 = sys.x;
    warning('no input coded...')
%     options = odeset('RelTol',1e-6,'AbsTol',1e-9);
    
    [t,X] = ode15s(@(t,y) func_ode45(t,y), T, x_0);
    
    N = size(t,1);
    for i = 1:N
        SYS(i) = nlsys(f, sys.h, X(i,:), sys.Ts, t(i), U(i), parms);
    end
    
    
    function f_new = func_ode45(t,x)
        u = interp1(T,U,t);
        if parms == false
            f_new = f(x,u);
        else
            f_new = f(x,u,parms);
        end
    end
end

