function [y,T] = nlsim(sys, U, T, x0, t_step)
%   This is used for all the important nonlinear simulations
    arguments
        sys
        U
        T
        x0 = 0;
        t_step = -1;
    end
    
    % Correct input for U and T
    if size(U,1) < size(U,2)
        U = U';
    end
    if size(T,1) < size(T,2)
        T = T';
    end
    
    % Simulation Setup
    if t_step == -1
        t_step = T(3) - T(2);
        T_sim = T;
    else
        T_sim = T(1):t_step:T(end);
        t_step = T_sim(3) - T_sim(2);
    end
    
    % Simulation Initialize
    N = size(T_sim,2);
    x = x0;
        
    for i = 1:N
        t_sim = T_sim(i);
        u = interp1(T,U,t_sim);
        dx = sys(x,u);
        x = x + dx * t_step;
        X(i,:) = x;
    end
    
    y = interp1(T_sim,X,T);
    
end