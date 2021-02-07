% NLSYS testing
% Jonas Wagner 2020-12-11
clear
close all

% Basic nlsys constroctor with a nonlinear state equation
sys = nlsys(@nonlin_state_func);


% nlsys constructor with nonlinear state and output equations and initial
% condition
x_0 = [1,2];
sys2 = nlsys(@nonlin_state_func,@nonlin_output_func,x_0,1);

% nlsys update operations... I wonder if using handle class would be better
% so that (in this example) the reference sys2 would be directly updated
% with the update method, but this is how it is done in the control systems
% toolbox...
u_test = 1;
t_test = 1;
sys2.dx(u_test);
sys3 = sys2.update(u_test,t_test);
x_3 = sys3.x;



% nlsys from ss object
A_test = eye(2);
B_test = [1; 0];
C_test = [1, 0];
D_test = 1;
sys_ss = ss(A_test,B_test,C_test,D_test);
sys4 = nlsys(sys_ss);

% nlsys from tf object
sys_tf = tf([1],[1 1 1]);
sys5 = nlsys(sys_tf);

% nlsys from double
sys10 = nlsys([1;2]);



% testing interconnectivity functions ----------------------------------

% func_sum
f1 = sys.f;
f2 = sys5.f;
nlsys.func_sum(f1,f2);
nlsys.func_sum(f1,f2,f1,f2);

% func_conv
f1 = @(x) x^2;
f2 = @(x) x^3;
f3 = nlsys.func_conv(f1,f2,f1);
syms x
f3_x = f3(x);

% func_append ---- doesn't work
f1 = @(x,u) x^2 + u;
f2 = @(x,u) u;
f3 = nlsys.func_append(f1,f2);
syms x u
f4_x = f3(x,u);


% Simple nlsys connections ----------------

% series
sys4.x = [1;2];
sys6 = nlsys.series(sys4,sys4);
sys6 = sys4 * sys4;
sys60 = 4 * sys4;

syms x1 x2 x3 x4 u t
sys6.x = [x1; x2; x3; x4];
sys7 = sys6.update(u,t);
sys7_x = sys7.x;

% parrellel
sys4.x = [1;2];
sys8 = nlsys.parrellel(sys4,sys4);
sys8 = sys4 + sys4;
sys80 = 4 + sys4;

syms x1 x2 x3 x4 u t
sys8.x = [x1; x2; x3; x4];
sys9 = sys8.update(u,t);
sys9_x = sys9.x;

% LTI Export Testing -----------------------
% ss
[A,B,C,D] = nlsys.linearize(sys6);
sys10 = nlsys.ss(sys6);

% tf
sys11 = nlsys.tf(sys6);

% zpk
sys12 = nlsys.zpk(sys6);


% Controller Implimentation --------------
% pid (this is siso... series does too)
k_p = 1;
k_i = 0.1;
k_d = 0;

pid_test = pid(k_p,k_i,k_d);
pid_nlsys = nlsys(pid_test);


% Feedback ------------------------

sys13 = nlfeedback(sys4,sys4);
sys13_x = sys13.x;
sys14 = sys13.update(0,1);
sys14_x = sys14.x;



% Simulation ---------------------
N = 100;
t_step = 0.1;
t_max = N * t_step - t_step;
T = reshape(0:t_step:t_max,N,1);
u_0= 2;
U = u_0* abs(sin(T));
SYS = nlsys.nlsim(sys4,U,T,x_0)


% Local Functions
function y = nonlin_state_func(x,u)
    % NONLIN_FUNC Example function for use with nlsys based simulation
    arguments
        x (2,1) = [0; 0];
        u (1,1) = 0;
    end
    
    % Array sizes
    n = 2; % Number of states
    p = 1; % Number of inputs

    % State Update Equations
    y(1,1) = 2 * x(1) + x(2) * u;
    y(2,1) = x(1) * x(2);
    
    if nargin ==0
        y = [n;p];
    end
end

function y = nonlin_output_func(x,u)
    % NONLIN_OUTPUT_FUNC Example function for use with nlsys
    arguments
        x (2,1) = [0; 0];
        u (1,1) = 0;
    end
    
    % Array sizes
    n = 2; % Number of states
    p = 1; % Number of inputs
    q = 2; % Number of outputs
    
    % State Update Equations
    y(1,1) = x(1) + u;
    y(2,1) = x(2) * u;
    
    if nargin ==0
        y = [n;p;q];
    end
end