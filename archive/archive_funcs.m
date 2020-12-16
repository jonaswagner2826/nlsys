% Archive functions...

function isFunc = IsFunc(f)
    % IsSS tests if a system is ss
    try
        isFunc = strcmp(class(f), 'function_handle');
    catch
        isFunc = false;
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

    % System Parameters
    [f1, h1, x1, n1, p1, q1, Ts1] = nlsys.data_export(sys1);
    [f2, h2, x2, n2, p2, q2, Ts2] = nlsys.data_export(sys2);

    % Compatibility 
    if q1 ~= p2 || q2 ~= p1
        error('sys1 and sys2 incompatible');
    end
    if Ts1 ~= Ts2
        error('sys1 and sys2 Ts different');
    end

    % Sys definition
    new_x = [x1;x2];
    n = n1 + n2;
    p = p1;
    q = q1;
    Ts = Ts1;


%             [f_new, h_new] = solve_feedback(f1,h1,f2,h2);


    syms u1 dx1 dx2 y1 y2

    x1_sym = sym('x1',[n1,1]);
    x2_sym = sym('x2',[n2,1]);
    r = sym('r',[p1,1]);

    eq1 = u1 == r - y2;
    eq2 = dx1 == f1(x1_sym,r - y2);
    eq3 = dx2 == f2(x2_sym,y1);
    eq4 = y1 == h1(x1_sym,r - y2);
    eq5 = y2 == h2(x2_sym,y1);


    [~, dx1, dx2, y1, y2] = solve([eq1, eq2, eq3, eq4, eq5],...
        [r, x1_sym, x2_sym]);


    x1 = x(1:n1);
    x2 = x((n1+1):(n1+n2));
%             dx1 = subs(dx1,[r, x1_sym, x2_sym],[u,x1,x2]);
%             dx2 = subs(dx2,[r, x1_sym, x2_sym],[u,x1,x2]);

    f_new = @(x,u)[dx1; dx2];
    h_new = @(x,u)[y1;y2];

    sys = nlsys.data_input(f_new, h_new, new_x, n, p, q, Ts);
end
% -----------------------------------------

function f = func_append_h(varargin)
    % FUNC_APPEND appends two varible functions horrizontally:
    % Ex: [f1(x,u), f2(x,u), f3(x,u)]
    f = @(x,u) varargin{1}(x,u);
    for i = 2:nargin
        f = @(x,u) [f(x,u), varargin{i}(x,u)];
    end
    f = @(x,u) f(x,u);
end