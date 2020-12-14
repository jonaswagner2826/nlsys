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