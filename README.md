# nlsys
Nonlinear Dynamical System Modeling for MATLAB

Note: 2020-12-13  
This is a work in progress that is desigened to ultimently work as a nonlinear equivelent to the control systems toolbox in MATLAB. This will eventually be able to encorporate the functionality of lti classes from that toolbox and hopefully be integratable into simulink.

2020-12-15  
This is now generally functional with integration with matlab lti blocks and is able to represent composite structures of nonlinear systems (including a rudementry feedback loop). Next up is simulation of them.

2020-12-18  
A simulation class (nlsim) was defined that is bassically an array of nlsys objects for different times given an input (so the definition is a function generating them). It also has ploting functionality (which is obviously nice...). Issues still remain with feedback.

2021-05-28
After multiple updates throughout the last few months (for use in the MECH 6313 course... see my other GitHub project), the project has been pretty much abandoned to instead use Simulink (or just simple ode45/DT sims). This project is no longer being maintained and is still useful, but a stable version should be used instead of the most recent 'work-in-progress' versions.
