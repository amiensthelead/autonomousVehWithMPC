%% hasankaantuna // 19015036 // Main Script
clear all
clc

%% Defining the constant values

% Veh Params
m=1575;
Iz=2875;
Caf=19000;
Car=33000;
lf=1.2;
lr=1.6;

% Controller Params
Ts=0.1; 
totalT=40;
hz =15; %prediction horizon
cz=5;  %control horizon
x_dot=15;
numberControlledStates=2;
velReassing=0; %experimental feature, makes change on A matrix of both MPC and Plant Model.
velReassingSetSpeed=x_dot; %experimental feature
trajectoryType=1; % pick one or two.

%% Calling reference creator script

t = 0:Ts:totalT;
if trajectoryType==1
    [psi_ref,X_ref,Y_ref]=trajectoryCreator(t,x_dot,trajectoryType);
else
    load trajTwo.mat
end
sim_length=length(t);

refSignals=zeros(length(X_ref(:,2))*numberControlledStates,1);

k=1;
for i = 1:numberControlledStates:length(refSignals)
    refSignals(i)=psi_ref(k,2);
    refSignals(i+1)=Y_ref(k,2);
    k=k+1;
end

clear i k

%% Initial state definitions

y_dot=0;
psi=psi_ref(1,2);
psi_dot=0;

if trajectoryType==2
    Y=0;
else
    Y=Y_ref(1,2);
end

states=[y_dot,psi,psi_dot,Y];
statesTotal=zeros(length(t),length(states));
statesTotal(1,:)=states;

%% Initiate the controller - simulation loops

delta=0;
dutotal=zeros(length(t),1);
deltaTotal=zeros(length(t),1);
deltaTotal(1,:)=delta;

psi = states(2);
Y=states(4);

%% Definitions of State Space Matrices

scriptA=[-(2*Caf+2*Car)/(m*x_dot) 0 (-x_dot-(2*Caf*lf-2*Car*lr)/(m*x_dot)) 0; ...
    0 0 1 0; ...
    -(2*lf*Caf-2*lr*Car)/(Iz*x_dot) 0 -(2*lf^2*Caf+2*lr^2*Car)/(Iz*x_dot) 0; ...
    1 x_dot 0 0];
scriptB=[2*Caf/m;0;2*lf*Caf/Iz;0];
C= [0 1 0 0;0 0 0 1];
D=[0; 0];

%% Definition of Discrete State Space Model

Ad=eye(length(scriptA(1,:)))+Ts*scriptA;
Bd=Ts*scriptB;
Cd=C;
Dd=D;

%%
[Hdb,Fdbt,Cdb,Adc]=costFuncScript(Ad,Bd,Cd,Dd,hz);
l=1;
k=1;
countie=0;
o=hz;
for i =1:sim_length-1


    % Generating the current state and the reference vector
    x_aug_t=[states';delta];

    k=k+numberControlledStates;

    if k+numberControlledStates*hz-1 <= length(refSignals)
        r=refSignals(k:k+numberControlledStates*hz-1);
    else
        r=refSignals(k:length(refSignals));
        hz=hz-1;
    end

    if hz<o
        [Hdb,Fdbt,Cdb,Adc]=costFuncScript(Ad,Bd,Cd,Dd,hz);
    end
    
        % Calling the optimizer (quadprog) // Cost function in quadprog: min(du)*1/2*du'Hdb*du+f'du

        ft=[x_aug_t',r']*Fdbt;

        if l==1
        % Call the solver
        
        countie=countie+1;
        options = optimset('Display', 'on');
        [du,fval]=quadprog(Hdb,ft,[],[],[],[],[],[],[],options);
        dutotal(i+1,:)=delta+du(1);
        % Update the real inputs
        if length(du)<cz
            changeOfDeltaStorage=du;
        else
        changeOfDeltaStorage=du(1:cz);
        end
            delta=delta+du(1);
            if delta < -pi/6
                delta=-pi/6;
            elseif delta>pi/6
                delta=pi/6;
            else
                delta=delta;
            end
        l=l+1;
        deltaTotal(i+1,:)=delta;
            
        else 
            delta=delta+changeOfDeltaStorage(l);
            if delta < -pi/6
                delta=-pi/6;
            elseif delta>pi/6
                delta=pi/6;
            else
                delta=delta;
            end
            l=l+1;
            deltaTotal(i+1,:)=delta;
            if l==cz+1
                l=1;
            end
        end

    
    % Assign the states

    y_dot=states(1);
    psi = states(2);
    psi_dot = states(3);

    % The nonlinear equation describing the dynamics of the vehicle
        dx(1,1)=-(2*Caf+2*Car)/(m*x_dot)*y_dot+(-x_dot-(2*Caf*lf-2*Car*lr)/(m*x_dot))*psi_dot+2*Caf/m*delta;
        dx(2,1)=psi_dot;
        dx(3,1)=-(2*lf*Caf-2*lr*Car)/(Iz*x_dot)*y_dot-(2*lf^2*Caf+2*lr^2*Car)/(Iz*x_dot)*psi_dot+2*lf*Caf/Iz*delta;
        dx(4,1)=sin(psi)*x_dot+cos(psi)*y_dot;

    % Simulate the new states
    T = (Ts)*(i-1):(Ts)/30:Ts*(i-1)+(Ts);
    [T,x]=ode45(@(t,x) dx,T,states);
    states=x(end,:);
    statesTotal(i+1,:)=states;
    if velReassing==1
        if states(4)-Y_ref(i,2)>0.5
            x_dot=x_dot-x_dot*5/100;
            X_ref(i+1,2)=X_ref(i,2)+(x_dot*Ts);
    
            scriptA=[-(2*Caf+2*Car)/(m*x_dot) 0 (-x_dot-(2*Caf*lf-2*Car*lr)/(m*x_dot)) 0; ...
                0 0 1 0; ...
                -(2*lf*Caf-2*lr*Car)/(Iz*x_dot) 0 -(2*lf^2*Caf+2*lr^2*Car)/(Iz*x_dot) 0; ...
                1 x_dot 0 0];
            scriptB=[2*Caf/m;0;2*lf*Caf/Iz;0];
            C= [0 1 0 0;0 0 0 1];
            D=[0; 0];
            
            Ad=eye(length(scriptA(1,:)))+Ts*scriptA;
            Bd=Ts*scriptB;
            Cd=C;
            Dd=D;
            [Hdb,Fdbt,Cdb,Adc]=costFuncScript(Ad,Bd,Cd,Dd,hz);
        else
            if x_dot+x_dot*15/100<velReassingSetSpeed
                x_dot=x_dot+x_dot*15/100;
            X_ref(i+1,2)=X_ref(i,2)+(x_dot*Ts);
            scriptA=[-(2*Caf+2*Car)/(m*x_dot) 0 (-x_dot-(2*Caf*lf-2*Car*lr)/(m*x_dot)) 0; ...
                0 0 1 0; ...
                -(2*lf*Caf-2*lr*Car)/(Iz*x_dot) 0 -(2*lf^2*Caf+2*lr^2*Car)/(Iz*x_dot) 0; ...
                1 x_dot 0 0];
            scriptB=[2*Caf/m;0;2*lf*Caf/Iz;0];
            C= [0 1 0 0;0 0 0 1];
            D=[0; 0];
            
            Ad=eye(length(scriptA(1,:)))+Ts*scriptA;
            Bd=Ts*scriptB;
            Cd=C;
            Dd=D;
            [Hdb,Fdbt,Cdb,Adc]=costFuncScript(Ad,Bd,Cd,Dd,hz);
            else
                x_dot=velReassingSetSpeed;
            end
        end
    end
    x_dotTotal(i)=x_dot;
end
    x_dotTotal(i+1)=x_dot;
%% Plot Displacement

figure;
plot(X_ref(:,2),Y_ref(:,2),'--b','LineWidth',2)
hold on
plot(X_ref(:,2),statesTotal(1:end,4),'r','LineWidth',1)
grid on;
xlabel('x actual [m]','FontSize',12)
ylabel('y actual [m]','FontSize',12)
legend({'Position Ref','Position Actual'},'Location','northeast','FontSize',12)

%% Plot Adaptive

figure;
subplot(4,1,1)
stairs(t(1:sim_length),deltaTotal,'r','LineWidth',2.5)
title('Steering Angle // Discrt. Time MPC // Cz=5, Hz=15','FontSize',16)
xlabel('Time [s]','FontSize',12)
ylabel('Steering Angle [rad]','FontSize',12)
legend({'Discrt. Steer. Angl.'},'Location','northeast','FontSize',12)

grid on
xlim([0 40])

subplot(4,1,2)
plot(t(1:sim_length),Y_ref(1:sim_length,2),'--b','LineWidth',2)
title('Lateral Displacement // Discrt. Time MPC // Cz=5, Hz=15','FontSize',16)
hold on

subplot(4,1,3)
plot(t(1:sim_length),psi_ref(1:sim_length,2),'--b','LineWidth',2)
title('Yaw Angle // Discrt. Time MPC // Cz=5, Hz=15','FontSize',16)
hold on

subplot(4,1,2)
plot(t(1:sim_length),statesTotal(1:sim_length,4),'r','LineWidth',1)
grid on
xlabel('time [s]','FontSize',12)
ylabel('Lat. Disp. [m]','FontSize',12)
legend({'Ref Lat. Disp.','Actual Lat. Disp.'},'Location','northwest','FontSize',12)

subplot(4,1,3)
plot(t(1:sim_length),statesTotal(1:sim_length,2),'r','LineWidth',1)
grid on
legend({'Ref Yaw','Actual Yaw'},'Location','northwest','FontSize',12)
xlabel('time [s]','FontSize',12)
ylabel('Yaw Angle [rad]','FontSize',12)

subplot(4,1,4)
plot(t(1:sim_length),x_dotTotal,'r','LineWidth',1)
grid on
title('Longudit. Vel. Change // Discrt. Time MPC // Cz=5, Hz=15','FontSize',16)
legend({'Vx [m/s]'},'Location','northwest','FontSize',12)
xlabel('time [s]','FontSize',12)
ylabel('Vx [m/s]','FontSize',12)