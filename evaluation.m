%% connect to the robot
%-----Create an API instance to Connect to the Robot
gen3Kinova = makeConnection();

%%
admittanceMode = int32(2);
isOk1 = gen3Kinova.SetAdmittance(admittanceMode);
if isOk1
    disp('Command sent to the robot. Wait for the admittance mode to be active.');
else
    disp('Command error.');
end
disp('Admittance mode actiavted');

%% sample points to form the frame
[isOk,baseFb, actuatorFb, interconnectFb] = gen3Kinova.SendRefreshFeedback();
q_init = actuatorFb.position;
i = 1;
kb = HebiKeyboard();
keys = read(kb);

sampe_vec = [];
while i<=3
    
    strg = strcat('sampling',' ', num2str(i),'th',' point');
    disp(strg)
    while keys.SPACE ~= 1
        [isOk,baseFb, actuatorFb, interconnectFb] = gen3Kinova.SendRefreshFeedback();
        keys = read(kb);
    end
    pos = baseFb.tool_pose(1:3)'+[0,0,0.12]';
    sampe_vec = [sampe_vec,pos];
    
    kb = HebiKeyboard();
    keys = read(kb);
    i = i+1;
end

%% object Frame creation
sampe_vec=sampe_vec-[0;0;0.12];
p1 = sampe_vec(:,1); p2 = sampe_vec(:,2);
y_RO = (p2-p1)/norm(p2-p1);
z_RO = [0,0,1]';
x_RO = cross(y_RO,z_RO);

R_RO = [x_RO,y_RO,z_RO];
P_RO = sampe_vec(:,3);
T_RO = ones(4,4); T_RO(1:3,1:3) = R_RO; T_RO(1:3,4) = P_RO;   %%

data = readmatrix('3D_coords.csv')';
data_corr = [data(:,3) data(:,2) data(:,4) data(:,1)];

q1 = data_corr(:,1);
T_BO = eye(4);T_BO(1:3,4) = q1;
% convert all coords to object coord system
q_ob = [];
for i = 1:size(data,2)
    d = [data_corr(:,i);1];
    d_O = inv(T_BO)*d;
    q_ob = [q_ob,d_O(1:3)];
    
end
%% convert object frame coordinates to Robot frame coordinates
%T_RO* coordinates in object frame
q_R = []
for i = 1:size(data,2)
    d = [q_ob(:,i);1];
    d_R = T_RO*d;
    q_R = [q_R,d_R(1:3)];%+[0;0;0.07]];
    
end
plot(q_R(1,:))
%% 
timestamp = [0,1,2,3];
timestamp1 = 0:0.1:2
q_R_int = interp1(timestamp,q_R',timestamp1)

p_arr_up = q_R_int';
J_arr_up = PMPupdated_v1_test_ptp(p_arr_up, myKin,q_init);

 %getTransform(myKin,J_arr_up(:,3)','Tool_link')
% simulation---upmotiono------------------------------
q_curr = deg2rad(q_init);
%finalSim_dent(myKin,q_curr,J_arr_up)
J = J_arr_up;

totalTime = 6; % secs
dt = 2/size(J,2);
[pos_up, vel_up, acc_up,timestamp_up] = dentTraj (J, dt);


% simulation down motion
J_arr_down = fliplr(J_arr_up);
%finalSim_dent(myKin,q_curr,J_arr_down)

q_curr = deg2rad(J_arr_down(:,1));
J = J_arr_down;
[pos_down, vel_down, acc_down,timestamp_down] = dentTraj (J, dt);

%% %% Go to the initial position of the trajectory

%startExecution(gen3Kinova, pos_int_pick, vel_int_pick, acc_int_pick, timestamp_pick)
kinPos = wrapTo360(pos_up);

q_init = kinPos(1,:);  %make sure the robot's starting position matches the first position of the trajectory
jointCmd = wrapTo360(q_init);
constraintType = int32(0);
speed = 0;
duration = 0;

%----------Send joint command to the robot.---------------------------
isOk = gen3Kinova.SendJointAngles(jointCmd, constraintType, speed, duration);

if isOk
    disp('Command sent to the robot. Wait for the robot to stop moving.');
else
    disp('Command error.');
end
%Command sent to the robot. Wait for the robot to stop moving.

while 1
    [isOk,~, actuatorFb, ~] = gen3Kinova.SendRefreshFeedback();
    %show(gen3, ((actuatorFb.position)*pi/180)','PreservePlot',false);
    drawnow;
    if isOk
        if max(abs(wrapTo360(q_init)-actuatorFb.position)) < 0.1
            disp('End Point reached.')
            break;
        end
    else
        error('SendRefreshFeedback error')
    end
end
pause(0.5)

%% verify the cartesian trajectory
for i = 1:size(J_arr_up,2)
    q_curr = J_arr_up(:,i)';
    T = getTransform(myKin,q_curr,'Tool_link');
    p_comp = T(1:3,4);
    p_act = q_R_int(i,:);
    plot(i,p_comp(1),'r*');hold on; 
    plot(i,p_act(1),'g*');hold on; 

 end
%% startMotion
trials_no = 8;
for i =1:trials_no
 posVec_up = []; posVec_down = [];timeVec_up = [];timeVec_down = [];
[posVec_up,timeVec_up] = startExecution_test_v1_dent(gen3Kinova, pos_up, vel_up, acc_up, timestamp_up,posVec_up,timeVec_up);
posVec_up_struct{i} = posVec_up;

timeVec_up_struct{i} = timeVec_up;

pause(0.5)
posVec_down = startExecution_test_v1_dent(gen3Kinova, pos_down, vel_down, acc_down, timestamp_down,posVec_down,timeVec_down);
posVec_down_struct{i} = posVec_down;
timeVec_down_struct{i} = timeVec_down;

pause(0.5)
end

%% plotting
for i = 1:trials_no
    p = posVec_up_struct{i}
    
    subplot(3,1,1)
    plot1a = plot(timeVec_up_struct{i},p(1,:),'r','LineWidth',2); hold on
    legend('actual position');hold on

    subplot(3,1,2)
    plot2a = plot(timeVec_up_struct{i},p(2,:),'g','LineWidth',2); hold on
    legend('actual position');hold on
    
    subplot(3,1,3)
    plot3a = plot(timeVec_up_struct{i},p(3,:),'b','LineWidth',2); hold on
    legend('actual position');hold on;
end

subplot(3,1,1)
plot1c = plot(timestamp1,q_R_int(:,1),'k','LineWidth',2); hold on;
xlabel('time(s)'); ylabel('robot X position (m)');
legend([plot1a plot1c],{'actual position','commanded position'}) 

subplot(3,1,2)
plot2c = plot(timestamp1,q_R_int(:,2),'k','LineWidth',2); hold on;
xlabel('time(s)');ylabel('robot Y position (m)');
legend([plot2a plot2c],{'actual position','commanded position'})

subplot(3,1,3)
plot3c = plot(timestamp1,q_R_int(:,3),'k','LineWidth',2); hold on;
xlabel('time(s)');ylabel('robot Z position (m)');
legend([plot3a plot3c],{'actual position','commanded position'})

%%
pos_resultant = vecnorm(posVec_up);
q_R_int = timestamp1;

plot(timeVec_up,posVec_up(1,:)); hold on
plot(timestamp1,q_R_int(:,1)); hold on

plot(timeVec_up,posVec_up(2,:)); hold on
plot(timestamp1,q_R_int(:,2)); hold on

plot(timeVec_up,posVec_up(3,:)); hold on
plot(timestamp1,q_R_int(:,3)); hold on

