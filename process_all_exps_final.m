%segment the first 5s for platform 

function process_all_exps_final 
clc;clear;close all;
    path = sprintf('./data/');
    file_list = dir([path strcat('*.Occlusal.Linear.mocap.mat')]); 
    
    for file_index = 1:length(file_list)
        disp('Now processing');
        FileRecorded = file_list(file_index).name;
        strcat(sprintf('(%d) -- ',file_index),FileRecorded)

        offline_set_trial_settings(file_index)
        load_data_settings(file_index)
        disp('Offline file information loaded');
        interp_data
        disp('Interpolation done');
        set_SE3filter_params
        disp('SE3 filter set');
        disp('Processing has begun');
        process_mocap_data_v_clean_mocap
        disp('Processing done');
        fuse_data
        disp('Data fused');
        blender
        disp('Ready for Blender');
        
    end
    
end

function offline_set_trial_settings(file_index)

    path = sprintf('./data/');
    file_list = dir([path strcat('*.Occlusal.Linear.mocap.mat')]); 
    FileRecorded = file_list(file_index).name;
    load([path FileRecorded]);

extension='.mat';
figextension = '.fig';

matFile = strsplit(char(FileRecorded),'.Occlusal.Linear.mocap.mat'); matFile = char(matFile(1));

loadcell_filename = strcat(path,matFile,'.linear.occlusal.loadcell',extension);
mocap_filename = strcat(path,matFile,'.Occlusal.Linear.mocap',extension); 
interp_mocap_filename = strcat(path,matFile,'.Occlusal.Linear.mocap.interp',extension);
interp_loadcell_filename = strcat(path,matFile,'.linear.occlusal.loadcell.interp',extension);
filtered_mocap_filename = strcat(path,matFile,'.Occlusal.Linear.mocap.SE3_filtered',extension);
full_experiment_filename = strcat(path,matFile,'.full_experiment',extension);
save('trial_settings.mat','loadcell_filename','mocap_filename','interp_mocap_filename','interp_loadcell_filename','filtered_mocap_filename','full_experiment_filename');

end

function load_data_settings(file_index)
    
    path = sprintf('./data/');
    mocap_file_list = dir([path strcat('*.Occlusal.Linear.mocap.mat')]); 
    loadcell_file_list = dir([path strcat('*.linear.occlusal.loadcell.mat')]);
    no_markers = 9;
    exp_dur = 30;
    sampling_frequency = 150;
    mocap_FileRecorded = mocap_file_list(file_index).name;
    load([path mocap_FileRecorded]);
    save('data_settings.mat','mocap_record','no_markers','sampling_frequency','exp_dur');
    loadcell_FileRecorded = loadcell_file_list(file_index).name;
    load([path loadcell_FileRecorded]);
    
    save('data_settings.mat','time_mocap','mocap_record','no_markers','time_loadcell','loadcell_record','sampling_frequency','exp_dur');

end

function set_SE3filter_params

%% Physical params (tune the behaviour of the response)
    freq=150;
    t_c=1/(2*pi*freq);        
    k=100; %PID controller error x value; k too high --> reactive, k too low --> not reactive enough, depends on freq  
    
    N=100; %N larger will converge to actual value but not feasible for real-time data (using too much RAM required) 
    t_R_c=t_c/3; %tc 3x higher than trc
    
    dT=1/freq; %dependent on sampling freq
    
    %% Ground truth vector params
    
mm=1/1000;
cm=1/100;

platform_com_n = [0*mm 0*mm 0*mm]';

platform_m_i_n={};
platform_m_i_n{1}= [0 65.5*mm 0]'; %LED17
platform_m_i_n{2}= [-61.5*mm 0 0]'; %LED18
platform_m_i_n{3}= [61.5*mm 0 0]'; %LED19

toothbrush_handle_com_n = [0*mm 0 0*mm]';
toothbrush_handle_m_i_n={};
toothbrush_handle_m_i_n{1}= [-11.7*mm 0*mm 11.7*mm]'; %LED23
toothbrush_handle_m_i_n{2}= [4.3*mm 0*mm 16*mm]'; %LED24 
toothbrush_handle_m_i_n{3}= [15.9*mm 0*mm 4.3*mm]'; %LED25 
toothbrush_handle_m_i_n{4}= [11.7*mm 0*mm -11.7*mm]'; %LED26 
toothbrush_handle_m_i_n{5}= [-4.3*mm 0*mm -16*mm]'; %LED27 
toothbrush_handle_m_i_n{6}= [-15.9*mm 0*mm -4.3*mm]'; %LED28 

% Characteristic Length calculation
platform_N_marker_task = 3;     % number of markers
toothbrush_handle_N_marker_task = 6;     % number of markers
platform_L = 0.05; %calculate this
toothbrush_handle_L = 0.05; %calculate this
for platform_i =1:platform_N_marker_task
platform_L = platform_L + norm(platform_com_n-platform_m_i_n{platform_i})/platform_N_marker_task; %characteristic length of object
end
for toothbrush_handle_i =1:toothbrush_handle_N_marker_task
toothbrush_handle_L = toothbrush_handle_L + norm(toothbrush_handle_com_n-toothbrush_handle_m_i_n{toothbrush_handle_i})/toothbrush_handle_N_marker_task; %characteristic length of object
end


    v17_idx = [3:5]; v18_idx = [9:11]; v19_idx = [15:17]; v23_idx = [21:23];
    v24_idx = [27:29]; v25_idx = [33:35]; v26_idx = [39:41]; v27_idx = [45:47];
    v28_idx = [51:53]; 
    
    platform_frame_idx = [v17_idx;
        v18_idx;
        v19_idx];

    toothbrush_handle_frame_idx = [v23_idx;
        v24_idx;
        v25_idx;
        v26_idx;
        v27_idx;
        v28_idx];


%% Save filter params
disp('SE3filtparams set again');
save('SE3filter_params.mat','dT','N','t_R_c','t_c','k','freq','platform_N_marker_task','toothbrush_handle_N_marker_task','platform_frame_idx','toothbrush_handle_frame_idx','platform_m_i_n','toothbrush_handle_m_i_n','platform_com_n','toothbrush_handle_com_n','platform_L','toothbrush_handle_L');

end             

function process_data
    interp_data
    process_mocap_data_v_clean_mocap
    fuse_data
    beep
end

function interp_data
        load('trial_settings.mat');
        load(mocap_filename);
        load(loadcell_filename);
        load('data_settings.mat','sampling_frequency');
        %% Interpolating data to sampling frequency requirements
        time_loadcell = time_loadcell(:,2);
        range_loadcell = time_loadcell > 0;
        time_loadcell_ext = time_loadcell(range_loadcell);
        starting_time = max(time_loadcell_ext(1,1),time_mocap(1,1)); %to find the common starting t
        ending_time = min(time_loadcell_ext(end,1),time_mocap(end,1)); %to find the common ending t
        sampling_time = 1/sampling_frequency; 
        interp_time = starting_time:sampling_time:ending_time;

        mocap_record_interp = interp1(time_mocap,mocap_record,interp_time);
        loadcell_record_interp = interp1(time_loadcell_ext,loadcell_record(range_loadcell,:),interp_time); %now all 3 variables are of the same size
        time_mocap_interp = interp_time';
        time_loadcell_interp = interp_time';
        save(interp_mocap_filename,'time_mocap_interp','mocap_record_interp');
        save(interp_loadcell_filename,'time_loadcell_interp','loadcell_record_interp');
end

function process_mocap_data_v_clean_mocap

    mm=1/1000;
    cm=1/100;

    %% Filter parameters

    load('SE3filter_params.mat')
    load('trial_settings.mat');

    %% Importing data
        load(mocap_filename);
        load('data_settings.mat','sampling_frequency');
        load('SO3.mat');
        load('SE3.mat');

        %% SE3 Filtering initiation
        platform_G0 = eye(4);
        toothbrush_handle_G0 = eye(4);
        previous = eye(4);

        T_W_H = zeros(4*length(mocap_record),4); %Transformation matrix for platform to mocap
        T_T_H = zeros(4*length(mocap_record),4); %Transofrmation matrix for toothbrush to mocap
        flag_signal = zeros(length(mocap_record),3); % W/M/F for flag_signal(i,1/2/3) respectively
        platform_m_i_S={};
        toothbrush_handle_m_i_S={};

        tic;
        disp('SE3 filtering has begun.');

        %% SE3 filtering
        initial_5s = mocap_record(1:800,:);
        figure, plot(initial_5s);
        avg_initial_5s = mean(initial_5s,1);
        
        % Platform (Find T_W_H)
        % Constant transformation matrix calculated
        % Finding x,y,z unit vectors (Normalised)
        X_hat = (avg_initial_5s(1,platform_frame_idx(3,:))'-avg_initial_5s(1,platform_frame_idx(2,:))')...
            /norm((avg_initial_5s(1,platform_frame_idx(3,:))'-avg_initial_5s(1,platform_frame_idx(2,:))'));
        A_hat = (avg_initial_5s(1,platform_frame_idx(1,:))'-avg_initial_5s(1,platform_frame_idx(2,:))')...
            /norm((avg_initial_5s(1,platform_frame_idx(1,:))'-avg_initial_5s(1,platform_frame_idx(2,:))'));
        Z_hat = cross(X_hat,A_hat)/norm(cross(X_hat,A_hat));
        Y_hat = cross(Z_hat,X_hat)/norm(cross(Z_hat,X_hat));

        platform_G_final = [X_hat Y_hat Z_hat cm*avg_initial_5s(1,platform_frame_idx(1,:))';
                 0 0 0 1]; %read this variable as the constant transformation matrix as calculated 
        T_W_H = platform_G_final;
        disp(T_W_H);
        
        % 6266 error
        for sample = 2:length(mocap_record)
            disp(sample);
            count = 0;
            platform_p0_n = SE3.Pos(platform_G0);
            platform_R0_n = SE3.Rot(platform_G0);
            toothbrush_handle_p0_n = SE3.Pos(toothbrush_handle_G0);
            toothbrush_handle_R0_n = SE3.Rot(toothbrush_handle_G0);

            platform_m_i_n_visible={};
            toothbrush_handle_m_i_n_visible={};
            jumping_threshold = 5*cm; 

            j = 1;
            for platform_i =1:platform_N_marker_task
                curr_marker = mocap_record(sample,platform_frame_idx(platform_i,:))';
                prev_marker = mocap_record(sample-1,platform_frame_idx(platform_i,:))';
                if(all(curr_marker==[0 0 0]') || (logical(norm(curr_marker'*cm - prev_marker'*cm)>jumping_threshold))) % out of view
                    flag_signal(sample,1)=1;
                else
                    platform_m_i_S{j} = curr_marker*cm;
                    platform_m_i_n_visible{j} = platform_m_i_n{platform_i};
                    j = j + 1;
                end
            end
            
            j = 1;
            for toothbrush_handle_i =1:toothbrush_handle_N_marker_task
                curr_marker = mocap_record(sample,toothbrush_handle_frame_idx(toothbrush_handle_i,:))';
                prev_marker = mocap_record(sample-1,toothbrush_handle_frame_idx(toothbrush_handle_i,:))';
                if(all(curr_marker==[0 0 0]') || norm(curr_marker'*cm - prev_marker'*cm)>jumping_threshold) % out of view
                    count = count + 1;
                else
                    toothbrush_handle_m_i_S{j} = curr_marker*cm;
                    toothbrush_handle_m_i_n_visible{j} = toothbrush_handle_m_i_n{toothbrush_handle_i}; %toothbrush_handle_i is the specific index for the specific LED
                    j = j + 1; %no. of markers visible and functional ie. nice jumping threshold or no zeros (well-behaved)                   
                end
            end

            
            % If more than three LEDs were not visible, set SE(3) matrix to the old one
            if(count >3)
                toothbrush_handle_G_final = previous;
            else          
                toothbrush_handle_G_final=SE3filter_10Mar2021(k,N,t_R_c,t_c,toothbrush_handle_R0_n,...
                    toothbrush_handle_p0_n,toothbrush_handle_m_i_S,toothbrush_handle_m_i_n_visible,...
                    toothbrush_handle_com_n,dT,toothbrush_handle_L);
            end
            previous = toothbrush_handle_G_final;
            
            T_T_H(4*sample-3:4*sample,:) = toothbrush_handle_G_final; %(4*sample-3:4*sample,:)= each of the 4x4 matrix
            T_W_H(4*sample-3:4*sample,:) = platform_G_final; 
            T_T_W(4*sample-3:4*sample,:) = SE3.inv(T_W_H(4*sample-3:4*sample,:))*T_T_H(4*sample-3:4*sample,:); %Transformation matrix from toothbrush to platform

            p_WT(sample,:) = SE3.Pos(T_T_W(4*sample-3:4*sample,:))';        
                
    end
        toc
        disp('Done.');

    filtered_filename = strsplit(char(mocap_filename),'.mat'); filtered_filename = filtered_filename(1);
    filtered_filename = strcat(filtered_filename,'.SE3_filtered.mat');
    save(char(filtered_filename),'T_W_H','T_T_H','T_T_W','p_WT','flag_signal');
    
    %% Plot Scatter plot for P_T_W (2D and 3D) and animated plot
    figure (2)
    scatter(p_WT(:,1),p_WT(:,2))
    axis equal
    ylabel('Y')
    xlabel('X')
    title('2D Scatterplot Occlusal Linear')

    figure (3)
    scatter3(p_WT(:,1),p_WT(:,2),p_WT(:,3))
    axis equal
    ylabel('Y')
    xlabel('X')
    zlabel('Z')
    title('3D Scatterplot Occlusal Linear')
    
    N_s  = length(p_WT);
    %animated figure plots
    %orientation 
    %figure(4)
    for idx = 1:100:N_s
        figure(4)
        %subplot(1,2,1);
        scatter3(p_WT(idx,1),p_WT(idx,2),p_WT(idx,3));hold on;
        axis equal;xlim([-0.2 0.2]);ylim([-0.2 0.2]);zlim([-0.2 0.2]);xlabel('x');ylabel('y');zlabel('z');
        title('3D Animated Trajectory');hold on
        figure(5)
        %subplot(1,2,2);
        plot(p_WT(1:idx,1),'-k'); hold on;plot(p_WT(1:idx,2),'-m'); hold on;plot(p_WT(1:idx,3),'-b');
        legend('x','y','z');
        title('Position-Sampling Frequency Graph');
        hold on;%drawnow
    end    
    
end

function fuse_data

    load('SO3.mat');
    load('SE3.mat');
    mm=1/1000;
    cm=1/100;

    %% Filter parameters

    load('SE3filter_params.mat');
    load('trial_settings.mat');
    load('data_settings.mat','sampling_frequency');
    
    %% Importing data
    load(interp_loadcell_filename);
    load(interp_mocap_filename);
    load(filtered_mocap_filename);

    loadcell_record = loadcell_record_interp;
    interp_time = time_loadcell_interp;

     %% Defining empty arrays for the fused dataset
     T_T_W = zeros(4*length(interp_time),4); % 4*length(interp_time),4 --> T_T_W = T_T_H*SE3.inv(T_W_H);
     p_WT = zeros(length(interp_time),3); 
     
     %% Fusing data - will yield
       for sample = 1:length(interp_time)
           T_T_W(4*sample-3:4*sample,:) = SE3.inv(T_W_H(4*sample-3:4*sample,:))*T_T_H(4*sample-3:4*sample,:);
           p_WT(sample,:) = SE3.Pos(T_T_W(4*sample-3:4*sample,:))';
       end
       
     %% Plotting of time graph
     %Set interp_time to start from 0
     interp_time_x = zeros(length(interp_time),1);
     for sample = 1:length(interp_time)
         normalised_value = interp_time(sample,1)-interp_time(1,1); %offset using the first value of interp_time
         interp_time_x(sample,1) = normalised_value;
     end
     
     % Position-time graph
     figure(5)
     N_s  = length(p_WT);     
     plot(interp_time_x(:,1),p_WT(:,1),'-.k','LineWidth',1);hold on; 
     plot(interp_time_x(:,1),p_WT(:,2),'-.m','LineWidth',1); hold on;
     plot(interp_time_x(:,1),p_WT(:,3),'-.b','LineWidth',1);
     legend('x','y','z');
     title('Position-time Graph');

     figure(6)
     %F-t graph
     subplot(3,1,1)
     plot(interp_time_x,loadcell_record(:,2));hold on; 
     plot(interp_time_x,loadcell_record(:,3));hold on; 
     plot(interp_time_x,loadcell_record(:,4));hold on;
     legend('Fx','Fy','Fz');
     title('Force-time Graph');
 
     
     %tau-t graph
     subplot(3,1,2)
     plot(interp_time_x,loadcell_record(:,5));hold on;  
     plot(interp_time_x,loadcell_record(:,6));hold on;
     plot(interp_time_x,loadcell_record(:,7));hold on;  
     legend('tx','ty','tz');
     title('Torque-time Graph');
     
     %position-t graph 
     subplot(3,1,3)
     plot(interp_time_x(:,1),p_WT(:,1));hold on; 
     plot(interp_time_x(:,1),p_WT(:,2)); hold on;
     plot(interp_time_x(:,1),p_WT(:,3));
     legend('x','y','z');
     title('Position-time Graph');
     
     save(full_experiment_filename,'p_WT','loadcell_record','interp_time','interp_time_x','T_T_H','T_W_H','T_T_W');     
     
end

function blender
    % Store csv into this format in Blender v2.90
    % [x y z eul_x eul_y eul_z time]
    
    load('trial_settings.mat');
    load(full_experiment_filename);
    
    fps = 24; %default 
    last_itr = 1000; %based on fig4 
    interval = 30; %experiment duration    
    
    for i = 1:(length(interp_time_x))
        index_for_ttw = 1+(i-1)*4;
        %T_T_W_B{i,1} = T_T_W(index_for_ttw:(index_for_ttw+3),1:4)*1000;
        T_T_W_B{i,1} = T_T_W(index_for_ttw:(index_for_ttw+3),1:4);
        T_T_W_B{i,2} = interp_time_x(i,1);
    end

    
%     for i = 1:fps*interval %i = each frame, fps*interval = total frames
%         %frame_time: timestamp of current frame
%         frame_time = (i-1)/fps+(interval/length(interp_time_x)* last_itr);
%         %find a data which matches the required timestamp
%         %the last iteration variable is to optimize the FOR loop
%         for j = last_itr:length(T_T_W_B)
%             if T_T_W_B{j,2} > frame_time 
%                 %record the pos in this frame
%                 blender_input(i,:) = [T_T_W_B{j,1}(1:3,4)'  tform2eul(T_T_W_B{j,1}, 'XYZ') i];
% %               blender_input(i,:) = [T_T_W_B{j,1}(1:3,4)'  0 1.5708 0 i];
%                 break;
%             end
%         end
%     end


tmp = T_T_W_B{1,1}(1:3,4)';
blender_input(1,:) = [T_T_W_B{1,1}(1:3,4)'  tform2eul(T_T_W_B{1,1}, 'XYZ') 1]; %set the first row first        
pos = 2;
speed = 3 %increase speed = slower, max speed = 1, cannot be non-integer. where speed=number of repeated rows taken

% if values of row same, skip. Else if values diff,   
for i = 2 : length(T_T_W_B)
    if tmp ~= T_T_W_B{i,1}(1:3,4)' 
        for j = 1:speed
            blender_input(pos,:) = [T_T_W_B{i,1}(1:3,4)' tform2eul(T_T_W_B{i,1}, 'XYZ') pos];
            tmp = T_T_W_B{i,1}(1:3,4)';
            pos = pos+1;
        end
    end
end

%       %compare TTWB
%       figure(7)
%       for i = 1:10:length(T_T_W_B)
%            scatter3(T_T_W_B{i,1}(1,4),T_T_W_B{i,1}(2,4),T_T_W_B{i,1}(3,4));hold on;
%            axis equal;xlim([-0.2 0.2]);ylim([-0.2 0.2]);zlim([-0.2 0.2]);xlabel('x');ylabel('y');zlabel('z');
%            hold on;drawnow
%       end
% 
%       %compare blender_input
%      figure(8)
%      for i = 1 :length(blender_input)
%           scatter3(blender_input(i,1),blender_input(i,2),blender_input(i,3));hold on;
%           axis equal;xlim([-0.2 0.2]);ylim([-0.2 0.2]);zlim([-0.2 0.2]);xlabel('x');ylabel('y');zlabel('z');
%           hold on;drawnow
%      end
    
    
   %Write to csv file to blender
    writematrix(blender_input,'motion_planning.csv') 

end

        
        
        
        
        
