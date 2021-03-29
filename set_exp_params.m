function set_exp_params
clc; clear; close all;
    prompt = '(1) New Experiment? \n(2) Processing old experiment?\n';
    flag = input(prompt);
     validate_prompt = '(1) Validation? or \n(2) Collecting data of human demonstrations?\n';
    validate_flag = input(validate_prompt);
 set_data_settings_dentistry
    if(flag ==1)
    online_set_trial_settings
    else
    offline_set_trial_settings
    end
    
   
    set_SE3filter_params
    
    if(flag ==1)
        switch validate_flag
            case 1
                disp('Please run send_loadcell_to_server.m and then send_mocap_to_server.m and then server_real_time.m');
            case 2
                disp('Please run loadcell_collect.m and then mocap_collect.m');
        end
    else
        disp('Please run process_data.m');
    end
end

function online_set_trial_settings
exp_prompt = 'which experiment? buccal / lingual / occlusal. \n';
exp = input(exp_prompt,'s');
subject_prompt = 'stroke? linear / elliptical \n';
subject = input(subject_prompt,'s');

extension='.mat';
formatIn = 'mm-dd-yy_';
DateString=datestr(now,formatIn);
timestamp = datestr(now,'HH_MM_SS');
path = sprintf('./data/');
loadcell_filename = strcat(path,DateString,timestamp,'.',subject,'.',exp,'.loadcell',extension);
mocap_filename = strcat(path,DateString,timestamp,'.',subject,'.',exp,'.mocap',extension);
interp_mocap_filename = strcat(path,DateString,timestamp,'.',subject,'.',exp,'.mocap.interp',extension);
interp_loadcell_filename = strcat(path,DateString,timestamp,'.',subject,'.',exp,'.loadcell.interp',extension);
filtered_mocap_filename = strcat(path,DateString,timestamp,'.',subject,'.',exp,'.mocap.SE3_filtered',extension);
bimanual_manipulation_analysis_filename = strcat(path,DateString,timestamp,'.',subject,'.',exp,'.analysis',extension);

save('trial_settings.mat','loadcell_filename','mocap_filename','interp_mocap_filename','interp_loadcell_filename','filtered_mocap_filename','bimanual_manipulation_analysis_filename');

end

function offline_set_trial_settings
exp_prompt = 'which experiment? buccal / lingual / occlusal. \n';
exp = input(exp_prompt,'s');

strcat('*.',exp,'.mocap.mat');

path = sprintf('./data/');
file_list = dir([path strcat('*.',exp,'.mocap.mat')]);
disp('Files available:');
    for file_index = 1:length(file_list)
    FileRecorded = file_list(file_index).name;
    strcat(sprintf('(%d) -- ',file_index),FileRecorded)
    end
    file_prompt = 'Choose which file to process.\n';
    file_index = input(file_prompt);
    FileRecorded = file_list(file_index).name;
    load([path FileRecorded]);
    
    
extension='.mat';

matFile = strsplit(char(FileRecorded),'.mocap.mat'); matFile = char(matFile(1));

loadcell_filename = strcat(path,matFile,'.loadcell',extension);
mocap_filename = strcat(path,matFile,'.mocap',extension);
interp_mocap_filename = strcat(path,matFile,'.mocap.interp',extension);
interp_loadcell_filename = strcat(path,matFile,'.loadcell.interp',extension);
filtered_mocap_filename = strcat(path,matFile,'.mocap.SE3_filtered',extension);
bimanual_manipulation_analysis_filename = strcat(path,matFile,'.analysis',extension);

save('trial_settings.mat','loadcell_filename','mocap_filename','interp_mocap_filename','interp_loadcell_filename','filtered_mocap_filename','bimanual_manipulation_analysis_filename');

end


function set_data_settings_dentistry
    
    time_prompt = 'Enter experiment duration in seconds.\n';
    exp_dur = input(time_prompt);
    sampling_frequency_prompt = 'Enter sampling frequency in Hz.\n';
    sampling_frequency = input(sampling_frequency_prompt);

    time_loadcell = zeros(exp_dur*sampling_frequency,2);
    loadcell_record = zeros(exp_dur*sampling_frequency,7); % 1 t, 6 variables

    no_markers = 9;
    mocap_record = zeros(exp_dur*sampling_frequency,no_markers*6);
    time_mocap = zeros(exp_dur*sampling_frequency,1);
    
    loadcell_traffic_port = 36505;
    mocap_traffic_port = 36500; 
    server_ip = '192.168.0.101';
    mocap_ip = '192.168.0.100';
    quanser_ip = '192.168.0.103'; % as of 10/3/2021
    

    save('data_settings.mat');

end



function set_SE3filter_params

%% Physical params
    freq=150;
    t_c=1/(2*pi*freq);        
    k=100;
    
    N=100;
    t_R_c=t_c/3;
    
    dT=1/freq; % depends on the application sampling time
    
    %% Ground truth vector params
    
mm=1/1000;
cm=1/100;

platform_com_n = [0*mm 0*mm 0*mm]';

platform_m_i_n={};
platform_m_i_n{1}= [0 6.55*mm 0]'; %LED17
platform_m_i_n{2}= [-6.15*mm 0 0]'; %LED18
platform_m_i_n{3}= [6.15*mm 0 0]'; %LED19

toothbrush_handle_com_n = [0*mm 0 0*mm]';
toothbrush_handle_m_i_n={};
toothbrush_handle_m_i_n{1}= [-11.7*mm 0*mm 11.7*mm]'; %LED23
toothbrush_handle_m_i_n{2}= [4.3*mm 0*mm 16*mm]'; %LED24 
toothbrush_handle_m_i_n{3}= [15.9*mm 0*mm 4.3*mm]'; %LED25 
toothbrush_handle_m_i_n{4}= [11.7*mm 0*mm -11.7*mm]'; %LED26 
toothbrush_handle_m_i_n{5}= [-4.3*mm 0*mm -16*mm]'; %LED27 
toothbrush_handle_m_i_n{6}= [-15.9*mm 0*mm -4.3*mm]'; %LED28 

% Characteristic Length calculation
platform_N_marker_task = 3;     % number of makers
toothbrush_handle_N_marker_task = 6;     % number of makers
platform_L = 0;
toothbrush_handle_L = 0;
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
    
save('SE3filter_params.mat','dT','N','t_R_c','t_c','k','freq','platform_N_marker_task','toothbrush_handle_N_marker_task','platform_frame_idx','toothbrush_handle_frame_idx','platform_m_i_n','toothbrush_handle_m_i_n','toothbrush_handle_com_n','platform_L','toothbrush_handle_L');

end

