%% record data from model
load_system('dentistry_v2020'); %change this to model name
collection = zeros(50,6);
disp('Recording ...');
for i = 1:50
    rto = get_param('dentistry_v2020/send_to_server/Integration/Wrench conversion', 'RuntimeObject'); % change this to appropriate simulink block
    for j = 1:6
        collection(i,j) = rto.InputPort(j).Data;
    end
    pause(0.02);
end

clear rto

%% display statistics & extract only mean.
tare = mean(collection);
disp('tare = [');
for i = 1:6
    disp(num2str(tare(i), 15));
end
fprintf('];\n');

clear collection tare i