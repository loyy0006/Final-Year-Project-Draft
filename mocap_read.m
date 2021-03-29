timeout = 0;
markers = zeros(9,6);
while timeout < 2000
    pause(0.02)
    %timeout = timeout + 1;
    % get current time from UDP (sent by quanser)
    msg = judp('RECEIVE', 36500, 1000);
    time = typecast(msg, 'double');
    % get current marker positions
    % format is n*6 matrix [TCMID, LEDID, X, Y, Z, Timestamp]
    curr_markers = VzGetDat;
    % replace timestamp with quanser time for each marker
    for i = 1:size(curr_markers,1)
        curr_markers(i,6) = time;
    end
    % concat curr_markers to 3-dimensional array markers
    markers = cat(3, markers, curr_markers);
end