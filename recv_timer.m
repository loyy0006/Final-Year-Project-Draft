function curr_time = recv_timer
    MSSG = judp('RECEIVE',36500,2000);
    out=typecast(MSSG,'double');
    curr_time=transpose(out);
end