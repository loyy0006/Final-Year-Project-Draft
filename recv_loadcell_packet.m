function loadcell_record_current = recv_loadcell_packet
    MSSG = judp('RECEIVE',36500,2000);
    out=typecast(MSSG,'double');
    loadcell_record_current=transpose(out);
end