function send(client, order, data)
    write(client,order,'uint8');
    if (order(1)==1)
        write(client,data,'single');
    end
end