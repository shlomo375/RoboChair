function data = receive(client, order)
pack_num = 100;
flush(client);
write(client,order,'uint8');
while(client.NumBytesAvailable == 0) end
packege = read(client,1,"single");
data = zeros(1,packege);
i = 1;
while(client.NumBytesAvailable > 0)
    if(client.NumBytesAvailable/4 > pack_num)
        data(i:i+pack_num) = read(client,pack_num,"single");
        i = i + pack_num;
    else
        data(i:i+client.NumBytesAvailable/4) = read(client,client.NumBytesAvailable,"single");
        i = i + client.NumBytesAvailable/4;
    end
end


end