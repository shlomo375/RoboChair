function data = step_velocity(client,motor,k,velocity)
if(motor == 1)
    order = single([111 k])
    write(client,order,'single');
    pause(0.1);
    order = single([101 velocity])
    write(client,order,'single');
end
if(motor == 2)
    order = single([112 k])
    write(client,order,'single');
    pause(0.1);
    order = single([102 velocity])
    write(client,order,'single');
end
while(client.NumBytesAvailable < 1000)
end
data = read(client,1000,'single');
end