%Communication protocol:
%Sending an array of instructions in the following structure: [Additional information (float), command, number of packages of the following information]
%The decimal information is then sent. In case the information is int it is sent along with the instructions.
function data = communication(client,order, data)
switch order
    case "set_speed_r"
        order = uint8([1 101 1]);
        data = single(data);
        send(client, order, data);
    case "set_speed_l"
        order = uint8([1 102 1]);
        data = single(data);
        send(client, order, data);
    case "do_step_response"
        order = uint8([1 103 1]);
        data = single(data);
        send(client, order, data);
    case "set_pid_k_r"
        order = uint8([1 111 3]);
        data = single(data);
        send(client, order, data);
    case "set_pid_k_l"
        order = uint8([1 112 3]);
        data = single(data);
        send(client, order, data);
    case "drive_motor"
        data = uint8(data);
        order = uint8([0 100 data]);
        send(client, order, data);
    case "step_response_data"
        order = uint8([0 200]);
        data = receive(client, order);
        
    otherwise
        printf("order not recognaized");
        

end