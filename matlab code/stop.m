function [] = stop(client)
engine_1 = uint8([100 1 0 1]);
engine_2 = uint8([100 2 0 1]);
write(client,engine_1);
pause(0.001);
write(client,engine_2);
end