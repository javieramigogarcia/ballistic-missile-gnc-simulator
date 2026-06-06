function [value, isterminal, direction] = f_event_burnout(t, Y, tb)
    value = t - tb;      
    isterminal = 1;      
    direction = 0;       
end