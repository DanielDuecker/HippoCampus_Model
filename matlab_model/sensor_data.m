function sensor_output = sensor_data(t, x)
%% In this function sensor properties, like noise, or other disturbances 
%% can be added to the vehicle dynamics

sensor_output = x;