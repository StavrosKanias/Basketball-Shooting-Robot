function s = initialize_arduino()

    % construct the object arduino
    a = arduino('COM5');
    % create the servos
    s1 = servo(a, 'D3',  'MinPulseDuration', 500*10^-6, 'MaxPulseDuration', 2600*10^-6);
    s2 = servo(a, 'D4',  'MinPulseDuration', 500*10^-6, 'MaxPulseDuration', 2600*10^-6);
    s3 = servo(a, 'D5',  'MinPulseDuration', 500*10^-6, 'MaxPulseDuration', 2600*10^-6);
    s4 = servo(a, 'D6',  'MinPulseDuration', 500*10^-6, 'MaxPulseDuration', 2600*10^-6);
    s5 = servo(a, 'D7',  'MinPulseDuration', 500*10^-6, 'MaxPulseDuration', 2600*10^-6);
    s6 = servo(a, 'D8',  'MinPulseDuration', 500*10^-6, 'MaxPulseDuration', 2600*10^-6);
    s7 = servo(a, 'D9',  'MinPulseDuration', 500*10^-6, 'MaxPulseDuration', 2600*10^-6);
    s8 = servo(a, 'D10',  'MinPulseDuration', 100*10^-6, 'MaxPulseDuration', 4000*10^-6);
    
    s = [s1;s2;s3;s4;s5;s6;s7;s8];

end