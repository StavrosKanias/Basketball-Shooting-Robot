function move(servos, q, dt)

    n = width(q);
    % Use this after having calculated the joint angles q to move the servos
    % dt is the Euler integration step you chose (di corresponds to the sampling step - you might have to change it)
    di = 0.22/dt;
    % Convert q from rads to degs
    theta = round(rad2deg(q));
    % Declare the angle configuration matrix
    points = height(q);
    angle = zeros(points,n);
    % Normalization to range [0,0.95]
    for i=1:n
            angle(:,i) = 0.95*((theta(:,i) + 140) / (270));
    end

    % Write the normalized angles to each servo
    for i = 1:di:points
        for s= 1:n
                %update the position of servos
                writePosition(servos(s), angle(i,s));
                %position = readPosition(s(j));
        end
    end
    
end