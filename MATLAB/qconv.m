function [J,pos,quat]=qconv(robot,q)
    T = robot.fkine(q);
    pos = T.t;
    quat = T.UnitQuaternion;
    J = robot.jacob0(q);
end