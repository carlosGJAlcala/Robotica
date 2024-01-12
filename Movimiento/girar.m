distancia=4.0;
th= -90.0*(3.14/180);
velocidadgiro=0.0;
msg_vel.Linear.X=0.0;
msg_vel.Linear.Y=0.0;
msg_vel.Linear.Z=0;
msg_vel.Angular.X=0.0;
msg_vel.Angular.Y=0;
if (th>=0)
    msg_vel.Angular.Z=0.1;
else
    msg_vel.Angular.Z=-0.1;
end
yaw=0.0;

%Leemos la primera posicion
initpos = sub_odom.LatestMessage.Pose.Pose.Position;
%angulo
% initori=sub_odom.LatestMessage.Pose.Pose.Orientation;
% ang_euler=quat2eul([initori.W initori.X initori.Z]);
% yawini=ang_euler(1);
initori = sub_odom.LatestMessage.Pose.Pose.Orientation;
ang_euler=quat2eul([initori.W initori.X initori.Y initori.Z]);
yawini=ang_euler(1);


disp("Inicializamos leyendo la primera posicion: ");
disp(yawini);
while(1)
    ori = sub_odom.LatestMessage.Pose.Pose.Orientation;
    ang_euler1=quat2eul([ori.W ori.X ori.Y ori.Z]);
    yaw=ang_euler1(1);
    ang=angdiff(yawini,yaw);
    disp(ang);
    if(abs(ang)>abs(th))
        msg_vel.Angular.Z=0.0;
        send(pub_vel,msg_vel);
        break;  
    else
        send(pub_vel,msg_vel);
    end
    lee_sensores;
    waitfor(r);
end