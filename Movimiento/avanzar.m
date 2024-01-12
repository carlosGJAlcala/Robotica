distancia=4.0;
r=robotics.Rate(10);
msg_vel.Linear.X=0.1;
msg_vel.Linear.Y=0.1;
msg_vel.Linear.Z=0;
msg_vel.Angular.X=0;
msg_vel.Angular.Y=0;
msg_vel.Angular.Z=0;

%Leemos la primera posicion
initpos = sub_odom.LatestMessage.Pose.Pose.Position;
disp("Inicializamos leyendo la primera posicion: ");

while(1)
    pos=sub_odom.LatestMessage.Pose.Pose.Position;
    dist=sqrt((initpos.X-pos.X)^2+(initpos.Y-pos.Y)^2);
    if(dist>distancia)
        msg_vel.Linear.X=0;
        send(pub_vel,msg_vel);
        break;
    else
       disp("avanza el bot ");

        send(pub_vel,msg_vel);
    end
    lee_sensores;
    waitfor(r);
end