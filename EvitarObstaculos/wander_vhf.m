%Crear figuras distintas para el láser y el visualizador del VFH
% NOTA: para dibujar sobre una figura concreta, antes de llamar a la
% correspondiente función de dibujo debe convertirse en la figura activa
% utilizando figure(fig_laser) o figure(fig_vfh) respectivamente.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all
fig_laser=figure; title('LASER')
fig_vfh=figure; title('VFH')
r=robotics.Rate(10);
%Crear el objeto VFH…
%%%%%%%%%%%%%%%%%%%%%%
VFH=controllerVFH;
%y ajustamos sus propiedades
%%%%%%%%%%%%%%%%%%%%%%%%%%
VFH.NumAngularSectors=180;
VFH.DistanceLimits=[0.1 3] ;
VFH.RobotRadius=0.1;
VFH.SafetyDistance=0.1;
VFH.MinTurningRadius=0.1;
VFH.TargetDirectionWeight=5;
VFH.CurrentDirectionWeight=2;
VFH.PreviousDirectionWeight=2;
VFH.HistogramThresholds=[3 10];
VFH.UseLidarScan=true; %para permitir utilizar la notación del scan
%Rellenamos los campos por defecto de la velocidad del robot, para que la lineal
%sea siempre 0.1 m/s
pub_vel=rospublisher('/robot0/cmd_vel','geometry_msgs/Twist');
msg_vel=rosmessage(pub_vel);

msg_vel.Linear.X=0.1;
msg_vel.Linear.Y=0;
msg_vel.Linear.Z=0;
msg_vel.Angular.X=0;
msg_vel.Angular.Y=0;
msg_vel.Angular.Z=0;

send(pub_vel,msg_vel);
%valor para ajustar velocidad angualr
K=0.5;
%Bucle de control infinito

while(1)

 %Leer y dibujar los datos del láser en la figura ‘fig_laser’
 figure(fig_laser);
 lee_sensores2;

 %Llamar al objeto VFH para obtener la dirección a seguir por el robot para
 %evitar los obstáculos. Mostrar los resultados del algoritmo (histogramas)
 %en la figura ‘fig_vfh’
 targetDir=0;
 scan = lidarScan(msg_laser);
 steeringDir = VFH(scan,targetDir);
 figure(fig_vfh);
 show(VFH);

 %Rellenar el campo de la velocidad angular del mensaje de velocidad con un
 %valor proporcional a la dirección anterior (K=0.1)
    V_ang = K*steeringDir;
    msg_vel.Angular.Z=V_ang;
 %Publicar el mensaje de velocidad
    send(pub_vel,msg_vel);

 %Esperar al siguiente periodo de muestreo
 waitfor(r);
end
