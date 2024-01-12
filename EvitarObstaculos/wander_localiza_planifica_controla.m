%Definir la posicion de destino
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
endLocation = [12 9]; 

%Cargar el mapa
%%%%%%%%%%%%%%%
load map_modified.mat map_modified
map= map_modified;
%Crear el objeto VFH…y ajustar sus propiedades
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fig_laser=figure; title('LASER')
fig_vfh=figure; title('VFH')
r=robotics.Rate(10);
%Crear el objeto VFH…
%%%%%%%%%%%%%%%%%%%%%%
VFH=controllerVFH;
%y ajustamos sus propiedades%%%%%%%%%%%%%%%%%%%%%%%%%%
VFH.NumAngularSectors=180;
VFH.DistanceLimits=[0.5 2] ;
VFH.RobotRadius=0.1;
VFH.SafetyDistance=0.4;
VFH.MinTurningRadius=0.1;
VFH.TargetDirectionWeight=10;
VFH.CurrentDirectionWeight=2;
VFH.PreviousDirectionWeight=2;
VFH.HistogramThresholds=[3 10];
VFH.UseLidarScan=true; %para permitir utilizar la notación del scan
%Inicializar el localizador AMCL (práctica 1)%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%iniciamos odometria
odometryModel = odometryMotionModel;
odometryModel.Noise = [0.2 0.2 0.2 0.2];
rangeFinderModel = likelihoodFieldSensorModel;
rangeFinderModel.SensorLimits = [0 8];
rangeFinderModel.Map = map;
tftree = rostf;
%Obtener transformada entre los frames del robot y del sensor_laser
waitForTransform(tftree,'/robot0','/robot0_laser_1');
sensorTransform = getTransform(tftree,'/robot0', '/robot0_laser_1');
% Get the euler rotation angles.
laserQuat = [sensorTransform.Transform.Rotation.W sensorTransform.Transform.Rotation.X sensorTransform.Transform.Rotation.Y sensorTransform.Transform.Rotation.Z];
laserRotation = quat2eul(laserQuat, 'ZYX');
rangeFinderModel.SensorPose =[sensorTransform.Transform.Translation.X sensorTransform.Transform.Translation.Y laserRotation(1)];
%% Initialize AMCL object
% Instantiate an AMCL object |amcl|. See <docid:nav_ref.bu31hfz-1 monteCarloLocalization> 
% for more information on the class.
amcl = monteCarloLocalization;
amcl.UseLidarScan = true;
 
% Assign the |MotionModel| and |SensorModel| properties in the |amcl| object.
amcl.MotionModel = odometryModel;
amcl.SensorModel = rangeFinderModel;
amcl.UpdateThresholds = [0.2,0.2,0.2];
amcl.ResamplingInterval = 1;

%% Configure AMCL object for localization with initial pose estimate.
amcl.ParticleLimits = [500 10000];           % Minimum and maximum number of particles
amcl.GlobalLocalization = false;      % global = true      local=false
% The particle filter only updates the particles when the robot's movement exceeds the 
amcl.InitialPose = [0 0 0];              % Initial pose of vehicle   
amcl.InitialCovariance = diag([0.5 0.5 0.5])*2; % Covariance of initial pose
%diag([0.5 0.5 0.5])*...
%% Setup helper for visualization and driving AmigoBot.
% Setup ExampleHelperAMCLVisualization to plot the map and update robot's estimated pose, particles,
% and laser scan readings on the map.
visualizationHelper = ExampleHelperAMCLVisualization(map);
%% Crear el objeto PurePursuit y ajustar sus propiedades
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
controller=controllerPurePursuit;
controller.LookaheadDistance = 2;
controller.DesiredLinearVelocity= 0.1;
controller.MaxAngularVelocity =  0.5;

%% Rellenamos los campos por defecto de la velocidad del robot, para que la lineal
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

umbralx=0.1;
umbraly=0.1;
umbralyaw=0.1;
i=1;
while(1)

 %Leer y dibujar los datos del láser en la figura ‘fig_laser’
 figure(fig_laser);
 lee_sensores2;
 %Leer la odometría
 scan = receive(sub_laser);
 odompose = sub_odom.LatestMessage;
 scans = lidarScan(scan);
 %Obtener la posición pose=[x,y,yaw] a partir de la odometría anterior
 %Ejecutar amcl para obtener la posición estimada estimatedPose y la
 %covarianza estimatedCovariance (mostrar la última por pantalla para
 odomQuat = [odompose.Pose.Pose.Orientation.W, odompose.Pose.Pose.Orientation.X,odompose.Pose.Pose.Orientation.Y, odompose.Pose.Pose.Orientation.Z];
 odomRotation = quat2eul(odomQuat);
 pose = [odompose.Pose.Pose.Position.X, odompose.Pose.Pose.Position.Y odomRotation(1)];
 % Update estimated robot's pose and covariance using new odometry and sensor readings.
 [isUpdated,estimatedPose, estimatedCovariance] = amcl(pose, scans);
 %facilitar la búsqueda de un umbral)


     % Plot the robot's estimated pose, particles and laser scans on the map.
 if isUpdated
    i = i + 1;
    plotStep(visualizationHelper, amcl, estimatedPose, scans, i)
 end
 %Si la covarianza está por debajo de un umbral, el robot está localizado y
 %finaliza el programa

 if (estimatedCovariance(1,1)<umbralx && estimatedCovariance(2,2)<umbraly &&estimatedCovariance(3,3)<umbralyaw)
    disp('Robot Localizado');
    break;
 end
 %Dibujar los resultados del localizador con el visualizationHelper
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

%%%%%%%%%%% AL SALIR DE ESTE BUCLE EL ROBOT YA SE HA LOCALIZADO %%%%%%%%%%
%%%%%%%%%%% COMIENZA LA PLANIFICACIÓN GLOBAL %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Paramos el robot, para que no avance mientras planificamos
msg_vel.Linear.X=0;
msg_vel.Linear.Y=0;
msg_vel.Linear.Z=0;
msg_vel.Angular.X=0;
msg_vel.Angular.Y=0;
msg_vel.Angular.Z=0;
send(pub_vel,msg_vel);
%Hacemos una copia del mapa, para “inflarlo” antes de planificar
cpMap= copy(map);
inflate(cpMap,0.5);
%Crear el objeto PRM y ajustar sus parámetros
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
planner = mobileRobotPRM;
planner.Map=cpMap;
planner.NumNodes=1000;
planner.ConnectionDistance=2;
%Obtener la ruta hacia el destino desde la posición actual del robot y mostrarla
%en una figurasssssss
startPosition=estimatedPose(1:2);
ruta=findpath(planner,startPosition,endLocation);
figure; show(planner);



%%%%%%%%%%% COMIENZA EL BUCLE DE CONTROL %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Indicamos al controlador la lista de waypoints a recorrer (ruta)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
controller.Waypoints= ruta;

%Bucle de control
%%%%%%%%%%%%%%%%%
while(1)
    %Leer y dibujar los datos del láser en la figura ‘fig_laser’
 figure(fig_laser);
 lee_sensores2;
 %Leer la odometría
 scan = receive(sub_laser);
 odompose = sub_odom.LatestMessage;
 scans = lidarScan(scan);

 %Obtener la posición pose=[x,y,yaw] a partir de la odometría anterior
 
 %Ejecutar amcl para obtener la posición estimada estimatedPose y la
 %covarianza estimatedCovariance (mostrar la última por pantalla para
 %facilitar la búsqueda de un umbral)
 odomQuat = [odompose.Pose.Pose.Orientation.W, odompose.Pose.Pose.Orientation.X,odompose.Pose.Pose.Orientation.Y, odompose.Pose.Pose.Orientation.Z];
 odomRotation = quat2eul(odomQuat);
 pose = [odompose.Pose.Pose.Position.X, odompose.Pose.Pose.Position.Y odomRotation(1)];
 % Update estimated robot's pose and covariance using new odometry and sensor readings.
 [isUpdated,estimatedPose, estimatedCovariance] = amcl(pose, scans);

 % Plot the robot's estimated pose, particles and laser scans on the map.
 if isUpdated
    i = i + 1;
    plotStep(visualizationHelper, amcl, estimatedPose, scans, i)
 end

 %Si la covarianza está por debajo de un umbral, el robot está localizado y
 %finaliza el programa

    %Ejecutar el controlador PurePursuit para obtener las velocidades lineal
    %y angular
    [lin_vel, ang_vel] = controller(estimatedPose);
    ang_vel
    dir_corregida = VFH(scans,ang_vel)
    figure(fig_vfh);
    show(VFH);
    ang_vel_corregida=0.7*dir_corregida

    %Rellenar los campos del mensaje de velocidad
    msg_vel.Linear.X = lin_vel;
    msg_vel.Angular.Z = ang_vel_corregida;

    %Publicar el mensaje de velocidad
    send(pub_vel, msg_vel);
    
    %Comprobar si hemos llegado al destino, calculando la distancia euclidea
    %y estableciendo un umbral
    % currentPose = [estimatedPose(1), estimatedPose(2)];
    % distanceToGoal = norm(currentPose - endLocation);
    if(sqrt((endLocation(1) -estimatedPose(1))^2+(endLocation(2)-estimatedPose(2))^2)<0.5)
    % if (distanceToGoal < umbralDistancia)
        %Parar el robot
        msg_vel.Linear.X = 0;
        msg_vel.Angular.Z = 0;
        send(pub_vel, msg_vel);
        disp('Destino alcanzado');
        break;
    end
    
    %Esperar al siguiente periodo de muestreo
    waitfor(r);
end
