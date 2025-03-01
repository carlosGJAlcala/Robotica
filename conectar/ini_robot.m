%% DECLARACIÓN DE SUBSCRIBERS
sub_odom=rossubscriber('/pose');
%Laser 
sub_laser=rossubscriber('/scan');

%Sonars
sub_sonar0=rossubscriber('/sonar_0');
sub_sonar1=rossubscriber('/sonar_1');
sub_sonar2=rossubscriber('/sonar_2');
sub_sonar3=rossubscriber('/sonar_3');
sub_sonar4=rossubscriber('/sonar_4');
sub_sonar5=rossubscriber('/sonar_5');
sub_sonar6=rossubscriber('/sonar_6');
sub_sonar7=rossubscriber('/sonar_7');
%Declaracion de publis
pub_vel = rospublisher('/cmd_vel', 'geometry_msgs/Twist'); 
pub_enable= rospublisher('/cmd_motor_state','std_msgs/Int32');
msg_enable_motor=rosmessage(pub_enable);
msg_enable_motor.Data=1;
send(pub_enable,msg_enable_motor);
%generacion de mensajes
msg_vel=rosmessage(pub_vel);
%Definimos la periodicidad del bucle
r=rateControl(10);
disp('Inicializacion finalizada correctamente');
