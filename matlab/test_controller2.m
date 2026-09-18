function test_controller2()
% This function is executed after the update data timer

% global ePic object. Use get and set methods to access the different
% fields and check if the required sensors are activated using the 
% updateDet methode. For more information about this different commands, 
% please read the help file.
global ePic;  
xg=36;
local_minima_Flag=0;
U=-100000;
yg=24.5;
ka=10;
kw=2500;
kv=6;
xv=[22 26 7 22];
yv=[9 25 21 9];
rho_0=6;
kr=5000;
de=2;
ke=1000;
xtp=0;
ytp=0;
load('x_p.mat','t')
load('flag.mat','local_minima_Flag')
% a global variable for the controller state
global ControllerState; % 0 = controller halted, all other states are active transition or static states;

% Controller states (content of ControllerState variable)
% 0 = "controller off" state
% 1 = transition to "controller on" state is in progress (initiated by user in main.m)
% -1 = transition to "controller off" state is in progress (initiated by user in main.m)
% -2 = means that the controller is in "suspend" state (automatically activated when control goal has been reached)
%
% all other states can be used freely in this function (e.g. to signify "controller on" state)



% put your controller variable declarations here (if possible not declared
% as "global" but as "persistent")
persistent done;


%-----------------------------------------------%
% main code for the different controller states %
%-----------------------------------------------%

%-------------------------------------------------------------------------%
% if controller is to be switched on, execute initialization code and go to
% "on"-state
if (ControllerState==1) 
  disp 'Controller has been switched on!';
  ControllerState = 2;
  
  %----------------------------%
  % setup controller variables %  
  %----------------------------%
  
  % ******* PUT YOUR CONTROLLER INITIALISATION CODE HERE ********
  % you can comment of edit these lines to write you controller
  
  % activate the requiered sensors
  ePic = deactivate(ePic,'accel');
  ePic = deactivate(ePic,'proxi');
  %ePic = activate(ePic,'image'); % activate the camera
  % ...... add other sensors
  
  % desactivate the unrequired sensors
  ePic = deactivate(ePic, 'light');
  % ...... add other sensors
  
  % reset odometry
  ePic = set(ePic, 'odom' , [0.04 0.04 0]);
  
  % put your variable initialization code here
  done = 0;
  

%-------------------------------------------------------------------------%  
% if controller is to be switched off, execute termination code and go to "off"-state  
elseif (ControllerState==-1)
  ePic = set(ePic,'speed',[0 0]);
  disp 'Controller has been switched off!';
  ControllerState = 0;
  

%-------------------------------------------------------------------------%  
% if controller is in suspend state
elseif (ControllerState==-2)
  % don't do anything, and wait for user to switch controller off
  

%-------------------------------------------------------------------------%  
% controller running, write your own code here
elseif (ControllerState~=0)  

 if (done==0)    
     % ******* PUT YOUR CONTROLLER MAIN CODE HERE ********
     
     % read the sensors values
     [val, up] = get(ePic, 'odom'); % read accelerometer values
     % ........ read other sensors
     [d,x_poly,y_poly] = p_poly_dist(val(1)*100, val(2)*100, xv, yv);
     
     x_c=[val(1)*100 val(2)*100]';
  
     if(d < rho_0)
        fx_r=kr*(1/d-1/rho_0)*(1/d^2)*(val(1)*100-x_poly)/d;
        fy_r=kr*(1/d-1/rho_0)*(1/d^2)*(val(2)*100-y_poly)/d;
     else
         fx_r=0;fy_r=0;
     end
     fx_a=ka*(xg-val(1)*100);
     fy_a=ka*(yg-val(2)*100);
     t=[t(:,2:end) x_c];
     t_std=std(t');
     if norm(t_std)<=1.4/1.2 && norm(val(1:2)*100-[xg yg])>9 && norm(val(1:2)*100-[4 4])>5
        local_minima_Flag=1;
        save('flag.mat','local_minima_Flag');
        x_center=mean(t');
        u=val(1:2)*100-[xg yg];
        u=u/norm(u);
        u=[u(2) -u(1)];
        xtp=x_center(1)+u(1)*4;
        ytp=x_center(2)+u(2)*4;
%         for l=(x_center(1)-3*t_std(1)):0.1:(x_center(1)+3*t_std(1))
%         for w=(x_center(2)-3*t_std(2)):0.1:(x_center(2)+3*t_std(2))
%         fx_a1=ka*(xg-l);
%         fy_a1=ka*(yg-w);
%         fx_r1=kr*(1/d-1/rho_0)*(1/d^2)*(l-x_poly)/d;
%         fy_r1=kr*(1/d-1/rho_0)*(1/d^2)*(w-y_poly)/d;
%         [U,I]=max(U,fx_a1*(-fx_r1)+fy_a1*(-fy_r1));
%         if I==2
%             xtp=l;
%             ytp=w;
%         end
%         end
%         end
     end
     if local_minima_Flag==1 && norm(val(1:2)*100-[xg yg])>=12
         if norm(val(1:2)-[xtp ytp])<de
             f_ext=ke/de*(val(1:2)-[xtp ytp]);
         else
             f_ext=ke*(val(1:2)-[xtp ytp])/norm((val(1:2)-[xtp ytp]));%*(1-exp(-(norm(val(1:2)-[xg yg]))^2/10));
         end
    else
         f_ext=[0 0];
     end
     fx=fx_a+fx_r+f_ext(1);
     fy=fy_a+fy_r+f_ext(2);  
     alpha=atan2(fy,fx)-val(3);
     V=kv*norm([fx fy])*cos(alpha);
     W=kw*alpha;
     vl=V-W;
     vr=V+W;
     [ f_ext  fx_a+fx_r  fy_a+fy_r (1-exp(-(norm(val(1:2)-[xg yg]))^2/4)) d]  
     % if you want to use the camera from the ePuck, uncomment
     % thoses lines and activate camera during initialisation
     % ePic = updateImage(ePic);
     % [image, up] = get(ePic, 'image');
     
    % determine somewhere here, if variable "done" has to be set to 1 to
    % end the controller
     
    % finally set motor speeds
    ePic = set(ePic,'speed',[vl,vr]/2);
  save('x_p.mat','t');
 else
    ControllerState = -2;  % go to suspend state
    
 end  
  
end