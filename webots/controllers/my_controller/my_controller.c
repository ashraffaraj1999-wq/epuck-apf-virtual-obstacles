#include <stdio.h>
#include <webots/accelerometer.h>
#include <webots/camera.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/light_sensor.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>
#include <float.h>

#define TIME_STEP 32
#define ON 1
#define OFF 0
#define NB_LEDS 10
#define NB_LIGHT_SENS 8
#define NB_DIST_SENS 8
#define WHEEL_RADIUS 0.0205
#define AXLE_LENGTH 0.052
#define ENCODER_RESOLUTION 159.23
#define RANGE (1024 / 2)

double prev_l=0;
double prev_r=0;
bool first=true;
double l,r;
double x,y,th,prev_x=0.5,prev_y=0.5,prev_th=0;
const double yv[]={22+46,26+46,7+46,22+46};
const double xv[]={9+46,25+46,21+46,9+46};
double xwindow[30]={0};
double ywindow[30]={0};
double norm_std=1000;
double window_length=30;
double d,x_poly,y_poly;
double Vl=0,Vr=0;
double xtp=-1,ytp=-1;
bool local_minima=false;
double meanx,meany;
double SDX,SDY;
double de=0.05;
double ke=1000;
double virtual [50][2]={0};
int numvirt=0;
double counter=0;
double sigma=4;
bool first_virt=true;
int wait_time=120;

double xg=36+46;
double yg=24.5+46;
double ka=20;
double kw=800;
double kv=6;
double rho_0=8;
double kr=1000000;

WbDeviceTag left_motor, right_motor;
WbDeviceTag left_position_sensor, right_position_sensor;

static void compute_odometry()
{
  l = wb_position_sensor_get_value(left_position_sensor);
  r = wb_position_sensor_get_value(right_position_sensor);
  double dl=(l-prev_l);
  double dr=(r-prev_r);
  
  th=prev_th+WHEEL_RADIUS*(dl-dr)/AXLE_LENGTH;
  x=prev_x+cos(th)*WHEEL_RADIUS*(dl+dr)/2;
  y=prev_y+sin(th)*WHEEL_RADIUS*(dl+dr)/2;
  
  prev_l=l; prev_r=r; prev_th=th; prev_x=x;prev_y=y;
}

void p_poly_dist()
{
    double A,C,xp,yp,B,AB,vv,min_v=10000,min_p=10000,xp_min=0,yp_min=0;
    int indx_v=0;
    
    for (int i=1;i<=3;i++)
    {
        if (sqrt(pow(xv[i-1]-100*x,2)+pow(yv[i-1]-100*y,2))<min_v)
        {
            indx_v=i-1;
            min_v=sqrt(pow(xv[i-1]-100*x,2)+pow(yv[i-1]-100*y,2));
        }
        A=-(yv[i]-yv[i-1]);
        B=(xv[i]-xv[i-1]);
        C=yv[i]*xv[i-1]-yv[i-1]*xv[i];
        AB=1/(A*A+B*B);
        vv=A*100*x+B*100*y+C;
        xp=100*x-A*AB*vv;
        yp=100*y-B*AB*vv;
        if ((((xp>=xv[i-1]) && (xp<=xv[i])) || ((xp>=xv[i]) && (xp<=xv[i-1]))))
        {
            if ((((yp>=yv[i-1]) && (yp<=yv[i])) || ((yp>=yv[i]) && (yp<=yv[i-1]))))
            {
                 if (sqrt(pow(xp-100*x,2)+pow(yp-100*y,2))<min_p)
                 {
                   xp_min=xp;
                   yp_min=yp;
                   min_p=sqrt(pow(xp-100*x,2)+pow(yp-100*y,2));
                 }
            }
        }
    }
    
    if (min_v<=min_p)
    {
        d=min_v;
        x_poly=xv[indx_v];
        y_poly=yv[indx_v];
    }
    else
    {
        d=min_p;
        x_poly=xp_min;
        y_poly=yp_min;
        
    }
}

void std() 
{
    double sumx = 0.0;
    double sumy = 0.0;
    int i;
    for (i = 0; i < window_length; ++i) {
        sumx += xwindow[i];
        sumy += ywindow[i];
    }
    meanx = sumx / window_length;
    meany = sumy / window_length;
    for (i = 0; i < window_length; ++i) 
    {
        SDX += pow(xwindow[i] - meanx, 2);
        SDY += pow(ywindow[i] - meany, 2);
    }
    SDX=sqrt(SDX/(window_length-1));
    SDY=sqrt(SDY/(window_length-1));
}

void mycontroller()
{
    double fx_r,fy_r,fx_a,fy_a,fx,fy,alpha,V,W;
   // double fx_a1,fy_a1,fx_r1,fy_r1;
    double goal_dist=sqrt(pow(x*100-xg,2)+pow(y*100-yg,2));
    double start_dist=sqrt(pow(x*100-50,2)+pow(y*100-50,2));
    std(); 
    double norm_std=sqrt((pow(SDX,2)+pow(SDY,2)));
    double fext_x=0,fext_y=0;
    p_poly_dist();
    printf("d = %f , ",d);
    if (d<rho_0)
    {
        fx_r=kr*(1/d-1/rho_0)*(1/pow(d,2))*(x*100-x_poly)/d;
        fy_r=kr*(1/d-1/rho_0)*(1/pow(d,2))*(y*100-y_poly)/d;
    }
    else
    {
        fy_r=0;
        fx_r=0;
    }
    fx_a=ka*(xg-x*100);
    fy_a=ka*(yg-y*100);
    double angle=0;
    
    if(fy_r!=0 || fx_r!=0)
    {
      double dot=fx_a*fx_r+fy_a*fy_r;
      double fa_norm=sqrt((pow(fx_a,2)+pow(fy_a,2)));
      double fr_norm=sqrt((pow(fx_r,2)+pow(fy_r,2)));
      angle=acos(dot/(fa_norm*fr_norm))*180/(3.141592653589);
    }
    bool cond=(norm_std<=0.05 && goal_dist>9 && start_dist>3 && (counter>=wait_time || first_virt));
    if (norm_std<=0.05 && goal_dist>9 && start_dist>3 && (counter>=wait_time || first_virt) && angle>90)
    {
        local_minima=true;
        first_virt=false;
        counter=0;
        virtual[numvirt][0]=meanx; virtual[numvirt][1]=meany;
        numvirt++;
    }
    if(local_minima)
    {
      counter++;
      for(int i=0;i<numvirt;i++)
      {
          if(sqrt(pow(x-virtual[i][0],2)+pow(y-virtual[i][1],2)) < de)
          {
            fext_x+=(ke/de)*(x-virtual[i][0]);
            fext_y+=(ke/de)*(y-virtual[i][1]);
          }
          else 
          {
            fext_x+=0.001*ke*(x-virtual[i][0])/sqrt(pow(x-virtual[i][0],2)+pow(y-virtual[i][1],2));
            fext_y+=0.001*ke*(y-virtual[i][1])/sqrt(pow(x-virtual[i][0],2)+pow(y-virtual[i][1],2));
          }
      }
    
    }
    fext_x*=(1-exp(-pow(goal_dist,2)/(sigma)));
    fext_y*=(1-exp(-pow(goal_dist,2)/(sigma)));
    
        
     /*   double U=-DBL_MAX;
        double tempx=(x-3*SDX),tempy=(y-3*SDY);
        for(int i=1;i<=20;i++)
          for(int j=1;j<=20;j++)
          {
              tempx+=xstep;
              tempy+=ystep;
              fx_a1=ka*(xg-tempx);
              fy_a1=ka*(yg-tempx);
              fx_r1=kr*(1/d-1/rho_0)*(pow(d,-2))*(tempx-x_poly)/d;
              fy_r1=kr*(1/d-1/rho_0)*(pow(d,-2))*(tempy-y_poly)/d;
          
              if(U<fx_a1*(-fx_r1)+fy_a1*(-fy_r1))  
              {
                 U=fx_a1*(-fx_r1)+fy_a1*(-fy_r1);
                 xtp=tempx;
                 ytp=tempy;
              }
          }
    }
    if (local_minima)
         if (sqrt(pow(x-xtp,2)+pow(y-ytp,2))<de)
         {
             f_extx=ke/de*(x-xtp);
             f_exty=ke/de*(y-ytp);
         }
         else
         {
             f_extx=ke*(x-xtp)/sqrt(pow(x-xtp,2)+pow(y-ytp,2));
             f_exty=ke*(y-ytp)/sqrt(pow(x-xtp,2)+pow(y-ytp,2));
         }
     else
     {
         f_extx=0;
         f_exty=0;
     }*/
    fx=fx_a+fx_r+fext_x;
    fy=fy_a+fy_r+fext_y;
    alpha=atan2(fy,fx)-th;
    V=kv*sqrt(pow(fx,2)+pow(fy,2))*cos(alpha);
    W=kw*alpha;
    Vl=V+W;
    Vr=V-W;
    printf("fx_a=%f ,fy_a=%f , fx_r=%f , fy_r=%f , fext_x=%f ,fext_y=%f , ",fx_a,fy_a,fx_r,fy_r,fext_x,fext_y);
    printf("angle=%f\n",angle);
    printf("norm_std=%f , local_minima=%d , cond=%d , numvirt=%d\n",norm_std,local_minima,cond,numvirt);
}
/*
static void compute_odometry() {
  double l = wb_position_sensor_get_value(left_position_sensor);
  double r = wb_position_sensor_get_value(right_position_sensor);
  double dl = l / ENCODER_RESOLUTION * WHEEL_RADIUS; // distance covered by left wheel in meter
  double dr = r / ENCODER_RESOLUTION * WHEEL_RADIUS; // distance covered by right wheel in meter
  double da = (dr - dl) / AXLE_LENGTH; // delta orientation
  printf("estimated distance covered by left wheel: %g m.\n",dl);
  printf("estimated distance covered by right wheel: %g m.\n",dr);
  printf("estimated change of orientation: %g rad.\n",da);
}*/


int main(int argc, char *argv[]) {


  /*int it, m, n;
  double position_sensor_offest[2] = {0, 0};
*/
  /* initialize Webots */
  wb_robot_init();

  /* get and enable devices 
  char text[5] = "led0";
  for (it = 0; it < NB_LEDS; it++) {
    led[it] = wb_robot_get_device(text);
    text[3]++;
    wb_led_set(led[it], OFF);
  }
  char textPS[] = "ps0";
  for (it = 0; it < NB_DIST_SENS; it++) {
    ps[it] = wb_robot_get_device(textPS);
    textPS[2]++;
    wb_distance_sensor_enable(ps[it], 2 * TIME_STEP);
  }

  char textLS[] = "ls0";
  for (it = 0; it < NB_LIGHT_SENS; it++) {
    ls[it] = wb_robot_get_device(textLS);
    textLS[2]++;
    wb_light_sensor_enable(ls[it], 2 * TIME_STEP);
  }*/
  Vl=0;Vr=0;
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor,INFINITY);
  
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);
  
  left_position_sensor = wb_robot_get_device("left wheel sensor");
  right_position_sensor = wb_robot_get_device("right wheel sensor");
  wb_position_sensor_enable(left_position_sensor, TIME_STEP);
  wb_position_sensor_enable(right_position_sensor, TIME_STEP);
  /* main loop */
  while (wb_robot_step(TIME_STEP) != -1) 
  {
    if(first)
    { 
        prev_l = wb_position_sensor_get_value(left_position_sensor);
        prev_r = wb_position_sensor_get_value(right_position_sensor);
        first =false;
    }
    else 
    {
      compute_odometry();
    //printf("x=%f,y=%f,th=%f\n",x,y,th);
      for (int i=29; i>0;i--)
      {
        xwindow[i]=xwindow[i-1];
        ywindow[i]=ywindow[i-1];
      }
      xwindow[0]=x;
      ywindow[0]=y;
      mycontroller();
      wb_motor_set_velocity(left_motor, Vl/1024.0);
      wb_motor_set_velocity(right_motor, Vr/1024.0);
    }
    /*const unsigned char *im = wb_camera_get_image(cam);
    for (m = 0; m < camera_width; m++) {
      for (n = 0; n < camera_height; n++)
        sum += wb_camera_image_get_gray(im, camera_width, m, n);
    }

    const double *a = wb_accelerometer_get_values(accelerometer);

    for (it = 0; it < NB_DIST_SENS; it++)
      count += wb_distance_sensor_get_value(ps[it]);

    for (it = 0; it < NB_LEDS; it++)
      wb_led_set(led[it], OFF);

    if (wb_robot_get_mode() == 1)
      wb_led_set(led[0], 1);

    if (a[0] <= 0.0 && a[1] <= 0.0)
      wb_led_set(led[1], 1);
    if (a[0] <= 0.0 && a[1] > 0.0)
      wb_led_set(led[3], 1);
    if (a[0] > 0.0 && a[1] > 0.0)
      wb_led_set(led[5], 1);
    if (a[0] > 0.0 && a[1] <= 0.0)
      wb_led_set(led[7], 1);

    if (count > 7000)
      wb_led_set(led[8], 1);
    if (sum < 100000)
      wb_led_set(led[9], 1);
*/


    /*if ((wb_position_sensor_get_value(left_position_sensor) - position_sensor_offest[0]) > (2 * M_PI) ||
        (wb_position_sensor_get_value(left_position_sensor) - position_sensor_offest[0]) < -(2 * M_PI)) {
      if (direction == 1)
        direction = -1;
      else
        direction = 1;
      position_sensor_offest[0] = wb_position_sensor_get_value(left_position_sensor);
      printf("Other direction\n");
    }

    wb_motor_set_velocity(left_motor, 1.88 * direction);
    wb_motor_set_velocity(right_motor, -1.88 * direction);*/
  }

  wb_robot_cleanup();

  return 0;
}
