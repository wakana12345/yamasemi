/*URGの取得データ表示とロボットを動かすプログラム               */ 
/*URGの取得データによりロボットの動きが変化するものではなく、   */
/*単純に同じファイル上でURG、ロボットをどちらも動かすだけである */


#include <cmath>
#include <cstdio>

#include <unistd.h>
#include <signal.h>
#include <scip2awd.h>

#include <ypspur.h>
#ifdef __WIN32
# include <windows.h>
#endif


int gIsShuttingDown;

/* グローバル変数の定義 */
int ret;
S2Port   *urg_port;   /* port */
S2Sdd_t   urg_buff;   /* buffer */
S2Scan_t *urg_data;   /* pointer to buffer */
S2Param_t urg_param;  /* parameter */


/*** Ctrl + C を押された場合 shutdown 関数 ***/
void ctrl_c(int aStatus) {
  /* exit loop */
  gIsShuttingDown = 1;
  signal(SIGINT, NULL);
}


/*** 実行時の引数のエラー確認関数 ***/
void check_argument(int &argc_a, char **argv_a)
{
  /* check argument */
  if( argc_a < 2 ) 
  {
    fprintf(stderr, "ERORR: missing device operand\n");
    fprintf(stderr, "USAGE: %s <device>\n", argv_a[0]);
    fprintf(stderr, " e.g.: %s /dev/ttyACM0\n", argv_a[0]);
//    return 1;
  }
}




/*** URG オープン関数 ***/
bool urg_port_open(char **argv_a)
{
  urg_port = Scip2_Open(argv_a[1], B115200);
  if(urg_port == 0) 
  {
    fprintf(stderr, "ERORR: cannot open %s\n", argv_a[1]);
    return 1;
  }
  fprintf(stdout, "port opened\n");


  /* init buffer */
  S2Sdd_Init(&urg_buff);

  /* get paramerter */
  Scip2CMD_PP(urg_port, &urg_param);

  /* scan start */
  //Scip2CMD_StartMS(urg_port, 44, 725, 1, 0, 0, &urg_buff, SCIP2_ENC_3BYTE);
  ret = Scip2CMD_StartMS(urg_port, urg_param.step_min, urg_param.step_max, 1, 0, 0, &urg_buff, SCIP2_ENC_3BYTE );
  if(ret == 0) 
  {
    fprintf(stderr, "ERROR: Scip2CMD_StartMS\n");
    return false;
  }
 fprintf(stdout, "scan started\n");

 return true;
}





/*** GNU 画像の軸設定関数  ***/
void open_pipe_gnu(FILE *fp)
{

  fputs("set terminal x11\n", fp);   /* drawing destination */
  fputs("set grid\n", fp);  /* draw grid */
  fputs("set mouse\n", fp);  /* use mouse */
  fputs("set xlabel \"y [m]\"\n", fp);  /* label of x-axis */
  fputs("set ylabel \"x [m]\"\n", fp);  /* label of y-axis */
  fputs("set xrange [-6:6] reverse\n", fp);  /* range of x-axis */
  fputs("set yrange [-6:6]\n", fp);  /* range of y-axis */
  fputs("set size ratio -1\n", fp);  /* aspect ratio */
  fputs("unset key\n", fp);  /* hide graph legends */
}



/*** URGのデータ取得とプロット関数 ***/
void urg_distance(FILE *fp)
{
  int i;
  double x, y, rad;
//  double rad_front, rad_min, rad_max;
//  int flag_go; // 0or1を取って「1:Go 0:Stop」

    /* lock buffer */
  ret = S2Sdd_Begin(&urg_buff, &urg_data);

  if(ret > 0) 
  {
    fputs("plot '-'\n", fp);
    for(i = 0; i < urg_data->size; ++i) 
    {
      if(urg_data->data[i] < 20) 
      { /* error code */
        continue;
      }

      rad = (2 * M_PI / urg_param.step_resolution) * (i + urg_param.step_min - urg_param.step_front);
      x = urg_data->data[i] * cos(rad) / 1000.0;
      y = urg_data->data[i] * sin(rad) / 1000.0;
      fprintf(fp,"%f %f\n", y, x);
       
    }
    fputs("e\n",fp);
     
    /* unlock buffer */
    S2Sdd_End(&urg_buff);

    /* wait 90 ms (URG-04LX 1scan = 100 ms) */
    usleep(90000);
  } 
  
  /*else if(ret == -1) 
  {
    fprintf(stderr, "ERROR: S2Sdd_Begin\n");
    break;
  }
  */
  else
  {
    usleep(100);
  }
}
    
 
/*** GNU のパイプをクローズ関数 ***/
bool close_gnu(FILE *fp)
{
  /* close pipe */    
  pclose(fp);
  fprintf(stdout, "pipe closed\n");

  /* scan stop */ 
  ret = Scip2CMD_StopMS(urg_port, &urg_buff);
  if(ret == 0) 
  {
    fprintf(stderr, "ERROR: Scip2CMD_StopMS\n");
    return false; 
  }

    fprintf(stdout, "scan stopped\n");

  /* destruct buffer */
  S2Sdd_Dest(&urg_buff);

  /* close port */     
  Scip2_Close(urg_port);
  fprintf(stdout, "port closed\n");

  return true;
}
 
 
 /************************** MAIN文 **************************/
 
    
 
int main(int argc, char **argv) 
{ 

  // Windows環境で標準出力がバッファリングされないように設定
  setvbuf( stdout, 0, _IONBF, 0 ); 

  // 初期化
  if(Spur_init(  ) < 0 )
  {
    fprintf(stderr, "ERROR : cannot open spur.\n");
    return -1;
  }
     
     
  //ypspur初期設定
  Spur_set_vel(0.2);
  Spur_set_accel(1.0);
  Spur_set_angvel(M_PI/2.0);
  Spur_set_angaccel(M_PI / 2.0);

  //初期位置設定
  Spur_set_pos_GL( 0.0, 2.0, 0 );
  Spur_line_GL( 1.5, 1.0, -M_PI/4 );
  while( !Spur_near_pos_GL( 3.0 - 0.005, 0.0 + 0.005, 0.5 ) )
    usleep( 10000 );


  FILE *fp;
  int arg_number = argc;
  char **urg_file_ptr = argv;

  /* set Ctrl-c function */
  signal(SIGINT, ctrl_c);

  /*実行時のエラー確認*/
  check_argument(arg_number, urg_file_ptr);

  /*urgポートのオープン*/
  urg_port_open(urg_file_ptr);

  /*GNUのパイプオープン*/
  fp = popen("gnuplot -noraise", "w");
  if(fp == NULL) 
  {
    fprintf(stderr, "ERROR: popen\n");
    return 1;
  }
  fprintf(stdout, "pipe opened\n");
  open_pipe_gnu(fp);


  /*urgのデータ取得*/
  while(!gIsShuttingDown)
  {
    urg_distance(fp);
  }

  /*GNUのパイプクローズ*/
  close_gnu(fp);

  /* init buffer */
  S2Sdd_Init(&urg_buff);


  return 0;
}





