/***********************************************************/
/* IRC Preparation                                         */
/* Week VII                                                */
/* PID Trials                                              */
/* MHammad                                                 */
/* Date : 21.10.14                                         */
/* Grid.ino                                                */
/* Co-ordinates and Orientation                            */
/* Dry && Maze && Flood Fill && BFS                        */
/***********************************************************/

/**************** Headers **********************/
#include <EEPROM.h>
#include <QueueList.h>

/**************** Constants ********************/
#define Right 5
#define Left 3
#define M1_r 7
#define M1_b 6
#define M2_r 2
#define M2_b 1

#define avg_speed 95

#define Kp 12
#define Ki 0 
#define Kd 17

#define AREA 50
#define LENGTH 10 // 9
#define UNDEFINED -10

#define arm_en 8
#define arm_r 10
#define arm_b 9
#define gripper_r 19
#define gripper_b 18
#define gripper_en 12

#define buzz 13

/****************** Ismail Functions **************/
void UpdateCoordinate();
void UpdateOrintation();
char CheckOrintation();
void CheckForTurn();
void TurnRight();
void TurnLeft();
void TakeAction();

void Check_Wing();
void Push();

void calculateProportional(); 

void PID_Motion();

char orintation = 'N';
char Direction = 'R';
int error = 0; 
int lastError = 0; 

boolean FlagR = true;
boolean FlagL = true;
boolean NFlag = false;
boolean TFlag = false;

/***************** Variables *********************/
// sensor init
int sensor[5] = {
  46, 44, 42, 40, 38};
int sensor_read[5];
int sensor_sum = 0;

int Left_sensor =14;
int Right_sensor =15;
int Front_sensor = 16;
boolean Left_read;
boolean Right_read;
boolean Front_read;

// PID init
float pos;
float proportional = 0;
float integral = 0;
float derivative = 0;
float error_val;
float last = 0;

// Motor
int right_speed = 0;
int left_speed = 0;

// Grid
char orientation = 'N';
int pos_x = 0;
int pos_y = -1;

int current_x = 0;
int current_y = 0;

// Target
int next_x;
int next_y;

int target_x[8]= {0, 4, 4, 0, 4, 4, 3, 2};
int target_y[8] = {0, 4, 7, 0, 8, 9, 9, 9};
int target_findx[2];
int target_findy[2];
int ptr = 0;
int task_ptr = 0;
void Calc_Target();
// Maze

char main_dir = 'y';

void next_coor();
void Stop();
void Wait();
void Adjust();

//Task
//void Tasks();
void Arm_up();
void Arm_down();
void Gripper_open();
void Gripper_close();
void Enter_District();
void Back();
void Forwards();
// path
int Filled[5][10] = {
{1,1,1,1,1,1,1,1,1,1},
{1,1,1,1,1,1,1,1,1,1},
{1,1,1,1,1,1,1,1,1,1},
{1,1,1,1,1,1,1,1,1,1},
{1,1,1,1,1,1,1,1,1,1}
};
//volatile int flag_black = 1;
//volatile int flag_target = 1;
//void Set_Filled();

// Dry Run

boolean Dry_run;
boolean flag_read = true;
void Snail();
//void Set_State();
void Set_Map();

// BFS
int rowColtoZ(int row, int col);
void zToRowCol(int z, int &row, int &col);

void BFS(int curr_x, int curr_y, int tar_x, int tar_y);
void MakePath();
QueueList <int> path;

void initializeFloodfill();
void initPath();
int floodfillArr[AREA] = {
};
int visited[AREA]= {
};
boolean passed(int temp);


int temp;



/*********** Prototypes *******************/

void CheckSensor();
void Forward();
void PID();
void CalcTurn();
void MotorDrive(int left_speed, int right_speed);
void Turn_right();
void Turn_left();
void Turn_180();


void UpdatePosition();
void Action();
void UpdateOrientation_r();
void UpdateOrientation_l();

/************* init ************/
void setup()
{
  pinMode(Right, OUTPUT);
  pinMode(Left, OUTPUT);
  pinMode(M1_r, OUTPUT);
  pinMode(M1_b, OUTPUT);
  pinMode(M2_r, OUTPUT);
  pinMode(M2_b, OUTPUT); 

  for(int i = 0; i<5; i++)
  {
    pinMode(sensor[i], INPUT); 
  }
  pinMode(arm_en, OUTPUT);
  pinMode(arm_r, OUTPUT);
  pinMode(arm_b, OUTPUT);
  
  pinMode(gripper_r, OUTPUT);
  pinMode(gripper_b, OUTPUT);
  pinMode(gripper_en, OUTPUT);
  
  pinMode(Left_sensor, INPUT);
  pinMode(Right_sensor, INPUT);
  pinMode(Front_sensor, INPUT);
  pinMode(buzz, OUTPUT);
  attachInterrupt(3, Check_Wing, LOW);

}


/************* Main loop *************/
void loop()
{
  Dry_run = EEPROM.read(100);
  if(Dry_run)
  {
	PID_Motion();
  }
  else
  {
    if(flag_read)
    {
      Set_Map();
      Calc_Target();
      for(int i = 0; i < 8; i++)
      {
        BFS(current_x, current_y, target_x[i], target_y[i]);    
        current_x = target_x[i];
        current_y = target_y[i];  
      }
      flag_read = false; 
    }
    
    CheckSensor();
    if(sensor_read[4] && sensor_read[3] && sensor_read[2] && sensor_read[1] && sensor_read[0])
    {
      delay(200);
      UpdatePosition();
      next_coor();
      Action();    
    }
    else
    {
      Forward(); 
    }
    
    Tasks();
  }	
}
/************ Functions ***************/

void CheckSensor()
{
  sensor_sum = 0;
  for(int i=0; i<5; i++)
  {
    sensor_read[i] = digitalRead(sensor[i]);
    sensor_sum += sensor_read[i];
  }

  Left_read = digitalRead(Left_sensor);
  Right_read = digitalRead(Right_sensor);
  Front_read = digitalRead(Front_sensor);
}


void PID()
{
  int error_right = 0;
  int error_left = 0;

  for(int i =0 ; i <= 2; i++)
  {
    if(sensor_read[i] == 0)
    {
      error_right += 1;
    }
  }
  for(int i = 4; i>= 2; i--)
  {
    if(sensor_read[i] == 0)
    {
      error_left += 1;
    }
  }

  if(error_right > error_left)
  {
    pos = error_right + error_left;
  }
  else if(error_right < error_left)
  {
    pos = (error_right + error_left) * (-1);
  }
  else if ((error_right == error_left) && (sensor_sum == 3))
  {
    pos = 0;
  }
  if (sensor_sum != 0)
  {
    proportional = pos;
    integral += proportional;
    derivative = (proportional - last);
    last = proportional;

    if(integral >= avg_speed)
    {
      integral = avg_speed; 
    }
    if(integral <= - avg_speed)
    {
      integral = - avg_speed;
    }

    error_val = proportional * Kp + integral * Ki + derivative * Kd;

    if (error_val <= - avg_speed)
    {
      error_val = - avg_speed;
    }

    if(error_val >= avg_speed)
    {
      error_val = avg_speed;
    }
  }
  else
  {
    if(last < 0)
    {
      error_val = -avg_speed;
    }
    else
    {
      error_val = avg_speed;
    }
  }
}

void CalcTurn()
{
  if(error_val >= 0)
  {
    right_speed = avg_speed;
    left_speed = avg_speed - error_val;
  }
  else
  {
    right_speed = avg_speed + error_val;
    left_speed = avg_speed;
  }
}

void MotorDrive(int left_speed, int right_speed)
{
  digitalWrite(M1_r, HIGH);
  digitalWrite(M2_r, HIGH);
  digitalWrite(M1_b, LOW);
  digitalWrite(M2_b, LOW);

  analogWrite(Right, right_speed);
  analogWrite(Left, left_speed);

}

void Forward()
{
  CheckSensor();
  PID();
  CalcTurn();
  MotorDrive(left_speed, right_speed);
}

void Turn_right()
{ 
  UpdateOrientation_r();
  digitalWrite(M1_b, HIGH);
  digitalWrite(M2_r, HIGH);
  digitalWrite(M1_r, LOW);
  digitalWrite(M2_b, LOW);
  analogWrite(Right, 85); //110-20=90
  analogWrite(Left, 85); //110 - 50 = 70

  delay(200);

  while(1)
  {

    CheckSensor();
    if(!sensor_read[0] && sensor_read[1] && sensor_read[2] && sensor_read[3])
    {
      break;
    }

  }  
}

void Turn_left()
{
  UpdateOrientation_l();
  digitalWrite(M1_r, HIGH);
  digitalWrite(M2_b, HIGH);
  digitalWrite(M1_b, LOW);
  digitalWrite(M2_r, LOW);

  analogWrite(Right, 85); // 90
  analogWrite(Left, 85); // 70
  delay(200);

  while(1)
  {
    CheckSensor();
    if(!sensor_read[4] && sensor_read[3] && sensor_read[2] && sensor_read[1])
    {
      break;
    }
  }
}

void Turn_180()
{
  Turn_right();
  Turn_right(); 
}

void UpdatePosition()
{
  switch(orientation)
  {
  case 'N':
    pos_y++;

    break;

  case 'E':
    pos_x++;

    break;

  case 'S':
    pos_y--;

    break;

  case 'W':
    pos_x--;

    break;
  }  

}

void Action()
{       
  if((pos_x == next_x) && (pos_y != next_y) && (main_dir == 'y'))
  {
    if(pos_y > next_y && orientation == 'N')
    {
      Turn_180(); 
    }
    else if(pos_y < next_y && orientation == 'S')
    {
      Turn_180(); 
    }
    else
    {
      return; 
    }
  }
  else if((pos_x != next_x) && (pos_y == next_y) && (main_dir == 'x'))
  {
    if(pos_x > next_x && orientation == 'E')
    {
      Turn_180(); 
    }
    else if(pos_x < next_x && orientation == 'W')
    {
      Turn_180(); 
    }
    else
    {
      return; 
    }
  }

  else if((pos_x != next_x) && (pos_y == next_y) && (main_dir == 'y'))
  {
    if(next_x > pos_x)
    {
      if(orientation == 'N')
      {
        Turn_right();
      }
      else if(orientation == 'S')
      {
        Turn_left();
      }
    }
    else
    {
      if(orientation == 'N')
      {
        Turn_left();
      }
      else if(orientation == 'S')
      {
        Turn_right();
      }
    }
    main_dir = 'x';
    return;
  }
  else if((pos_x == next_x) && (pos_y != next_y) && (main_dir == 'x'))
  {
    if(next_y > pos_y)
    {
      if(orientation == 'E')
      {
        Turn_left();
      }
      else if(orientation == 'W')
      {
        Turn_right();
      }
    }
    else
    {
      if(orientation == 'E')
      {
        Turn_right();
      }
      else if(orientation == 'W')
      {
        Turn_left();
      }

    }
    main_dir = 'y';
    return;
  }
  else
  {
    Turn_180(); 
  }

}

void UpdateOrientation_r()
{
  switch (orientation)
  {
  case 'N':
    orientation = 'E';

    break;

  case 'E':
    orientation = 'S';

    break;

  case 'S':
    orientation = 'W';

    break;

  case 'W':
    orientation = 'N';

    break;

  }
}

void UpdateOrientation_l()
{
  switch (orientation)
  {
  case 'N':
    orientation = 'W';

    break;

  case 'E':
    orientation = 'N';

    break;

  case 'S':
    orientation = 'E';

    break;

  case 'W':
    orientation = 'S';

    break;

  }
}

void next_coor()
{
  int row, col, z;
  if(!path.isEmpty())
  { 
    z = path.pop();
    zToRowCol( z, row, col );
    next_x = row;
    next_y = col;
    if(next_x == pos_x && next_y == pos_y)
    {
      z = path.pop();
      zToRowCol( z, row, col );
      next_x = row;
      next_y = col;
    }
  }
  else
  {
    Stop();  
  }
}


void Stop()
{
  while(true)
  {
    analogWrite(Right, 0);
    analogWrite(Left, 0);
  }
}
void Wait()
{
    analogWrite(Right, 0);
    analogWrite(Left, 0); 
    delay(1000);
}


void Snail()
{
  switch (pos_y)
  {
  case 0:
    if(pos_x == 1 || pos_x == 4)
    {
      Turn_right();
      break; 
    }
    else
    {
      delay(50);
      break; 
    }

  case 1:
    if(pos_x == 2 || pos_x == 3)
    {
      Turn_right();
      break; 
    }
    else
    {
      delay(50);
      break; 
    }
  case 6:
    if(pos_x == 2)
    {
      //Calc_Target();
      Save_toepp();
      Stop();
      
    }
    else
    {
      delay(50);
      break; 
    }
  case 7:
    if(pos_x == 1 || pos_x == 3)
    {
      Turn_right();
      break; 
    }
    else
    {
      delay(50);
      break; 
    }
  case 8:
    if(pos_x == 0 || pos_x == 4)
    {
      Turn_right(); 
      break;
    }

    else
    {
      delay(50);
      break;
    }



  default:
   break;
  } 
}

//void Set_state()
//{ 
//  CheckSensor();
//  if(Right_read && Left_read) 
//  {
//    if(Front_read)
//    {
//      flag_target = 0;
//      digitalWrite(buzz, HIGH);
//      while(!(!Front_read && !Right_read && !Left_read))
//      {
//        CheckSensor();
//        analogWrite(Right,avg_speed);
//        analogWrite(Left,avg_speed);
//      }
//      digitalWrite(buzz,LOW);
//    }
//    else
//    {
//      flag_black = 0;
//      digitalWrite(buzz, HIGH);
//      delay(50);
//      digitalWrite(buzz, LOW); 
//    }
//  }
//}

void Save_toepp()
{        
  int address = 0;
  for(int i =0; i<5; i++)
  {
    for(int j = 0; j < 9; j++)
    {
      if(Filled[i][j] == 0)
      {
        EEPROM.write(address, Filled[i][j]);
      }
      address++;
    }
  }
  EEPROM.write(100, 0);
  EEPROM.write(110, target_findx[0]);
  EEPROM.write(111, target_findy[0]);
  EEPROM.write(112, target_findx[1]);
  EEPROM.write(113, target_findy[1]);
}

void Set_Map()
{
  int address = 0;
  int check;
  for(int i = 0; i < 5; i++)
  {
    for(int j = 0; j < 9; j++)
    {
      check = EEPROM.read(address);
      if(check == 0)
      {
        Filled[i][j] = 0;
      }
      address = address + 1;
    }
  }
}

//void Set_Filled()
//{
//  if(flag_black == 0)
//  {
//     Filled[pos_x][pos_y] = 0;
//  } 
//
//  if(flag_target == 0)
//  {
//    target_findx[ptr] = pos_x;
//    target_findy[ptr] = pos_y;
//    ptr++;
//  }
//  
//  flag_black = 1;
//  flag_target = 1;
//}

void Calc_Target()
{
  int target_one = 0;
  int target_two = 0;
  int temp_x, temp_y;
  
  target_findx[0] = EEPROM.read(110);
  target_findy[0] = EEPROM.read(111);
  target_findx[1] = EEPROM.read(112);
  target_findy[1] = EEPROM.read(113);
  
  BFS(0, 0, target_findx[0], target_findy[0]);
  while(!(path.isEmpty()))
  {
   path.pop();
   target_one = target_one + 1;
  } 
  
  BFS(0, 0, target_findx[1], target_findy[1]);
  while(!(path.isEmpty()))
  {
  path.pop();
  target_two = target_two + 1; 
  }
 
 if(target_one > target_two)
 {
   temp_x = target_findx[0];
   temp_y = target_findy[0];
   target_findx[0] = target_findx[1];
   target_findy[0] = target_findy[1];
   target_findx[1] = temp_x;
   target_findy[1] = temp_y;
 }
 
 target_x[0] = target_findx[0];
 target_y[0] = target_findy[0];
 target_x[3] = target_findx[1];
 target_y[3] = target_findy[1];
}

void BFS(int curr_x, int curr_y, int tar_x, int tar_y) {
  QueueList <int> queue;

  // Clean the array
  initializeFloodfill();
  initPath();

  int start = rowColtoZ(tar_x, tar_y);
  int target = rowColtoZ(curr_x, curr_y);

  visited[start] = 1;
  // Starting point
  queue.push(start);

  while(!queue.isEmpty()) 
  {
    int x = queue.pop();
    int z = x & 255;
    int row, col;
    zToRowCol( z, row, col );
    int val = (x >> 8) & 255;
    int curVal = floodfillArr[ z ];


    if(curr_x == row && curr_y == col) break;

    // Floodfill this cell
    if((curVal != UNDEFINED && curVal <= val) || z < 0 || z > AREA) { 
      continue; 
    }
    floodfillArr[ z ] = val;

    if((col+1) < 9)
    {
      if(Filled[row][col + 1] == 1)
      {
        temp = rowColtoZ(row, col + 1);
        if (!passed(temp))
        {
          queue.push( temp | ((val + 1) << 8) );
          visited[temp] = z;
        }
      }
    }
    if((row+1) < 5)
    {
      if(Filled[row + 1][col] == 1)
      {	
        temp = rowColtoZ(row + 1, col);
        if(!passed(temp))
        {
          queue.push( temp | ((val + 1) << 8) );
          visited[temp] = z;
        }
      }
    }
    if((col - 1) >= 0)
    {
      if(Filled[row][col - 1] == 1)
      {
        temp = rowColtoZ(row, col - 1);
        if (!passed(temp))
        {
          queue.push( temp | ((val + 1) << 8) );
          visited[temp] = z;
        }
      }
    }
    if((row - 1) >= 0)
    {
      if(Filled[row - 1][col] == 1)
      {
        temp = rowColtoZ(row - 1, col);
        if (!passed(temp))
        {
          queue.push( temp | ((val + 1) << 8) );
          visited[temp] = z;
        }
      }
    }
  }
  int curr = target;
  path.push(curr);
  while(floodfillArr[curr])
  {
    if(floodfillArr[curr] != 1)
    {
      curr = visited[curr];
      path.push(curr); 
    }
    else
    {
      path.push(start);
      break; 
    }
  }

}


// X/Y coordiante linearizing methods
int rowColtoZ(int row, int col) {
  return abs(LENGTH * row + col);
}

void zToRowCol(int z, int &row, int &col) {
  row = floor(z / LENGTH);
  col = z % LENGTH;
}

void initializeFloodfill() {
  for(int i = 0; i < AREA; i++) {
    floodfillArr[i] = UNDEFINED;
  }
}


void initPath()
{
  for(int i = 0; i < AREA; i++)
  {
    visited[i] = 0;
  } 
}

boolean passed(int temp)
{
  if (visited[temp]) return true;
  else return false;
}


void Arm_up()
{
  digitalWrite(arm_r, HIGH);
  digitalWrite(arm_b, LOW);
  analogWrite(arm_en, 200);
  delay(2000);
  analogWrite(arm_en,0 );
  delay(250);
  digitalWrite(arm_r, LOW);
  digitalWrite(arm_b, HIGH);
  analogWrite(arm_en, 90);
  delay(100);
  analogWrite(arm_en,0 );
}
void Arm_down()
{
  digitalWrite(arm_r, LOW);
  digitalWrite(arm_b, HIGH);
  analogWrite(arm_en, 90); 
  delay(1800);
  analogWrite(arm_en, 0); 
  delay(150);
  digitalWrite(arm_r, HIGH);
  digitalWrite(arm_b, LOW);
  analogWrite(arm_en, 90); 
  delay(50);
  analogWrite(arm_en, 0);
}
void Gripper_open()
{
  digitalWrite(gripper_r, LOW);
  digitalWrite(gripper_b, HIGH);
  analogWrite(gripper_en, 255); 
  delay(500);
  digitalWrite(gripper_en, LOW); 
}
void Gripper_close()
{
  digitalWrite(gripper_r, HIGH);
  digitalWrite(gripper_b, LOW);
  analogWrite(gripper_en, 255);
  delay(1000);
  //analogWrite(gripper_en, 0); 
}
void Enter_District()
{
  boolean district = true;
  int counter = 2;
 CheckSensor();
 while(district)
 {
  if(sensor_sum == 5)
  {
   counter--;
  delay(250); 
  }
  else
  {
   Forward(); 
  }
  
  if(counter < 1)
  {
   district = false;
  }   
 }
}
void Back()
{
  digitalWrite(M1_r, LOW);
  digitalWrite(M2_r, LOW);
  digitalWrite(M1_b, HIGH);
  digitalWrite(M2_b, HIGH);

  analogWrite(Right, 80);
  analogWrite(Left, 80); 
  delay(300);
  analogWrite(Right, 0);
  analogWrite(Left, 0); 

}
void Forwards()
{
  digitalWrite(M1_r, HIGH);
  digitalWrite(M2_r, HIGH);
  digitalWrite(M1_b, LOW);
  digitalWrite(M2_b, LOW);

  analogWrite(Right, 80);
  analogWrite(Left, 80); 
  delay(250);
  analogWrite(Right, 0);
  analogWrite(Left, 0); 

}
void Adjust()
{
  analogWrite(Right, 0);
  analogWrite(Left, 0); 
  delay(500);
  CheckSensor();
  if(sensor_read[0])
  {
  while(1)
  {
    CheckSensor();
    digitalWrite(M1_r, LOW);
    digitalWrite(M1_b, HIGH);
    analogWrite(Right, 60);
    if(sensor_read[1] && sensor_read[2] && sensor_read[3])
     {
      break;
     } 
  }
  }
  
  if(sensor_read[4])
  {
    while(1)
  {
    CheckSensor();
    digitalWrite(M2_r, LOW);
    digitalWrite(M2_b, HIGH);
    analogWrite(Right, 60);
    if(sensor_read[1] && sensor_read[2] && sensor_read[3])
     {
      break;
     } 
  }
  }
}
void Tasks()
{
  if(next_x == target_x[task_ptr] && next_y == target_y[task_ptr])
  {
    switch (task_ptr)
    {
    case 0:
    //Adjust();
      Wait();
      Back();
      // lower arm
      Gripper_open();
      
      // open servo
      delay(100);
      Arm_down();
      delay(100);
      Forwards();
      delay(100);
      // close servo
      Gripper_close();
      delay(100);
      // raise arm
      Arm_up();
      delay(100);
      //Stop();
      task_ptr++;
      break;

    case 1:
    //Adjust();
      Wait();
      // delay kbir fash5 practice
      //Back();
      //Stop();
      task_ptr++;
      break;

    case 2:
    //Adjust();
      Wait();
     // Enter_District();
      delay(2000);
      Gripper_open();
      delay(1000);
      Arm_down();
      delay(5000);
      Stop();
      //harga3 marreten bedahri
      // center position
      // move
      // open servo
      // lower the arm
      //Back();
      //Arm_down();
      // delay || limit switch
      //delay(500);
      // raise arm
      //Arm_up();
      // out 
      task_ptr++;
      break;
    case 3:
    //Adjust();
      Wait();
      Gripper_open();
      delay(100);
      Arm_down();
      delay(100);
      Gripper_close();
      delay(100);
      Arm_up();
      delay(100);
      // center
      // lower the arm
      //Back();
      //Arm_down();
      // open servo
      // close servo
      //delay(500);
      // raise arm
      //Arm_up();
      task_ptr++;
      break;

    case 4:
    //Adjust();
      Wait();
      delay(1000); 
      // out of the land
      // Turn 
      // delay || practice
      // lower arm
      //Back();
      //Arm_down();
      // open servo
      //delay(500);
      // raise arm
      //Arm_down();
      // move along to open hinge
      // mission complete
      task_ptr++;
      break;
      
      
     case 5:
     Arm_down();
      delay(100);
     Gripper_open();
      delay(100);
    default:
      break;
    }
  } 
  else
  {
    return; 
  }
}
/********************************* Ismail Functions ***************************/
void calculateProportional(){ 
  int sum = 0; 
  int posLeft = 0; 
  int posRight = 0; 
   //readValues();
  for (int i=0; i<= 2; i++)
  { 
    sum=sum+sensor_read[i]; 
    if(sensor_read[i] == 0){ 
      posRight = posRight + 1;
    } 
  } 
  for (int i = (4); i >= 2; i--)
  { 
    sum=sum+sensor_read[i]; 
    if(sensor_read[i] == 0){ 
      posLeft = posLeft + 1; 
    } 
  }
 
  if((sum == 6) || (Left_read && Right_read))
  {
    delay(200);
    TakeAction();
	//Set_Filled();
    if(NFlag)
    {
      Filled[pos_x][pos_y] = 0;
      NFlag = false;
    }
//    else if(TFlag)
//    {
//      target_findx[ptr] = pos_x;
//      target_findy[ptr] = pos_y;
//      TFlag = false;
//      ptr++;
//    }
    if((pos_x == 2) && (pos_y == 6))
    {
      Save_toepp();
      digitalWrite(buzz,HIGH);
      delay(150);
      digitalWrite(buzz,LOW);
	  Stop();
    }
  }

  else if(posRight > posLeft)
  {
	error = posRight + posLeft;
  }
  else if(posLeft > posRight)
  {
	error = (posLeft + posRight) * (-1);
  }
  else
  {
	if((posLeft == posRight) && (sum == 4))
	{
		error = 0;
	}
	else
	{
		if(lastError < 0)
		{
			error = -75;
		}
		else
		{
			error = 75;
		}
	}
  }
  proportional = error; 
}

void PID_Motion()
{
  CheckSensor(); 
  calculateProportional(); 

  derivative = error-lastError; 
  integral += proportional; 

  if(integral > avg_speed){ 
    integral= avg_speed; 
  }; 
  if(integral < -avg_speed){ 
    integral = -avg_speed; 
  }; 

  float turn = proportional*Kp + derivative*Kd + integral*Ki; 

  if(turn >= avg_speed)
  { 
    turn = avg_speed; 
  }
  if(turn <= -avg_speed) 
  {
    turn = -avg_speed; 
  }
 

  if(turn >= 0){ 
    right_speed = avg_speed; 
    left_speed = avg_speed - turn; 
  } 
  else{ 
    right_speed = avg_speed + turn; 
    left_speed = avg_speed; 
  } 

  digitalWrite (M1_r, HIGH);
  digitalWrite (M1_b, LOW);
  digitalWrite (M2_r, HIGH);
  digitalWrite (M2_b, LOW);

  analogWrite(Right, right_speed); 
  analogWrite(Left, left_speed); 

  lastError=error;    
}

void TakeAction()
{
   CheckOrintation();
   UpdateCoordinate();
   CheckForTurn(); 
}

char CheckOrintation()
{
    return orintation;
}

void UpdateCoordinate()
{
    switch(CheckOrintation())
    {
      case 'N':
        pos_y++;
        
        break;
      case 'S':
        pos_y--;
        break;
      case 'E':
        pos_x++;
        break;
      case 'W':
        pos_x--;
        break;
    }
}

void CheckForTurn()
{
   switch(pos_y)
  {
      case 8:
        if((pos_x == 4) || (pos_x == 0))
        {
            TurnRight();
        }
        break;
      case 0:
        if((pos_x == 4)||(pos_x == 1))
        {
            TurnRight();
        } 
        break;
      case 7:
			if((pos_x == 1) || (pos_x == 3))
			{
				TurnRight();
			}
			break;
		case 1:
			if((pos_x == 3) || (pos_x == 2))
			{
				TurnRight();
			}
			break;			
      default:
        break;
  } 
}

void TurnRight(){
    analogWrite(Right,0);
    analogWrite(Left,0);
    delay(200);
    digitalWrite (M1_r, LOW);
    digitalWrite (M1_b, HIGH);
    digitalWrite (M2_r, HIGH);
    digitalWrite (M2_b, LOW); 
    analogWrite(Right,90);
    analogWrite(Left,80);
    delay(50);
    while(FlagR)
    {
      CheckSensor();
      if(!sensor_read[4] && !sensor_read[3] && !sensor_read[2] && sensor_read[1] && sensor_read[0])
      {
        FlagR = false;
      }
      
    }

    //error = -5;
    Direction = 'R';
    UpdateOrintation();
    FlagR = true;
}

// Declearation TurnLeft
void TurnLeft()
{  
    digitalWrite (M1_r, HIGH);
    digitalWrite (M1_b, LOW);
    digitalWrite (M2_r, LOW);
    digitalWrite (M2_b, HIGH);
    analogWrite(Right,95);
    analogWrite(Left,85);
    delay(50);
    while(FlagL)
    {
     CheckSensor();
      if(sensor_read[4] && sensor_read[3] && !sensor_read[2] && !sensor_read[0]&& !sensor_read[1])
      {
        FlagL = false;
      }
      
    }
    FlagL = true;
    Direction = 'L';
    UpdateOrintation();
}

void UpdateOrintation()
{
    switch(CheckOrintation())
    {
        case 'N':
          if(Direction == 'R')
          {
              orintation = 'E';
          }
          else if(Direction == 'L')
          {
              orintation = 'W';
          }
          break;
        case 'S':
          if(Direction == 'R')
          {
              orintation = 'W';
          }
          else if(Direction == 'L')
          {
              orintation = 'E';
          }
          break;
        case 'W':
          if(Direction == 'R')
          {
              orintation = 'N';
          }
          else if(Direction == 'L')
          {
              orintation = 'S';
          }
          break;
        case 'E':
          if(Direction == 'R')
          {
              orintation = 'S';
          }
          else if(Direction == 'L')
          {
              orintation = 'N';
          }
          break;
        default:
          break;
    }
}

void Check_Wing()
{
     CheckSensor();
     if((Left_read == HIGH) && (Right_read == HIGH))
     {
       
       if(Front_read == HIGH)
       {
         digitalWrite(buzz,HIGH);
         while(!((Right_read == LOW) && (Left_read == LOW) && (Front_read == LOW)))
         {
	    CheckSensor();
            Push();
         }
         digitalWrite(buzz,LOW);
         if(Dry_run)
         {
           TakeAction();
           target_findx[ptr] = pos_x;
           target_findy[ptr] = pos_y;
           ptr++;
         }
       }
       else
       {
         digitalWrite(buzz,HIGH);
         delay(150);
         digitalWrite(buzz,LOW);
         NFlag = true;
       }
     }

}
void Push()
{
  analogWrite(Right,75);
  analogWrite(Left,75);
}
